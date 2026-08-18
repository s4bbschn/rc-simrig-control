"""Force Feedback Controller — Berechnet Lenkrad-FFB aus IMU-Daten des RC-Cars."""

import threading
import time
import struct
import os

# evdev für Force Feedback (nur Linux)
try:
    import evdev
    from evdev import ecodes, ff
    HAS_EVDEV_FF = True
except ImportError:
    HAS_EVDEV_FF = False


class FFBController:
    """Empfängt IMU-Daten (Yaw-Rate, Lateral-G) und steuert Force Feedback am Lenkrad."""

    def __init__(self, config: dict):
        self.enabled = config.get("enabled", False)
        self.device_path = config.get("device_path", "")  # Lenkrad evdev Pfad
        self.yaw_gain = config.get("yaw_gain", 1.0)       # Stärke Yaw-FFB
        self.lateral_gain = config.get("lateral_gain", 0.5)  # Stärke Lateral-G FFB
        self.damping = config.get("damping", 0.3)          # Dämpfung (0=keine, 1=max)
        self.deadzone = config.get("deadzone", 2.0)        # Yaw-Deadzone (°/s)
        self.max_force = config.get("max_force", 0.8)      # Max Force (0.0-1.0)
        self.smoothing = config.get("smoothing", 0.3)      # Glättung (0=keine, 1=max)

        # Interner Zustand
        self._ff_device = None
        self._ff_effect_id = -1
        self._current_force = 0.0
        self._smoothed_force = 0.0
        self._last_yaw = 0.0
        self._last_lateral = 0.0
        self._current_steering = 0.0  # Aktueller Lenkwinkel (-1..1)
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

    def start(self):
        """Startet den FFB-Controller."""
        if not self.enabled:
            print("[FFB] Deaktiviert per Config")
            return False

        if not HAS_EVDEV_FF:
            print("[FFB] evdev nicht verfügbar (nur Linux)")
            return False

        if not self._open_ff_device():
            return False

        self._running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
        print("[FFB] Gestartet")
        return True

    def stop(self):
        """Stoppt den FFB-Controller und setzt Kraft auf 0."""
        self._running = False
        self._set_force(0.0)
        if self._ff_device and self._ff_effect_id >= 0:
            try:
                self._ff_device.erase_effect(self._ff_effect_id)
            except Exception:
                pass
        self._ff_effect_id = -1
        print("[FFB] Gestoppt")

    def update_config(self, config: dict):
        """Aktualisiert die FFB-Parameter live."""
        with self._lock:
            self.yaw_gain = config.get("yaw_gain", self.yaw_gain)
            self.lateral_gain = config.get("lateral_gain", self.lateral_gain)
            self.damping = config.get("damping", self.damping)
            self.deadzone = config.get("deadzone", self.deadzone)
            self.max_force = config.get("max_force", self.max_force)
            self.smoothing = config.get("smoothing", self.smoothing)

            new_enabled = config.get("enabled", self.enabled)
            if new_enabled and not self.enabled:
                self.enabled = True
                self.start()
            elif not new_enabled and self.enabled:
                self.enabled = False
                self.stop()

    def on_imu_data(self, yaw_rate: float, lateral_g: float, longitudinal_g: float = 0.0):
        """Callback: neue IMU-Daten vom ESP32 empfangen."""
        with self._lock:
            self._last_yaw = yaw_rate
            self._last_lateral = lateral_g

    def on_steering_input(self, value: float):
        """Callback: aktueller Lenkwinkel vom Input-Reader."""
        self._current_steering = value

    def _open_ff_device(self) -> bool:
        """Öffnet das Lenkrad als Force-Feedback-Gerät."""
        # Automatisch FF-fähiges Gerät finden
        device_path = self.device_path

        if not device_path:
            device_path = self._find_ff_device()

        if not device_path:
            print("[FFB] Kein Force-Feedback-Gerät gefunden!")
            return False

        try:
            self._ff_device = evdev.InputDevice(device_path)
            caps = self._ff_device.capabilities()

            if ecodes.EV_FF not in caps:
                print(f"[FFB] Gerät {device_path} unterstützt kein Force Feedback!")
                self._ff_device.close()
                self._ff_device = None
                return False

            # Constant Force Effekt erstellen
            effect = ff.Effect(
                ecodes.FF_CONSTANT,
                -1,  # id = -1 → neuer Effekt
                ff.Trigger(0, 0),
                ff.Replay(0, 0),  # unendlich
                ff.EffectType(ff_constant_ef=ff.Constant(level=0, envelope=ff.Envelope(0, 0, 0, 0)))
            )
            self._ff_effect_id = self._ff_device.upload_effect(effect)

            # Effekt aktivieren
            self._ff_device.write(ecodes.EV_FF, self._ff_effect_id, 1)
            print(f"[FFB] Gerät geöffnet: {self._ff_device.name} ({device_path})")
            return True

        except Exception as e:
            print(f"[FFB] Fehler beim Öffnen: {e}")
            return False

    def _find_ff_device(self) -> str:
        """Sucht automatisch nach einem FF-fähigen Gerät."""
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                caps = dev.capabilities()
                if ecodes.EV_FF in caps:
                    name = dev.name
                    dev.close()
                    if "logitech" in name.lower() or "force" in name.lower() or "wheel" in name.lower():
                        print(f"[FFB] FF-Gerät gefunden: {name} ({path})")
                        return path
                dev.close()
            except Exception:
                pass
        return ""

    def _update_loop(self):
        """Berechnet und setzt die Force alle 5ms (200Hz)."""
        while self._running:
            with self._lock:
                yaw = self._last_yaw
                lateral = self._last_lateral
                steering = self._current_steering

            force = self._calculate_force(yaw, lateral, steering)

            # Smoothing
            alpha = 1.0 - self.smoothing
            self._smoothed_force = self._smoothed_force * (1.0 - alpha) + force * alpha

            # Force setzen
            self._set_force(self._smoothed_force)

            time.sleep(0.005)  # 200Hz

    def _calculate_force(self, yaw_rate: float, lateral_g: float, steering: float) -> float:
        """Berechnet den Force-Wert aus IMU-Daten.
        
        Physik-Modell:
        1. Self-Aligning Torque: Wenn das Auto dreht (Yaw), will das Lenkrad 
           in den Gegeneinschlag → Kraft in Richtung der Yaw-Rotation
        2. Lateral-G: Querkraft drückt das Lenkrad zur Kurvenaußenseite
        3. Damping: Dämpft das aktuelle Lenkrad-Feedback proportional zum Lenkwinkel
        """
        # Deadzone für Yaw
        if abs(yaw_rate) < self.deadzone:
            yaw_rate = 0.0
        else:
            # Deadzone subtrahieren
            sign = 1.0 if yaw_rate > 0 else -1.0
            yaw_rate = sign * (abs(yaw_rate) - self.deadzone)

        # === Self-Aligning Torque ===
        # Yaw-Rate normalisieren (typisch max ~200°/s bei einem RC-Drift)
        yaw_normalized = yaw_rate / 200.0
        yaw_force = yaw_normalized * self.yaw_gain

        # === Lateral G Force ===
        # Laterale Beschleunigung (typisch max ~2g bei RC)
        lateral_normalized = lateral_g / 2.0
        lateral_force = lateral_normalized * self.lateral_gain

        # === Kombination ===
        total_force = yaw_force + lateral_force

        # === Damping ===
        # Lenkrad in der Mitte → weniger Damping, am Anschlag → mehr
        damping_force = -steering * self.damping * 0.3
        total_force += damping_force

        # Clamp
        total_force = max(-self.max_force, min(self.max_force, total_force))

        return total_force

    def _set_force(self, force: float):
        """Setzt die Constant-Force am Lenkrad. force: -1.0 bis 1.0"""
        if not self._ff_device or self._ff_effect_id < 0:
            return

        # evdev FF level: -32767 bis 32767
        level = int(force * 32767)
        level = max(-32767, min(32767, level))

        try:
            effect = ff.Effect(
                ecodes.FF_CONSTANT,
                self._ff_effect_id,
                ff.Trigger(0, 0),
                ff.Replay(0, 0),
                ff.EffectType(ff_constant_ef=ff.Constant(level=level, envelope=ff.Envelope(0, 0, 0, 0)))
            )
            self._ff_device.upload_effect(effect)
        except Exception:
            pass

    @property
    def status(self) -> dict:
        """Status für Web-UI."""
        return {
            "enabled": self.enabled,
            "active": self._running,
            "ff_device": self._ff_device.name if self._ff_device else None,
            "current_force": round(self._smoothed_force, 3),
            "yaw_rate": round(self._last_yaw, 1),
            "lateral_g": round(self._last_lateral, 3),
        }
