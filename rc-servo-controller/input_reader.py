"""Reads USB steering wheel input via evdev."""

import threading
import evdev
from evdev import ecodes


def list_devices():
    """Gibt alle verfügbaren Input-Geräte zurück."""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    result = []
    for dev in devices:
        caps = dev.capabilities(verbose=False)
        axes = []
        if ecodes.EV_ABS in caps:
            for axis_code, info in caps[ecodes.EV_ABS]:
                axis_name = ecodes.ABS.get(axis_code, f"ABS_{axis_code}")
                axes.append({"code": axis_code, "name": axis_name,
                             "min": info.min, "max": info.max})
        result.append({
            "path": dev.path,
            "name": dev.name,
            "phys": dev.phys,
            "axes": axes,
        })
        dev.close()
    return result


class InputReader(threading.Thread):
    def __init__(self, device_path: str, axis_callback=None):
        super().__init__(daemon=True)
        self.device_path = device_path
        self.axis_callback = axis_callback
        self.running = False
        self.device = None
        self.axis_states = {}
        self.axis_info = {}

    def open_device(self):
        self.device = evdev.InputDevice(self.device_path)
        caps = self.device.capabilities(verbose=False)
        if ecodes.EV_ABS in caps:
            for axis_code, info in caps[ecodes.EV_ABS]:
                self.axis_info[axis_code] = {"min": info.min, "max": info.max}
                self.axis_states[axis_code] = 0.0

    def normalize(self, axis_code: int, value: int) -> float:
        """Normalisiert Achsenwert.
        Lenkachse (symmetrisch): -1.0 bis 1.0 (Mitte = 0)
        Pedale (einseitig, min=0): 0.0 bis 1.0 (losgelassen=0, gedrückt=1)
        """
        info = self.axis_info.get(axis_code)
        if not info:
            return 0.0
        amin = info["min"]
        amax = info["max"]
        arange = amax - amin
        if arange == 0:
            return 0.0

        # Pedale erkennen: wenn min=0 und max=255 (oder ähnlich klein)
        # → einseitige Achse, 0.0 (losgelassen) bis 1.0 (gedrückt)
        mid = (amin + amax) / 2.0
        # Heuristik: bei Pedalen ist der Idle-Wert am oberen Ende (max)
        # evdev gibt den Rohwert: max = losgelassen, min = gedrückt
        if amin == 0 and amax <= 255:
            # Pedal: invertiert (max=los, min=gedrückt) → 0.0 bis 1.0
            return max(0.0, min(1.0, (amax - value) / arange))

        # Zentrierte Achse (Lenkung): -1.0 bis 1.0
        half_range = arange / 2.0
        return max(-1.0, min(1.0, (value - mid) / half_range))

    def run(self):
        self.open_device()
        self.running = True
        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                if event.type == ecodes.EV_ABS:
                    normalized = self.normalize(event.code, event.value)
                    self.axis_states[event.code] = normalized
                    if self.axis_callback:
                        self.axis_callback(event.code, normalized)
        except (OSError, IOError):
            self.running = False

    def stop(self):
        self.running = False
        if self.device:
            try:
                self.device.close()
            except Exception:
                pass
