"""ESP32 ESC Controller Client — Kommunikation per WebSocket über WiFi."""

import json
import threading
import time
import websocket


class ESP32Client:
    """Verbindet sich per WebSocket zum ESP32 und sendet Steering/Throttle-Befehle."""

    def __init__(self, host="192.168.4.1", port=80, hostname="rc-esc.local", on_status=None, on_imu=None):
        self.host = host
        self.hostname = hostname
        self.port = port
        self.url = None  # Wird beim Connect bestimmt
        self.on_status = on_status  # Callback für Status-Updates
        self.on_imu = on_imu        # Callback für IMU-Daten

        self.ws = None
        self.connected = False
        self.armed = False
        self.platform = "unknown"
        self._drive_mode = 0
        self._imu_available = False

        self._thread = None
        self._running = False
        self._reconnect_delay = 1.0
        self._lock = threading.Lock()

        # Rate-Limiting: max alle 20ms senden (50Hz)
        self._last_send_time = 0
        self._min_send_interval = 0.020  # 20ms = 50Hz

        # Letzte gesendete Werte (Deduplizierung)
        self._last_steering = None
        self._last_throttle = None

        # Pending-Werte (werden beim nächsten Send-Slot gesendet)
        self._pending_steering = None
        self._pending_throttle = None
        self._send_thread = None

    def connect(self):
        """Startet die WebSocket-Verbindung in einem Hintergrund-Thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        # Separater Thread zum Rate-Limited Senden
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

    def disconnect(self):
        """Trennt die Verbindung."""
        self._running = False
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass
        self.connected = False

    def _run_loop(self):
        """Reconnect-Loop — verbindet sich automatisch neu."""
        while self._running:
            try:
                self._connect_ws()
            except Exception as e:
                print(f"[ESP32] Verbindungsfehler: {e}")
            self.connected = False
            if self._running:
                time.sleep(self._reconnect_delay)

    def _connect_ws(self):
        """Baut die WebSocket-Verbindung auf. Versucht erst Hostname, dann feste IP."""
        import socket

        # Erst versuchen über Hostname (mDNS) zu verbinden
        resolved_host = None
        if self.hostname:
            try:
                resolved_host = socket.gethostbyname(self.hostname)
                print(f"[ESP32] Hostname '{self.hostname}' aufgelöst → {resolved_host}")
            except socket.gaierror:
                print(f"[ESP32] Hostname '{self.hostname}' nicht gefunden, Fallback auf {self.host}")

        target = resolved_host if resolved_host else self.host
        self.url = f"ws://{target}:{self.port}/ws"
        print(f"[ESP32] Verbinde zu {self.url}...")

        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        self.ws.run_forever(
            ping_interval=10,
            ping_timeout=5,
            skip_utf8_validation=True,
        )

    def _on_open(self, ws):
        self.connected = True
        print(f"[ESP32] Verbunden mit {self.host}")

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            if msg_type == "status":
                self.armed = data.get("armed", False)
                self.platform = data.get("platform", "unknown")
                self._drive_mode = data.get("mode", 0)
                self._imu_available = data.get("imuAvailable", False)
                if self.on_status:
                    self.on_status(data)
            elif msg_type == "ack":
                self.armed = data.get("armed", False)
                self._drive_mode = data.get("mode", self._drive_mode)
            elif msg_type == "imu":
                if self.on_imu:
                    self.on_imu(data)
            elif msg_type == "failsafe":
                print(f"[ESP32] FAILSAFE: {data.get('message')}")
                self._last_steering = None
                self._last_throttle = None
        except (json.JSONDecodeError, Exception):
            pass

    def _on_error(self, ws, error):
        if self._running:
            print(f"[ESP32] WS Fehler: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        self.connected = False
        if self._running:
            print("[ESP32] Verbindung getrennt — Reconnect...")

    def _send_raw(self, data: dict):
        """Sendet JSON über WebSocket (Thread-safe)."""
        if not self.connected or not self.ws:
            return False
        try:
            with self._lock:
                self.ws.send(json.dumps(data))
            return True
        except Exception:
            self.connected = False
            return False

    def _send_loop(self):
        """Sendet pending Werte mit Rate-Limiting (50Hz max) + Heartbeat."""
        heartbeat_interval = 0.5  # Alle 500ms den aktuellen Wert erneut senden
        last_heartbeat = 0

        while self._running:
            now = time.time()
            if now - self._last_send_time >= self._min_send_interval:
                sent = False
                # Steering senden wenn geändert
                if self._pending_steering is not None:
                    val = self._pending_steering
                    self._pending_steering = None
                    if val != self._last_steering:
                        self._last_steering = val
                        self._send_raw({"cmd": "steer", "value": val})
                        sent = True
                # Throttle senden wenn geändert
                if self._pending_throttle is not None:
                    val = self._pending_throttle
                    self._pending_throttle = None
                    if val != self._last_throttle:
                        self._last_throttle = val
                        self._send_raw({"cmd": "throttle", "value": val})
                        sent = True

                # Heartbeat: aktuelle Werte erneut senden damit ESP32 Failsafe nicht greift
                if not sent and now - last_heartbeat >= heartbeat_interval:
                    if self._last_steering is not None:
                        self._send_raw({"cmd": "steer", "value": self._last_steering})
                    if self._last_throttle is not None:
                        self._send_raw({"cmd": "throttle", "value": self._last_throttle})
                    last_heartbeat = now

                if sent:
                    self._last_send_time = now
                    last_heartbeat = now
            time.sleep(0.005)  # 5ms sleep, 200Hz check-rate

    # ============================
    # Public API
    # ============================

    def set_steering(self, value: float):
        """Setzt Lenkung. value: -1.0 bis 1.0"""
        int_val = int(max(-1000, min(1000, value * 1000)))
        self._pending_steering = int_val

    def set_throttle(self, value: float):
        """Setzt Throttle. value: -1.0 bis 1.0"""
        int_val = int(max(-1000, min(1000, value * 1000)))
        self._pending_throttle = int_val

    def arm(self):
        """ESC armen."""
        self._send_raw({"cmd": "arm"})

    def disarm(self):
        """ESC disarmen."""
        self._send_raw({"cmd": "disarm"})
        self._last_throttle = None
        self._last_steering = None

    def set_platform(self, platform_id: int):
        """ESC-Plattform wechseln (0=Traxxas, 1=Hobbywing, 2=Generic)."""
        self._send_raw({"cmd": "platform", "value": platform_id})

    def set_drive_mode(self, mode: int):
        """Drive-Modus wechseln (0=Direct, 1=Stability, 2=Autopilot)."""
        self._send_raw({"cmd": "mode", "value": mode})

    def set_stability_params(self, params: dict):
        """Stability Assist Parameter setzen."""
        msg = {"cmd": "stability_params"}
        msg.update(params)
        self._send_raw(msg)

    def set_autopilot_params(self, params: dict):
        """Drift Autopilot Parameter setzen."""
        msg = {"cmd": "autopilot_params"}
        msg.update(params)
        self._send_raw(msg)

    @property
    def status(self) -> dict:
        return {
            "connected": self.connected,
            "armed": self.armed,
            "platform": self.platform,
            "host": self.host,
            "drive_mode": self._drive_mode,
            "imu_available": self._imu_available,
        }
