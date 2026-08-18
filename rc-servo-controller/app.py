"""RC Servo Controller — Flask Web-UI + Lenkrad-Input + Servo-Steuerung."""

import json
import os
import sys
import threading
from flask import Flask, render_template, request, jsonify

# --- Hardware-Erkennung ---

# ESP32 Client (WiFi-Steuerung)
HAS_ESP32 = False
esp32_client = None
try:
    from esp32_client import ESP32Client
    HAS_ESP32 = True
    print("[OK] ESP32 Client verfügbar")
except ImportError as e:
    print(f"[INFO] ESP32 Client nicht verfügbar: {e}")

# Servo-Steuerung (nur auf Raspberry Pi, als Fallback wenn kein ESP32)
HAS_GPIO = False
servo_ctrl = None
try:
    from servo_controller import ServoController
    import pigpio
    pi = pigpio.pi()
    if pi.connected:
        servo_ctrl = ServoController(pi)
        HAS_GPIO = True
        print("[OK] pigpio verbunden")
except Exception:
    print("[INFO] Kein pigpio — Servo-Output wird simuliert")

# Input-Reader: evdev (Linux) oder pywinusb HID (Windows)
HAS_INPUT = False
if sys.platform == "linux":
    try:
        from input_reader import InputReader, list_devices
        HAS_INPUT = True
        print("[OK] evdev Input")
    except Exception:
        pass
if not HAS_INPUT:
    try:
        from input_reader_hid import InputReader, list_devices
        devs = list_devices()
        HAS_INPUT = True
        print(f"[OK] HID Input ({len(devs)} Geräte)")
    except Exception as e:
        print(f"[WARN] Kein Input-System verfügbar — {e}")

# --- App Setup ---

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "config.json")
app = Flask(__name__)
config_lock = threading.Lock()
input_reader = None

# Live-Daten (werden vom Input-Thread geschrieben, von der API gelesen)
live_axes = {}
live_servo = {}
servo_log = []


# --- Config ---

_config_cache = None

def load_config():
    global _config_cache
    with open(CONFIG_FILE, "r") as f:
        _config_cache = json.load(f)
    return _config_cache

def save_config(cfg):
    global _config_cache
    _config_cache = cfg
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


# --- Servo-Berechnung ---

def compute_servo(value, s):
    """Berechnet Servo-Output nach Trim/Expo/Deadzone/Scale/Endpoints. Gibt dict zurück."""
    v = -value if s.get("invert") else value
    dz = s.get("deadzone", 0)
    if abs(v) < dz:
        v = 0.0
    expo = s.get("expo", 0)
    if expo > 0:
        sign = 1 if v >= 0 else -1
        v = sign * ((1 - expo) * abs(v) + expo * abs(v) ** 3)
    # Input-Skalierung: multipliziert den Input, sodass z.B. bei scale=4
    # bereits 25% Lenkradweg (90° bei 360° Rad) Vollausschlag ergibt.
    # Alles darüber wird durch die Endpoints geclampt.
    input_scale = s.get("input_scale", 1.0)
    if input_scale != 1.0:
        v = v * input_scale
    v = max(s.get("endpoint_low", -1), min(s.get("endpoint_high", 1), v))
    # Pulsweite
    trim = s.get("trim", 0)
    center = s.get("center_pulse", 1450) + trim
    min_p, max_p = s.get("min_pulse", 500), s.get("max_pulse", 2400)
    pulse = center + v * (max_p - center) if v >= 0 else center + v * (center - min_p)
    pulse = max(min_p, min(max_p, int(pulse)))
    return {"value": round(v, 3), "pulse": pulse, "input_raw": round(value, 3)}


def on_axis_input(axis_code, value):
    """Callback vom InputReader bei jeder Achsenbewegung."""
    live_axes[axis_code] = round(value, 3)
    cfg = _config_cache
    if not cfg:
        return

    # Steering: einfaches 1:1 Mapping
    s = cfg["steering"]
    mapped = s.get("input_axis")
    if mapped is not None and int(mapped) == int(axis_code):
        result = compute_servo(value, s)
        live_servo["steering"] = result
        # ESP32: Lenkwert senden
        if esp32_client and esp32_client.connected:
            esp32_client.set_steering(result["value"])
        # Lokaler Servo (Fallback)
        elif HAS_GPIO and servo_ctrl:
            servo_ctrl.set_position("steering", value, trim=s["trim"], expo=s["expo"],
                deadzone=s["deadzone"], invert=s["invert"],
                endpoint_low=s["endpoint_low"], endpoint_high=s["endpoint_high"])

    # Throttle: Gas + Bremse kombinieren
    t = cfg["throttle"]
    gas_axis = t.get("input_axis")
    brake_axis = t.get("input_axis_brake")
    # Prüfen ob diese Achse relevant ist
    if gas_axis is not None and int(gas_axis) == int(axis_code):
        pass  # Gas-Achse hat sich geändert
    elif brake_axis is not None and int(brake_axis) == int(axis_code):
        pass  # Bremse-Achse hat sich geändert
    else:
        return  # Nicht relevant für Throttle

    if brake_axis is not None:
        # Kombinierter Modus: Bremse > Deadzone → negativ, sonst Gas
        gas_val = live_axes.get(int(gas_axis), 0) if gas_axis is not None else 0
        brake_val = live_axes.get(int(brake_axis), 0) if brake_axis is not None else 0
        dz = t.get("deadzone", 0.02)
        if brake_val > dz:
            combined = -brake_val
        else:
            combined = gas_val
    else:
        # Einfacher Modus: nur Gas-Achse
        combined = live_axes.get(int(gas_axis), 0) if gas_axis is not None else 0

    result = compute_servo(combined, t)
    live_servo["throttle"] = result
    # ESP32: Throttle senden
    if esp32_client and esp32_client.connected:
        esp32_client.set_throttle(result["value"])
    # Lokaler Servo (Fallback)
    elif HAS_GPIO and servo_ctrl:
        servo_ctrl.set_position("throttle", combined, trim=t["trim"], expo=t["expo"],
            deadzone=t["deadzone"], invert=t["invert"],
            endpoint_low=t["endpoint_low"], endpoint_high=t["endpoint_high"])


def setup_servos(cfg):
    if not HAS_GPIO or not servo_ctrl:
        return
    for ch in ("steering", "throttle"):
        s = cfg[ch]
        servo_ctrl.add_servo(ch, s["servo_gpio"], s["min_pulse"], s["max_pulse"], s["center_pulse"])


def setup_esp32(cfg):
    """ESP32-Verbindung aufbauen."""
    global esp32_client
    esp_cfg = cfg.get("esp32", {})
    if not esp_cfg.get("enabled", False) or not HAS_ESP32:
        return
    host = esp_cfg.get("host", "192.168.4.1")
    hostname = esp_cfg.get("hostname", "rc-esc.local")
    port = esp_cfg.get("port", 80)
    esp32_client = ESP32Client(host=host, port=port, hostname=hostname)
    esp32_client.connect()
    print(f"[ESP32] Client gestartet → Hostname: {hostname} / Fallback: {host}:{port}")


def start_input(cfg):
    global input_reader
    if not HAS_INPUT:
        return
    if input_reader:
        input_reader.stop()
    path = cfg.get("input_device")
    if path:
        input_reader = InputReader(path, axis_callback=on_axis_input)
        input_reader.start()


# --- API Routes ---

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/config", methods=["GET"])
def api_get_config():
    return jsonify(load_config())

@app.route("/api/config", methods=["POST"])
def api_set_config():
    with config_lock:
        cfg = load_config()
        data = request.json
        for key in data:
            if key in cfg and isinstance(cfg[key], dict):
                cfg[key].update(data[key])
            elif key in cfg:
                cfg[key] = data[key]
        save_config(cfg)
        setup_servos(cfg)
        if "input_device" in data:
            start_input(cfg)
    return jsonify({"status": "ok"})

@app.route("/api/devices")
def api_devices():
    if not HAS_INPUT:
        return jsonify([])
    return jsonify(list_devices())

@app.route("/api/live")
def api_live():
    """Alle Live-Daten in einem Request: Achsen + Servo-Output + ESP32-Status."""
    esp32_status = esp32_client.status if esp32_client else {"connected": False}
    return jsonify({"axes": live_axes, "servo": live_servo, "esp32": esp32_status})


@app.route("/api/esp32/status")
def api_esp32_status():
    """ESP32 Verbindungsstatus."""
    if esp32_client:
        return jsonify(esp32_client.status)
    return jsonify({"connected": False, "error": "ESP32 Client nicht aktiv"})


@app.route("/api/esp32/arm", methods=["POST"])
def api_esp32_arm():
    """ESP32 ESC armen."""
    if esp32_client and esp32_client.connected:
        esp32_client.arm()
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "Nicht verbunden"}), 503


@app.route("/api/esp32/disarm", methods=["POST"])
def api_esp32_disarm():
    """ESP32 ESC disarmen."""
    if esp32_client and esp32_client.connected:
        esp32_client.disarm()
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "Nicht verbunden"}), 503


@app.route("/api/esp32/platform", methods=["POST"])
def api_esp32_platform():
    """ESP32 Plattform wechseln."""
    data = request.json
    platform_id = data.get("platform", 0)
    if esp32_client and esp32_client.connected:
        esp32_client.set_platform(platform_id)
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "Nicht verbunden"}), 503


# --- Start ---

if __name__ == "__main__":
    cfg = load_config()
    setup_servos(cfg)
    setup_esp32(cfg)
    start_input(cfg)
    print(f"\n🏎️  RC Servo Controller → http://localhost:8080\n")
    app.run(host="0.0.0.0", port=8080, debug=False)
