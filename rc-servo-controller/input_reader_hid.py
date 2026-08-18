"""Input-Reader für Logitech Driving Force GT via pywinusb (Windows HID)."""

import threading
import pywinusb.hid as hid

LOGITECH_VID = 0x046D


def list_devices():
    """Gibt alle Logitech HID-Geräte als Joystick-kompatible Liste zurück."""
    devs = hid.HidDeviceFilter(vendor_id=LOGITECH_VID).get_devices()
    result = []
    for d in devs:
        result.append({
            "path": f"hid:{d.vendor_id:04X}:{d.product_id:04X}",
            "name": d.product_name or f"Logitech {d.product_id:04X}",
            "phys": d.device_path,
            "axes": [
                {"code": 0, "name": "Lenkung", "min": 0, "max": 16383},
                {"code": 1, "name": "Gas", "min": 0, "max": 255},
                {"code": 2, "name": "Bremse", "min": 0, "max": 255},
                {"code": 3, "name": "Kupplung", "min": 0, "max": 255},
            ],
            "_vid": d.vendor_id,
            "_pid": d.product_id,
            "_hid_path": d.device_path,
        })
    # Fallback: alle HID-Geräte die Joystick-artig aussehen
    if not result:
        for d in hid.HidDeviceFilter().get_devices():
            if d.product_name and ("wheel" in d.product_name.lower()
                    or "force" in d.product_name.lower()
                    or "joystick" in d.product_name.lower()):
                result.append({
                    "path": f"hid:{d.vendor_id:04X}:{d.product_id:04X}",
                    "name": d.product_name,
                    "phys": d.device_path,
                    "axes": [
                        {"code": 0, "name": "Axis 0", "min": 0, "max": 65535},
                    ],
                    "_hid_path": d.device_path,
                })
    return result


class InputReader(threading.Thread):
    def __init__(self, device_path: str, axis_callback=None):
        super().__init__(daemon=True)
        self.device_path = device_path
        self.axis_callback = axis_callback
        self.running = False
        self.device = None
        self.axis_states = {}

    def _find_device(self):
        """Findet das HID-Gerät anhand des device_path (hid:VID:PID)."""
        parts = self.device_path.split(":")
        if len(parts) >= 3:
            vid = int(parts[1], 16)
            pid = int(parts[2], 16)
            devs = hid.HidDeviceFilter(vendor_id=vid, product_id=pid).get_devices()
            if devs:
                return devs[0]
        return None

    def _parse_dfgt_report(self, data):
        """Parst einen HID-Report vom Driving Force GT.
        Kompatibilitätsmodus (8 bytes): [ID, SteerLo, SteerHi, Btns, Hat, Gas, Brake, Clutch]
        Native Modus (12+ bytes):       [ID, ?, ?, ?, SteerLo, SteerHi, Gas, Brake, Clutch, ...]
        """
        if len(data) < 8:
            return

        if len(data) >= 12:
            # Native Mode: Lenkung in Byte 4+5 (14-bit LE)
            steer_raw = data[4] | (data[5] << 8)
            steer_raw = steer_raw & 0x3FFF
            steer_norm = (steer_raw / 16383.0) * 2.0 - 1.0
            gas_norm = data[6] / 255.0
            brake_norm = data[7] / 255.0
            clutch_norm = data[8] / 255.0 if len(data) > 8 else 0.0
        else:
            # Kompatibilitätsmodus: Lenkung in Byte 1+2 (10-bit LE)
            steer_raw = data[1] | (data[2] << 8)
            max_val = 16383 if steer_raw > 1023 else 1023
            steer_norm = (steer_raw / max_val) * 2.0 - 1.0
            gas_norm = (255 - data[5]) / 255.0
            brake_norm = (255 - data[6]) / 255.0
            clutch_norm = (255 - data[7]) / 255.0

        steer_norm = max(-1.0, min(1.0, steer_norm))

        axes = {0: steer_norm, 1: gas_norm, 2: brake_norm, 3: clutch_norm}
        for code, value in axes.items():
            value = round(value, 4)
            old = self.axis_states.get(code)
            if old is None or abs(value - old) > 0.001:
                self.axis_states[code] = value
                if self.axis_callback:
                    self.axis_callback(code, value)

    def _set_native_mode(self):
        """Schaltet Logitech-Lenkräder in den nativen Modus (volle Auflösung + 900°).
        Sendet den Logitech-spezifischen HID-Output-Report."""
        if not self.device:
            return
        try:
            # Logitech Native Mode Command:
            # Byte 0: Report ID (0x00)
            # Byte 1: 0xF8 = Set Mode command
            # Byte 2: 0x09 = Native mode mit detached
            # Byte 3: 0x05 = DFGT identifier
            # Byte 4: 0x01 = Enable
            # Rest: 0x00
            report = self.device.find_output_reports()
            if report:
                cmd = [0x00] * report[0].report_id  # Padding für Report ID
                cmd = [0x00, 0xF8, 0x09, 0x05, 0x01, 0x00, 0x00, 0x00]
                report[0].set_raw_data(cmd)
                report[0].send()
                print("[DFGT] Native Mode aktiviert (14-bit, 900°)")
                import time
                time.sleep(0.5)

                # Range auf 900° setzen
                # Command: 0xF8 0x81 range_lo range_hi 0x00 0x00 0x00
                range_val = 900
                cmd2 = [0x00, 0xF8, 0x81, range_val & 0xFF, (range_val >> 8) & 0xFF, 0x00, 0x00, 0x00]
                report[0].set_raw_data(cmd2)
                report[0].send()
                print(f"[DFGT] Lenkbereich auf {range_val}° gesetzt")
            else:
                print("[DFGT] Kein Output-Report gefunden — Kompatibilitätsmodus bleibt aktiv")
        except Exception as e:
            print(f"[DFGT] Native Mode fehlgeschlagen: {e}")

    def run(self):
        self.device = self._find_device()
        if not self.device:
            print(f"HID-Gerät nicht gefunden: {self.device_path}")
            return

        self.device.open()
        self._set_native_mode()
        self.running = True
        self.device.set_raw_data_handler(self._parse_dfgt_report)
        print(f"HID Input gestartet: {self.device.product_name}")

        # Blockiert bis stop() aufgerufen wird
        import time
        while self.running:
            time.sleep(0.1)

    def stop(self):
        self.running = False
        if self.device:
            try:
                self.device.close()
            except Exception:
                pass
