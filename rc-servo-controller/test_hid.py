"""Test: Driving Force GT über pywinusb HID finden und lesen."""
import pywinusb.hid as hid
import time

# Logitech Vendor ID = 0x046D
# Driving Force GT Product ID = 0xC29A
LOGITECH_VID = 0x046D

print("Suche Logitech HID-Geräte...\n")
all_devs = hid.HidDeviceFilter(vendor_id=LOGITECH_VID).get_devices()

if not all_devs:
    print("Keine Logitech-Geräte gefunden. Alle HID-Geräte:")
    for d in hid.HidDeviceFilter().get_devices():
        print(f"  VID:{d.vendor_id:04X} PID:{d.product_id:04X} — {d.product_name}")
    exit()

for d in all_devs:
    print(f"  VID:{d.vendor_id:04X} PID:{d.product_id:04X} — {d.product_name}")

# Erstes Logitech-Gerät öffnen und Daten lesen
dev = all_devs[0]
dev.open()

last_data = None
def on_data(data):
    global last_data
    if data != last_data:
        last_data = data
        # Erste 10 Bytes anzeigen
        hex_str = " ".join(f"{b:02X}" for b in data[:12])
        print(f"  [{len(data):3d} bytes] {hex_str} ...")

dev.set_raw_data_handler(on_data)
print(f"\nLese HID-Daten von '{dev.product_name}' (5 Sek, Lenkrad bewegen)...\n")
time.sleep(5)
dev.close()
print("\nFertig.")
