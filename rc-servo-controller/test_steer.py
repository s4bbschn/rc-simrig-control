"""Analysiert die Lenkachse des DFGT im Detail."""
import pywinusb.hid as hid
import time

dev = hid.HidDeviceFilter(vendor_id=0x046D).get_devices()[0]
dev.open()

samples = []
def on_data(data):
    samples.append(list(data))

dev.set_raw_data_handler(on_data)
print("Dreh das Lenkrad langsam von links nach rechts (3 Sek)...")
time.sleep(3)
dev.close()

print(f"\n{len(samples)} Samples gesammelt. Analyse:\n")
print("Byte:  0    1    2    3    4    5    6    7")
print("-" * 55)
# Zeige jeden 10. Sample
for i in range(0, len(samples), max(1, len(samples)//20)):
    d = samples[i]
    hex_str = " ".join(f"0x{b:02X}" for b in d[:8])
    # Verschiedene Lenkung-Interpretationen
    b3b4_le = d[3] | (d[4] << 8)
    b4b3_be = d[4] | (d[3] << 8)
    b1b2_le = d[1] | (d[2] << 8)
    print(f"  {hex_str}  | b3+b4 LE={b3b4_le:5d} (14bit={b3b4_le & 0x3FFF:5d})  b4+b3 BE={b4b3_be:5d}  b1+b2={b1b2_le:5d}")
