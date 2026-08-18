"""Test mit 'inputs' Library — liest Raw HID Events auf Windows."""
import inputs

gamepads = inputs.devices.gamepads
print(f"Gamepads: {len(gamepads)}")
for g in gamepads:
    print(f"  {g.name}")

if not gamepads:
    print("\nKeine Gamepads. Versuche alle Geräte:")
    for d in inputs.devices.all_devices:
        print(f"  {d.name} ({type(d).__name__})")
    exit()

print("\nLese Events (Lenkrad drehen / Pedale drücken)...\n")
count = 0
while count < 50:
    events = inputs.get_gamepad()
    for e in events:
        if e.ev_type != "Sync":
            print(f"  {e.ev_type:12s} | {e.code:20s} | {e.state}")
            count += 1
