#!/usr/bin/env python3
"""Button-Scanner fuer das Sim-Racing-Lenkrad (evdev).

Zeigt live alle Tasten- und Achsen-Events an, damit wir herausfinden
welche Buttons das Lenkrad sendet und welche Codes sie haben.

Aufruf:  python button_scanner.py            (waehlt automatisch Logitech-Geraet)
         python button_scanner.py /dev/input/eventX   (bestimmtes Geraet)

Beenden: Strg+C
"""

import sys
import evdev
from evdev import ecodes


def find_wheel():
    """Sucht das Lenkrad automatisch (Logitech oder 'wheel'/'force' im Namen)."""
    devices = [evdev.InputDevice(p) for p in evdev.list_devices()]
    for dev in devices:
        name = dev.name.lower()
        if "logitech" in name or "wheel" in name or "force" in name or "driving" in name:
            return dev
    # Fallback: erstes Geraet mit Buttons anzeigen
    print("Kein Lenkrad automatisch erkannt. Verfuegbare Geraete:")
    for dev in devices:
        print(f"  {dev.path}  -  {dev.name}")
    return None


def main():
    if len(sys.argv) > 1:
        dev = evdev.InputDevice(sys.argv[1])
    else:
        dev = find_wheel()
        if dev is None:
            print("\nBitte Geraet als Argument angeben: python button_scanner.py /dev/input/eventX")
            sys.exit(1)

    print("=" * 50)
    print(f"Geraet: {dev.name}")
    print(f"Pfad:   {dev.path}")
    print("=" * 50)

    # Verfuegbare Buttons/Keys auflisten
    caps = dev.capabilities(verbose=False)
    if ecodes.EV_KEY in caps:
        keys = caps[ecodes.EV_KEY]
        print(f"\nGefundene Buttons/Keys ({len(keys)} Stueck):")
        for code in keys:
            names = ecodes.keys.get(code, f"CODE_{code}")
            if isinstance(names, list):
                names = "/".join(names)
            print(f"  Code {code}: {names}")

    print("\n" + "=" * 50)
    print("Druecke jetzt die Tasten am Lenkrad!")
    print("(Achsen werden auch angezeigt. Beenden mit Strg+C)")
    print("=" * 50 + "\n")

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY:
                names = ecodes.keys.get(event.code, f"CODE_{event.code}")
                if isinstance(names, list):
                    names = "/".join(names)
                state = {0: "LOSGELASSEN", 1: "GEDRUECKT", 2: "GEHALTEN"}.get(event.value, str(event.value))
                print(f"  [BUTTON] Code {event.code:5d}  ({names})  -> {state}")
            elif event.type == ecodes.EV_ABS:
                axis = ecodes.ABS.get(event.code, f"ABS_{event.code}")
                print(f"  [ACHSE ] Code {event.code:5d}  ({axis})  = {event.value}")
    except KeyboardInterrupt:
        print("\n\nScanner beendet.")


if __name__ == "__main__":
    main()
