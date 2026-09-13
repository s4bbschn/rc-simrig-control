#!/usr/bin/env python3
"""Standalone-Test fuer das Display-Menue mit echten Lenkrad-Tasten.

Liest die Buttons des Driving Force GT per evdev und steuert damit das
E-Paper-Menue. Nutzt eine echte config.json (wird gelesen + geschrieben),
aber KEINE ESP32-Aktionen (die werden nur geloggt).

Beenden: Dreieck-Button (291) oder Strg+C
"""

import sys
import os
import json
import evdev
from evdev import ecodes

from display_menu import (
    DisplayMenu,
    BTN_ENCODER_CW, BTN_ENCODER_CCW, BTN_ENCODER_PRESS,
    BTN_X, BTN_TRIANGLE,
)

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")


def load_config():
    with open(CONFIG_FILE) as f:
        return json.load(f)


def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


# Config im Speicher halten
_cfg = load_config()


def get_config():
    return _cfg


def set_value(path, value):
    """Setzt einen verschachtelten Wert und speichert."""
    node = _cfg
    for p in path[:-1]:
        node = node[p]
    node[path[-1]] = value
    save_config(_cfg)
    print(f"[SET] {'/'.join(map(str, path))} = {value}")


def do_action(name, arg):
    """Im Test nur loggen (keine echte ESP32-Verbindung)."""
    print(f"[ACTION] {name}" + (f" ({arg})" if arg is not None else ""))


def find_wheel():
    for p in evdev.list_devices():
        dev = evdev.InputDevice(p)
        name = dev.name.lower()
        if "logitech" in name or "wheel" in name or "force" in name or "driving" in name:
            return dev
    return None


def main():
    print("=== Display-Menue Test (Lenkrad-Tasten) ===")

    dev = find_wheel()
    if dev is None:
        print("Kein Lenkrad gefunden!")
        sys.exit(1)
    print(f"Lenkrad: {dev.name}")

    menu = DisplayMenu(get_config, set_value, do_action, simulate=False)
    print("Display init...")
    menu.start()

    print("\nBedienung:")
    print("  Roter Knopf drehen  -> navigieren / Wert aendern")
    print("  Roter Knopf druecken-> auswaehlen / Edit")
    print("  X-Button            -> zurueck")
    print("  Dreieck             -> beenden\n")

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY and event.value == 1:  # nur "gedrueckt"
                if event.code == BTN_TRIANGLE:
                    break
                menu.handle_button(event.code)
    except KeyboardInterrupt:
        pass

    print("\nBeende, Display-Sleep...")
    menu.sleep()
    print("Fertig.")


if __name__ == "__main__":
    main()
