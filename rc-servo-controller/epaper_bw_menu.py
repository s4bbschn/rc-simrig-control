#!/usr/bin/env python3
"""Interaktives Menü auf dem 2.13" B/W E-Paper (V4) — Partial Refresh.

Nutzt den offiziellen Waveshare-Treiber epd2in13_V4 mit displayPartial()
fuer schnelle Menue-Navigation (~0.3s pro Update).

Tastatur-Test ueber SSH:
  Pfeil HOCH / RUNTER  -> Menuepunkt waehlen
  Pfeil LINKS / RECHTS -> Wert verstellen
  q                    -> Beenden
"""

import sys
import os
import time
import termios
import tty

_repo_lib = os.path.expanduser("~/e-Paper/RaspberryPi_JetsonNano/python/lib")
if os.path.isdir(_repo_lib):
    sys.path.append(_repo_lib)

from PIL import Image, ImageDraw, ImageFont
from waveshare_epd import epd2in13_V4

epd = epd2in13_V4.EPD()
W, H = epd.height, epd.width  # Querformat 250 x 122

# Menue-Daten
menu = [
    {"name": "Trim",     "value": 0,   "min": -100, "max": 100, "step": 5},
    {"name": "Expo",     "value": 30,  "min": 0,    "max": 100, "step": 5},
    {"name": "Deadzone", "value": 5,   "min": 0,    "max": 50,  "step": 1},
    {"name": "Scale",    "value": 40,  "min": 10,   "max": 100, "step": 5},
]
cursor = 0
VISIBLE = 3


def font(size, bold=True):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)
    except Exception:
        return ImageFont.load_default()


def render():
    """Erzeugt das Menue-Bild als getbuffer."""
    img = Image.new("1", (W, H), 255)
    d = ImageDraw.Draw(img)

    # Titel
    d.text((6, 1), "RC SimRig", font=font(16), fill=0)
    d.line((4, 20, W - 4, 20), fill=0)

    # Scroll-Fenster
    start = max(0, min(cursor - VISIBLE // 2, len(menu) - VISIBLE))
    start = max(0, start)
    visible = menu[start:start + VISIBLE]

    y = 26
    row_h = 30
    for idx, item in enumerate(visible):
        real_i = start + idx
        line = f"{item['name']}: {item['value']}"
        if real_i == cursor:
            d.rectangle((2, y - 2, W - 3, y + row_h - 6), fill=0)
            d.text((10, y), line, font=font(24), fill=255)
        else:
            d.text((10, y), line, font=font(24), fill=0)
        y += row_h

    # Scroll-Indikatoren
    if start > 0:
        d.polygon([(W - 14, 24), (W - 8, 24), (W - 11, 20)], fill=0)
    if start + VISIBLE < len(menu):
        d.polygon([(W - 14, H - 6), (W - 8, H - 6), (W - 11, H - 2)], fill=0)

    return epd.getbuffer(img)


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            return {'[A': 'UP', '[B': 'DOWN', '[C': 'RIGHT', '[D': 'LEFT'}.get(ch2, 'ESC')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    global cursor
    print("=== B/W E-Paper Menue Test ===")
    print("Pfeiltasten navigieren | q beenden")

    print("Init...")
    epd.init()
    epd.Clear(0xFF)

    # Basisbild setzen (noetig fuer Partial Refresh)
    print("Basisbild...")
    base = render()
    epd.displayPartBaseImage(base)

    print("\nBereit! Pfeiltasten nutzen.\n")
    running = True
    while running:
        key = read_key()
        if key == 'q':
            running = False
            continue
        elif key == 'UP':
            cursor = (cursor - 1) % len(menu)
        elif key == 'DOWN':
            cursor = (cursor + 1) % len(menu)
        elif key == 'RIGHT':
            it = menu[cursor]
            it['value'] = min(it['max'], it['value'] + it['step'])
        elif key == 'LEFT':
            it = menu[cursor]
            it['value'] = max(it['min'], it['value'] - it['step'])
        else:
            continue

        t0 = time.time()
        epd.displayPartial(render())
        dt = time.time() - t0
        print(f"  {key:6} -> '{menu[cursor]['name']}' (Update {dt:.2f}s)")

    print("\nDeep-Sleep...")
    epd.init()  # re-init nach Partial fuer sauberen Sleep
    epd.sleep()
    print("Fertig.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAbgebrochen.")
        try:
            epd.sleep()
        except Exception:
            pass
    except Exception as e:
        print(f"[FEHLER] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
