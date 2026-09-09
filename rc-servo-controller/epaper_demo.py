#!/usr/bin/env python3
"""Schoene 3-farbige Status-Demo fuer das Waveshare 2.13" (B) E-Paper.

Zeigt einen RC-SimRig Status-Screen mit Schwarz + Rot als Akzentfarbe.
Ideal als statischer Screen (aendert sich selten).
"""

import sys
import os
import time

_repo_lib = os.path.expanduser("~/e-Paper/RaspberryPi_JetsonNano/python/lib")
if os.path.isdir(_repo_lib):
    sys.path.append(_repo_lib)

from PIL import Image, ImageDraw, ImageFont


def load_driver():
    for version in ("epd2in13b_V4", "epd2in13b_V3"):
        try:
            module = __import__(f"waveshare_epd.{version}", fromlist=["EPD"])
            print(f"[OK] Treiber: {version}")
            return module.EPD()
        except Exception:
            continue
    print("[FEHLER] Kein 3-Color Treiber gefunden")
    sys.exit(1)


def font(size, bold=True):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)
    except Exception:
        return ImageFont.load_default()


def draw_car(d, x, y, s, drift=0):
    """Zeichnet ein stilisiertes Drift-RC-Auto (Seitenansicht).
    x,y = obere linke Ecke, s = Skalierungsfaktor.
    """
    def P(px, py):
        return (x + px * s, y + py * s)

    # Karosserie-Silhouette (Rennwagen-Keilform)
    body = [P(2, 14), P(10, 14), P(16, 6), P(30, 4), P(38, 12),
            P(52, 12), P(56, 14), P(54, 20), P(4, 20)]
    d.polygon(body, fill=0)
    # Heckspoiler
    d.line([P(2, 8), P(10, 8)], fill=0, width=max(1, int(2 * s)))
    d.line([P(2, 8), P(2, 14)], fill=0, width=max(1, int(2 * s)))
    # Cockpit-Fenster (weiss ausgespart -> nur Outline)
    # Raeder
    for wx in (14, 44):
        d.ellipse([P(wx - 6, 16), P(wx + 6, 28)], fill=0)


def main():
    print("=== EmC2 Saar 3-Color Demo ===")
    epd = load_driver()
    print("Init... (3-Color, dauert ~15s)")
    epd.init()
    epd.Clear()

    W, H = epd.height, epd.width  # 250 x 122
    black = Image.new("1", (W, H), 255)
    red = Image.new("1", (W, H), 255)
    db = ImageDraw.Draw(black)
    dr = ImageDraw.Draw(red)

    # ============================================================
    # ROTER HEADER-BALKEN mit ausgesparter weisser Schrift
    # ============================================================
    dr.rectangle((0, 0, W - 1, 34), fill=0)               # voller roter Balken
    # Schrift "aussparen": wir zeichnen den Text im ROT-Layer mit fill=255
    # (=weiss) — dadurch bleibt an der Textstelle Rot weg und das Weiss des
    # Panels zeigt sich = weisse Schrift auf rotem Grund.
    f_head = font(24)
    dr.text((10, 4), "EmC", font=f_head, fill=255)
    try:
        emc_w = dr.textlength("EmC", font=f_head)
    except Exception:
        emc_w = 52
    f_sup = font(14)
    dr.text((10 + emc_w + 1, 2), "2", font=f_sup, fill=255)
    dr.text((10 + emc_w + 14, 8), "SAAR e.V.", font=font(16), fill=255)

    # ============================================================
    # HAUPTBEREICH: Drift-Auto mit Speed & Rauch
    # ============================================================
    # Rennstrecken-Grundlinie (schwarz)
    db.line((6, 96, W - 6, 96), fill=0, width=2)
    # gestrichelte Mittelmarkierung auf der Strecke
    for mx in range(12, W - 10, 22):
        db.line((mx, 92, mx + 10, 92), fill=0, width=1)

    # Das Auto driftet nach rechts
    draw_car(db, 150, 62, 1.4)

    # Rote Speed-Linien hinter dem Auto
    for i, ly in enumerate((70, 78, 86)):
        x2 = 150 - i * 12
        dr.line((40, ly, x2, ly), fill=0, width=3)

    # Rote Drift-Rauchwolken (gestaffelte Kreise) hinterm Heck
    for (rx, ry, rr) in ((44, 88, 7), (34, 84, 6), (26, 90, 5), (20, 86, 4)):
        dr.ellipse((rx - rr, ry - rr, rx + rr, ry + rr), fill=0)

    # ============================================================
    # FOOTER: Formel-Anspielung + Projektname
    # ============================================================
    db.text((8, 104), "E=mc2", font=font(14), fill=0)
    db.text((92, 104), "RC SimRig Control", font=font(13, False), fill=0)

    # Zielflagge oben rechts (schwarz-weiss Karo)
    fx, fy, sq = W - 40, 42, 5
    for row in range(3):
        for col in range(5):
            if (row + col) % 2 == 0:
                db.rectangle((fx + col * sq, fy + row * sq,
                              fx + col * sq + sq, fy + row * sq + sq), fill=0)
    db.line((fx, fy, fx, fy + 3 * sq + 8), fill=0, width=1)  # Fahnenmast

    print("Zeichne (Schwarz + Rot)...")
    epd.display(epd.getbuffer(black), epd.getbuffer(red))

    print("Deep-Sleep — Bild bleibt stehen.")
    epd.sleep()
    print("=== Fertig! ===")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[FEHLER] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
