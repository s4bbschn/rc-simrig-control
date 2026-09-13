"""Display-Menü fuer das 2.13" B/W E-Paper — gesteuert per Lenkrad-Tasten.

Hierarchisches Menue zum Einstellen aller Fahr-Parameter direkt am Rig.
Bedienung ueber den roten Dreh-Encoder des Driving Force GT:

  Encoder CW  (704)  -> naechster Eintrag / Wert erhoehen
  Encoder CCW (705)  -> vorheriger Eintrag / Wert verringern
  Encoder druecken (302) -> auswaehlen / Edit-Modus umschalten
  X-Button (288)     -> zurueck / abbrechen

Das Modul ist von der App entkoppelt: es bekommt Getter/Setter-Callbacks,
sodass es dieselbe config.json und dieselben ESP32-Aktionen nutzt wie die Web-UI.
"""

import sys
import os
import time
import threading

_repo_lib = os.path.expanduser("~/e-Paper/RaspberryPi_JetsonNano/python/lib")
if os.path.isdir(_repo_lib):
    sys.path.append(_repo_lib)

from PIL import Image, ImageDraw, ImageFont

# Button-Codes des Logitech Driving Force GT (per Scanner ermittelt)
BTN_ENCODER_CW = 704    # roter Knopf im Uhrzeigersinn
BTN_ENCODER_CCW = 705   # roter Knopf gegen Uhrzeigersinn
BTN_ENCODER_PRESS = 302 # roter Knopf druecken (Enter)
BTN_X = 288             # X-Button -> zurueck
BTN_SQUARE = 289
BTN_CIRCLE = 290
BTN_TRIANGLE = 291


# ============================================================
# Menue-Definition
# ============================================================
# Jeder Eintrag ist ein dict:
#   type: "submenu" | "value" | "bool" | "choice" | "action"
#   Fuer value: path (config-Pfad), min, max, step, scale (Anzeige-Faktor)
#   Fuer choice: options (Liste), path
#   Fuer action: callback-Name
def build_menu():
    return [
        {"label": "Lenkung", "type": "submenu", "items": [
            {"label": "Trim",       "type": "value", "path": ["steering", "trim"],          "min": -200, "max": 200, "step": 5},
            {"label": "Expo",       "type": "value", "path": ["steering", "expo"],          "min": 0,    "max": 100, "step": 5,  "disp": 0.01},
            {"label": "Deadzone",   "type": "value", "path": ["steering", "deadzone"],      "min": 0,    "max": 30,  "step": 1,  "disp": 0.01},
            {"label": "Input Scale","type": "value", "path": ["steering", "input_scale"],   "min": 10,   "max": 100, "step": 5,  "disp": 0.1},
            {"label": "Endp. Links","type": "value", "path": ["steering", "endpoint_low"],  "min": -100, "max": 0,   "step": 5,  "disp": 0.01},
            {"label": "Endp. Rechts","type": "value","path": ["steering", "endpoint_high"], "min": 0,    "max": 100, "step": 5,  "disp": 0.01},
            {"label": "Invertieren","type": "bool",  "path": ["steering", "invert"]},
        ]},
        {"label": "Gas/Bremse", "type": "submenu", "items": [
            {"label": "Trim",       "type": "value", "path": ["throttle", "trim"],          "min": -200, "max": 200, "step": 5},
            {"label": "Expo",       "type": "value", "path": ["throttle", "expo"],          "min": 0,    "max": 100, "step": 5,  "disp": 0.01},
            {"label": "Deadzone",   "type": "value", "path": ["throttle", "deadzone"],      "min": 0,    "max": 50,  "step": 1,  "disp": 0.01},
            {"label": "Endp. Zurueck","type": "value","path": ["throttle", "endpoint_low"], "min": -100, "max": 0,   "step": 5,  "disp": 0.01},
            {"label": "Endp. Vor",  "type": "value", "path": ["throttle", "endpoint_high"], "min": 0,    "max": 100, "step": 5,  "disp": 0.01},
            {"label": "Invertieren","type": "bool",  "path": ["throttle", "invert"]},
        ]},
        {"label": "Fahrmodus", "type": "submenu", "items": [
            {"label": "Modus", "type": "choice", "path": ["drive_assist", "mode"],
             "options": ["Direct", "Stability", "Autopilot"], "action": "set_mode"},
        ]},
        {"label": "Stability", "type": "submenu", "items": [
            {"label": "Gegenlenk",  "type": "value", "path": ["drive_assist", "stability", "counterSteerGain"], "min": 0, "max": 30, "step": 1, "disp": 0.1, "action": "stability"},
            {"label": "Gas-Limit",  "type": "value", "path": ["drive_assist", "stability", "throttleLimitGain"], "min": 0, "max": 10, "step": 1, "disp": 0.1, "action": "stability"},
            {"label": "Yaw-Grenze", "type": "value", "path": ["drive_assist", "stability", "yawThreshold"],      "min": 5, "max": 50, "step": 1, "action": "stability"},
            {"label": "Reaktion",   "type": "value", "path": ["drive_assist", "stability", "responseSpeed"],     "min": 1, "max": 10, "step": 1, "disp": 0.1, "action": "stability"},
        ]},
        {"label": "Autopilot", "type": "submenu", "items": [
            {"label": "Max Yaw",  "type": "value", "path": ["drive_assist", "autopilot", "yawRateMax"], "min": 50, "max": 300, "step": 10, "action": "autopilot"},
            {"label": "Steer P",  "type": "value", "path": ["drive_assist", "autopilot", "steerP"],     "min": 1,  "max": 50,  "step": 1,  "disp": 0.1, "action": "autopilot"},
            {"label": "Steer I",  "type": "value", "path": ["drive_assist", "autopilot", "steerI"],     "min": 0,  "max": 20,  "step": 1,  "disp": 0.01, "action": "autopilot"},
            {"label": "Steer D",  "type": "value", "path": ["drive_assist", "autopilot", "steerD"],     "min": 0,  "max": 20,  "step": 1,  "disp": 0.1, "action": "autopilot"},
            {"label": "Thr P",    "type": "value", "path": ["drive_assist", "autopilot", "thrP"],       "min": 1,  "max": 50,  "step": 1,  "disp": 0.1, "action": "autopilot"},
            {"label": "Thr I",    "type": "value", "path": ["drive_assist", "autopilot", "thrI"],       "min": 0,  "max": 10,  "step": 1,  "disp": 0.01, "action": "autopilot"},
            {"label": "Thr D",    "type": "value", "path": ["drive_assist", "autopilot", "thrD"],       "min": 0,  "max": 20,  "step": 1,  "disp": 0.1, "action": "autopilot"},
            {"label": "Basis-Gas","type": "value", "path": ["drive_assist", "autopilot", "baseThrottle"],"min": 1, "max": 10,  "step": 1,  "disp": 0.1, "action": "autopilot"},
        ]},
        {"label": "Force FB", "type": "submenu", "items": [
            {"label": "Aktiv",      "type": "bool",  "path": ["ffb", "enabled"], "action": "ffb"},
            {"label": "Yaw Gain",   "type": "value", "path": ["ffb", "yaw_gain"],     "min": 0, "max": 30, "step": 1, "disp": 0.1, "action": "ffb"},
            {"label": "Lat Gain",   "type": "value", "path": ["ffb", "lateral_gain"], "min": 0, "max": 20, "step": 1, "disp": 0.1, "action": "ffb"},
            {"label": "Damping",    "type": "value", "path": ["ffb", "damping"],      "min": 0, "max": 10, "step": 1, "disp": 0.1, "action": "ffb"},
            {"label": "Max Force",  "type": "value", "path": ["ffb", "max_force"],    "min": 1, "max": 10, "step": 1, "disp": 0.1, "action": "ffb"},
        ]},
        {"label": "ESP32", "type": "submenu", "items": [
            {"label": "ARM",         "type": "action", "action": "arm"},
            {"label": "DISARM",      "type": "action", "action": "disarm"},
            {"label": "IMU Kalibr.", "type": "action", "action": "calibrate"},
        ]},
    ]


class DisplayMenu:
    """Verwaltet Menue-Zustand, Rendering und Button-Handling."""

    def __init__(self, get_config, set_value, do_action, simulate=False):
        """
        get_config()          -> aktuelles config-dict
        set_value(path, val)  -> Wert in config setzen + speichern
        do_action(name, arg)  -> ESP32-Aktion ausloesen (arm/disarm/mode/...)
        simulate              -> True: ohne echtes Display (nur Konsole)
        """
        self.get_config = get_config
        self.set_value = set_value
        self.do_action = do_action
        self.simulate = simulate

        self.menu = build_menu()
        self.stack = [self.menu]     # Navigations-Stack (fuer Submenues)
        self.cursor = [0]            # Cursor pro Ebene
        self.editing = False         # Edit-Modus fuer Werte
        self.dirty = True            # Neu zeichnen noetig
        self._partial_count = 0

        self.epd = None
        if not simulate:
            from waveshare_epd import epd2in13_V4
            self.epd = epd2in13_V4.EPD()
            self.W, self.H = self.epd.height, self.epd.width
        else:
            self.W, self.H = 250, 122

    # ---------- Config-Zugriff ----------
    def _get(self, path):
        cfg = self.get_config()
        node = cfg
        for p in path:
            node = node[p]
        return node

    def _current_items(self):
        return self.stack[-1]

    def _current_entry(self):
        return self._current_items()[self.cursor[-1]]

    # ---------- Font ----------
    def _font(self, size, bold=True):
        name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
        try:
            return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)
        except Exception:
            return ImageFont.load_default()

    # ---------- Wert-Formatierung ----------
    def _format_value(self, entry):
        t = entry["type"]
        if t == "value":
            raw = self._get(entry["path"])
            disp = entry.get("disp", 1)
            if disp != 1:
                return f"{raw * 1:.2f}" if disp >= 0.01 else str(raw)
            return str(raw)
        elif t == "bool":
            return "AN" if self._get(entry["path"]) else "AUS"
        elif t == "choice":
            idx = int(self._get(entry["path"]))
            opts = entry["options"]
            return opts[idx] if 0 <= idx < len(opts) else "?"
        elif t == "submenu":
            return ">"
        elif t == "action":
            return ""
        return ""

    # ---------- Rendering ----------
    def render(self):
        img = Image.new("1", (self.W, self.H), 255)
        d = ImageDraw.Draw(img)

        # Breadcrumb-Titel
        if len(self.stack) == 1:
            title = "RC SimRig - Setup"
        else:
            # Ebene-Name aus dem Parent-Cursor holen
            parent = self.stack[-2]
            title = parent[self.cursor[-2]]["label"]
        d.rectangle((0, 0, self.W - 1, 18), fill=0)
        d.text((4, 1), title, font=self._font(13), fill=255)

        items = self._current_items()
        cur = self.cursor[-1]
        VISIBLE = 3
        start = max(0, min(cur - VISIBLE // 2, len(items) - VISIBLE))
        start = max(0, start)
        visible = items[start:start + VISIBLE]

        y = 24
        row_h = 30
        for idx, entry in enumerate(visible):
            real_i = start + idx
            selected = (real_i == cur)
            label = entry["label"]
            val = self._format_value(entry)

            if selected:
                # Im Edit-Modus: invertierter Rahmen mit Markierung
                d.rectangle((2, y - 2, self.W - 3, y + row_h - 6), fill=0)
                fg = 255
                if self.editing:
                    # Edit-Marker: kleine Pfeile links/rechts
                    d.text((6, y), "<", font=self._font(22), fill=255)
                    d.text((self.W - 18, y), ">", font=self._font(22), fill=255)
                    d.text((22, y), f"{label}: {val}", font=self._font(20), fill=fg)
                else:
                    d.text((8, y), f"{label}: {val}" if val else label, font=self._font(20), fill=fg)
            else:
                txt = f"{label}: {val}" if val else label
                d.text((8, y), txt, font=self._font(20), fill=0)
            y += row_h

        # Scroll-Indikatoren
        if start > 0:
            d.polygon([(self.W - 12, 22), (self.W - 6, 22), (self.W - 9, 18)], fill=0)
        if start + VISIBLE < len(items):
            d.polygon([(self.W - 12, self.H - 4), (self.W - 6, self.H - 4), (self.W - 9, self.H)], fill=0)

        return img

    def draw(self, force_full=False):
        if self.simulate:
            self._print_console()
            return
        buf = self.epd.getbuffer(self.render())
        # Alle 15 Partial-Refreshes ein Full-Refresh gegen Geisterbilder
        if force_full or self._partial_count >= 15:
            self.epd.init()
            self.epd.displayPartBaseImage(buf)
            self._partial_count = 0
        else:
            self.epd.displayPartial(buf)
            self._partial_count += 1

    def _print_console(self):
        os.system("clear")
        items = self._current_items()
        print(f"--- {'ROOT' if len(self.stack)==1 else 'SUB'} (edit={self.editing}) ---")
        for i, e in enumerate(items):
            mark = ">" if i == self.cursor[-1] else " "
            print(f"{mark} {e['label']}: {self._format_value(e)}")

    # ---------- Navigation ----------
    def start(self):
        if not self.simulate:
            self.epd.init()
            self.epd.Clear(0xFF)
            self.epd.displayPartBaseImage(self.epd.getbuffer(self.render()))
        else:
            self._print_console()

    def _adjust_value(self, entry, direction):
        """Aendert einen Wert im Edit-Modus. direction: +1 / -1"""
        t = entry["type"]
        if t == "value":
            step = entry.get("step", 1) * direction
            val = self._get(entry["path"]) + step
            val = max(entry["min"], min(entry["max"], val))
            self.set_value(entry["path"], val)
            if entry.get("action"):
                self.do_action(entry["action"], None)
        elif t == "bool":
            new = not self._get(entry["path"])
            self.set_value(entry["path"], new)
            if entry.get("action"):
                self.do_action(entry["action"], None)
        elif t == "choice":
            idx = int(self._get(entry["path"]))
            idx = (idx + direction) % len(entry["options"])
            self.set_value(entry["path"], idx)
            if entry.get("action"):
                self.do_action(entry["action"], idx)

    def handle_button(self, code):
        """Verarbeitet einen Tastendruck. Gibt True zurueck wenn neu gezeichnet."""
        items = self._current_items()
        entry = self._current_entry()

        if code in (BTN_ENCODER_CW, BTN_ENCODER_CCW):
            direction = 1 if code == BTN_ENCODER_CW else -1
            if self.editing:
                self._adjust_value(entry, direction)
            else:
                self.cursor[-1] = (self.cursor[-1] + direction) % len(items)
            self.draw()
            return True

        elif code == BTN_ENCODER_PRESS:
            if self.editing:
                # Edit beenden
                self.editing = False
            else:
                if entry["type"] == "submenu":
                    self.stack.append(entry["items"])
                    self.cursor.append(0)
                elif entry["type"] == "action":
                    self.do_action(entry["action"], None)
                else:
                    # value/bool/choice -> Edit-Modus an
                    self.editing = True
            self.draw()
            return True

        elif code == BTN_X:
            if self.editing:
                self.editing = False
            elif len(self.stack) > 1:
                self.stack.pop()
                self.cursor.pop()
            self.draw()
            return True

        return False

    def sleep(self):
        if not self.simulate and self.epd:
            self.epd.init()
            self.epd.sleep()
