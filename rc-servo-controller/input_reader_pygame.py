"""Reads USB steering wheel / joystick input via pygame (Windows/Mac/Linux)."""

import threading
import time
import os

# pygame gibt viel auf stdout aus — unterdrücken
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame


def list_devices():
    """Gibt alle verfügbaren Joystick/Lenkrad-Geräte zurück."""
    pygame.joystick.quit()
    pygame.joystick.init()
    result = []
    for i in range(pygame.joystick.get_count()):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        axes = []
        for a in range(joy.get_numaxes()):
            axes.append({
                "code": a,
                "name": f"Axis {a}",
                "min": -1,
                "max": 1,
            })
        result.append({
            "path": f"joystick:{i}",
            "name": joy.get_name(),
            "phys": f"joystick-{i}",
            "axes": axes,
        })
    return result


class InputReader(threading.Thread):
    def __init__(self, device_path: str, axis_callback=None):
        super().__init__(daemon=True)
        self.device_path = device_path
        self.axis_callback = axis_callback
        self.running = False
        self.axis_states = {}
        self.joystick = None

    def _get_joystick_index(self):
        # device_path ist "joystick:0", "joystick:1", etc.
        try:
            return int(self.device_path.split(":")[1])
        except (IndexError, ValueError):
            return 0

    def run(self):
        idx = self._get_joystick_index()
        pygame.joystick.quit()
        pygame.joystick.init()

        if idx >= pygame.joystick.get_count():
            print(f"Joystick {idx} nicht gefunden")
            return

        self.joystick = pygame.joystick.Joystick(idx)
        self.joystick.init()
        self.running = True
        num_axes = self.joystick.get_numaxes()

        print(f"Input gestartet: {self.joystick.get_name()} ({num_axes} Achsen)")

        while self.running:
            pygame.event.pump()
            for a in range(num_axes):
                value = round(self.joystick.get_axis(a), 4)
                old = self.axis_states.get(a)
                if old is None or abs(value - old) > 0.001:
                    self.axis_states[a] = value
                    if self.axis_callback:
                        self.axis_callback(a, value)
            time.sleep(0.02)  # ~50Hz polling

    def stop(self):
        self.running = False
