"""Schnelltest: Welche Joystick/HID-Geräte erkennt pygame?"""
import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import time

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print(f"\nGefundene Geräte: {count}\n")

if count == 0:
    print("Kein Joystick/HID-Gerät gefunden.")
    print("Tipp: 3D-Maus anstecken und Skript neu starten.")
    exit()

for i in range(count):
    joy = pygame.joystick.Joystick(i)
    joy.init()
    print(f"--- Gerät {i} ---")
    print(f"  Name:    {joy.get_name()}")
    print(f"  Achsen:  {joy.get_numaxes()}")
    print(f"  Buttons: {joy.get_numbuttons()}")
    print(f"  Hats:    {joy.get_numhats()}")
    print()

# Live-Achsen vom ersten Gerät anzeigen
joy = pygame.joystick.Joystick(0)
joy.init()
print(f"Live-Achsen von '{joy.get_name()}' (Ctrl+C zum Beenden):\n")

try:
    while True:
        pygame.event.pump()
        vals = [f"Axis{a}: {joy.get_axis(a):+.3f}" for a in range(joy.get_numaxes())]
        print("  " + "  |  ".join(vals), end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n\nBeendet.")
    pygame.quit()
