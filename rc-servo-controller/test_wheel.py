"""Schnelltest: Driving Force GT Achsen live anzeigen."""
import os, time
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame

pygame.init()
pygame.joystick.init()
count = pygame.joystick.get_count()
print(f"Geräte: {count}")
if count == 0:
    print("Kein Joystick gefunden"); exit()

joy = pygame.joystick.Joystick(0)
joy.init()
print(f"Name: {joy.get_name()}")
print(f"Achsen: {joy.get_numaxes()}")
print(f"Buttons: {joy.get_numbuttons()}\n")

# Test 1: Haupt-Thread polling
print("Dreh am Lenkrad / drück Pedale (5 Sekunden)...\n")
end = time.time() + 5
while time.time() < end:
    pygame.event.pump()
    vals = [f"{joy.get_axis(a):+.3f}" for a in range(min(joy.get_numaxes(), 6))]
    print(f"  Axes 0-5: {' | '.join(vals)}", end="\r")
    time.sleep(0.03)

print("\n\nTest 2: Aus Thread heraus...")
import threading

results = []
def thread_poll():
    for _ in range(100):
        pygame.event.pump()
        results.append(joy.get_axis(0))
        time.sleep(0.03)

t = threading.Thread(target=thread_poll)
t.start()
print("Dreh jetzt am Lenkrad (3 Sekunden)...")
t.join()
unique = len(set(round(v, 3) for v in results))
print(f"Verschiedene Werte aus Thread: {unique} (1 = Thread-Problem!)")
