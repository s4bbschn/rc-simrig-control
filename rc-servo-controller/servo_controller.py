"""Servo controller using pigpio for hardware PWM on Raspberry Pi."""

import pigpio
import math


class ServoController:
    def __init__(self, pi: pigpio.pi = None):
        self.pi = pi or pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon nicht erreichbar. Starte mit: sudo pigpiod")
        self.servos = {}

    def add_servo(self, name: str, gpio: int, min_pulse=500, max_pulse=2400, center_pulse=1450):
        self.servos[name] = {
            "gpio": gpio,
            "min_pulse": min_pulse,
            "max_pulse": max_pulse,
            "center_pulse": center_pulse,
        }
        self.pi.set_mode(gpio, pigpio.OUTPUT)
        self.set_position(name, 0.0)

    def update_servo_config(self, name: str, **kwargs):
        if name in self.servos:
            self.servos[name].update(kwargs)

    def apply_expo(self, value: float, expo: float) -> float:
        """Exponentielle Kurve: expo 0.0 = linear, 1.0 = stark exponentiell."""
        if expo <= 0:
            return value
        sign = 1 if value >= 0 else -1
        abs_val = abs(value)
        return sign * ((1 - expo) * abs_val + expo * abs_val ** 3)

    def set_position(self, name: str, value: float, trim=0, expo=0.0,
                     deadzone=0.0, invert=False, endpoint_low=-1.0, endpoint_high=1.0):
        """Setzt Servo-Position. value: -1.0 bis 1.0"""
        if name not in self.servos:
            return
        servo = self.servos[name]

        if invert:
            value = -value

        # Deadzone
        if abs(value) < deadzone:
            value = 0.0

        # Expo
        value = self.apply_expo(value, expo)

        # Endpoints begrenzen
        value = max(endpoint_low, min(endpoint_high, value))

        # Auf Pulsweite mappen
        center = servo["center_pulse"] + trim
        if value >= 0:
            pulse = center + value * (servo["max_pulse"] - center)
        else:
            pulse = center + value * (center - servo["min_pulse"])

        pulse = max(servo["min_pulse"], min(servo["max_pulse"], int(pulse)))
        self.pi.set_servo_pulsewidth(servo["gpio"], pulse)

    def center_all(self):
        for name in self.servos:
            self.set_position(name, 0.0)

    def stop(self):
        for name, servo in self.servos.items():
            self.pi.set_servo_pulsewidth(servo["gpio"], 0)
        self.pi.stop()
