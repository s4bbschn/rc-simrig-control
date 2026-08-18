#!/bin/bash
# RC Servo Controller starten
# Voraussetzung: sudo pigpiod muss laufen

echo "Starte pigpio daemon..."
sudo pigpiod 2>/dev/null || true
sleep 1

echo "Starte RC Servo Controller..."
cd "$(dirname "$0")"
python3 app.py
