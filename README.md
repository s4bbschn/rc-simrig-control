# RC SimRig Control

Steuere ein RC-Car (z.B. Traxxas Drifter) mit einem Sim-Racing-Lenkrad. Das System liest die Eingaben vom Lenkrad per Raspberry Pi, sendet sie per WiFi an einen ESP32, der dann Lenkservo und ESC direkt per PWM ansteuert.

## Architektur

```
┌─────────────┐       USB        ┌──────────────┐      WiFi/WebSocket     ┌─────────────┐
│  Sim-Racing │ ───────────────► │ Raspberry Pi │ ───────────────────────► │   ESP32     │
│   Lenkrad   │                  │  Zero W2     │                          │  D1 Mini    │
│ (Logitech)  │                  │              │                          │             │
└─────────────┘                  └──────────────┘                          └──────┬──────┘
                                       │                                          │
                                       │ Web-UI :8080                    PWM GPIO │
                                       │ (Config & Monitor)                       │
                                                                           ┌──────┴──────┐
                                                                           │  Lenkservo  │
                                                                           │     ESC     │
                                                                           └─────────────┘
```

## Features

- **Echtzeit-Steuerung** — Lenkrad-Input wird mit ~50Hz an den ESP32 gesendet
- **Smoothing** — Exponentielles Glätten für butterweiche Servo-Bewegungen
- **ESC-Plattformen** — Traxxas (mit Rückwärts-Sequenz), Hobbywing, Generic
- **Web-UI auf dem Pi** — Live-Achsenmonitor, Trim, Expo, Deadzone, Input Scale, Endpoints
- **Web-UI auf dem ESP32** — Direkte manuelle Steuerung (Slider für Gas + Lenkung)
- **Failsafe** — Automatisch Neutral nach 2s ohne Befehl
- **WiFi flexibel** — ESP32 verbindet sich in bestehendes WLAN (z.B. Handy-Hotspot), Fallback auf eigenen AP
- **OTA-Updates** — ESP32 Firmware kabellos aktualisieren
- **Input Scale** — Voller Servoausschlag mit nur einem Bruchteil des Lenkradwegs (z.B. 90° statt 900°)

## Hardware

| Komponente | Funktion |
|---|---|
| Raspberry Pi Zero W2 | Liest Lenkrad-Input, sendet an ESP32 |
| ESP32 D1 Mini | Erzeugt PWM-Signale für Servo + ESC |
| Logitech Driving Force GT (oder ähnlich) | Sim-Racing-Lenkrad als Eingabegerät |
| RC-Car Lenkservo | Lenkung des RC-Cars |
| RC-Car ESC (z.B. Traxxas XL-5) | Motorsteuerung (Gas/Bremse/Rückwärts) |

### Verkabelung ESP32

```
ESP32 GPIO 16 ──── Signal (weiß/orange) ──── Lenkservo
ESP32 GPIO 19 ──── Signal (weiß/orange) ──── ESC
ESP32 GND ─────── GND (braun/schwarz) ────── beide Servos/ESC
BEC 5V (vom ESC) ── VCC (rot) ─────────────── Lenkservo + (optional) ESP32 VIN
```

**Wichtig:** Den Lenkservo nicht über den ESP32 mit Strom versorgen. Das 5V BEC-Kabel vom ESC auf das rote Kabel des Lenkservos brücken.

## Projekt-Struktur

```
rc-simrig-control/
├── rc-servo-controller/          # Raspberry Pi Software
│   ├── app.py                    # Flask Web-Server + Hauptlogik
│   ├── esp32_client.py           # WebSocket-Client zum ESP32
│   ├── input_reader.py           # Lenkrad-Input via evdev (Linux)
│   ├── input_reader_hid.py       # Lenkrad-Input via HID (Windows)
│   ├── servo_controller.py       # Lokale Servo-Steuerung (pigpio, Fallback)
│   ├── config.json               # Konfiguration
│   ├── requirements.txt          # Python Dependencies
│   ├── setup_pi.sh               # Automatisches Setup-Script für Pi
│   └── templates/
│       └── index.html            # Web-UI
│
└── esp32-esc-controller/         # ESP32 Firmware (PlatformIO)
    ├── platformio.ini            # PlatformIO Konfiguration
    ├── src/
    │   └── main.cpp              # ESP32 Firmware
    └── data/
        └── index.html            # ESP32 Web-UI (SPIFFS)
```

## Setup

### ESP32

**Voraussetzungen:** [PlatformIO](https://platformio.org/) installiert (VS Code Extension oder CLI)

1. **WiFi konfigurieren** — In `esp32-esc-controller/src/main.cpp` die SSID/Passwort deines Hotspots eintragen:
   ```cpp
   const char* STA_SSID = "DEIN_HOTSPOT";
   const char* STA_PASS = "DEIN_PASSWORT";
   ```

2. **Firmware flashen:**
   ```bash
   cd esp32-esc-controller
   pio run -t upload -e esp32dev
   ```

3. **Web-UI (SPIFFS) flashen:**
   ```bash
   pio run -t uploadfs -e esp32dev
   ```

4. **Testen:** Mit dem WiFi `RC-ESC-Controller` verbinden (falls kein Hotspot da), dann `http://192.168.4.1` öffnen.

**OTA-Update** (kabellos, wenn im gleichen Netz):
```bash
pio run -t upload -e ota
```

### Raspberry Pi Zero W2

1. **Raspberry Pi OS Lite** auf SD-Karte flashen, SSH aktivieren, WLAN konfigurieren (gleicher Hotspot wie ESP32)

2. **Projekt kopieren:**
   ```bash
   scp -r rc-servo-controller/ emc2@rc-pi.local:/home/emc2/
   ```

3. **Setup ausführen:**
   ```bash
   ssh emc2@rc-pi.local
   cd /home/emc2/rc-servo-controller
   chmod +x setup_pi.sh
   sudo ./setup_pi.sh
   ```

4. **Service starten:**
   ```bash
   sudo systemctl start rc-servo-controller
   ```

5. **Web-UI öffnen:** `http://rc-pi.local:8080`

### Konfiguration (config.json)

| Parameter | Beschreibung |
|---|---|
| `esp32.enabled` | ESP32-Kommunikation an/aus |
| `esp32.hostname` | mDNS-Name des ESP32 (Standard: `rc-esc.local`) |
| `esp32.host` | Fallback-IP (Standard: `192.168.4.1`) |
| `steering.input_scale` | Lenkübersetzung (4.0 = Vollanschlag bei 25% Lenkradweg) |
| `steering.expo` | Exponentialkurve (0 = linear, 1 = stark exponentiell) |
| `steering.trim` | Trim-Offset in Mikrosekunden |
| `steering.deadzone` | Totzone um die Mitte (0.0 - 0.3) |
| `throttle.input_axis_brake` | Separate Brems-Achse (optional) |

## ESC-Plattformen

| Plattform | Besonderheiten |
|---|---|
| **Traxxas** (XL-5, VXL) | Braucht Brems-Sequenz für Rückwärts: Bremse → Neutral → Rückwärts |
| **Hobbywing** (QuicRun) | Direkter Rückwärtsgang, kein Brems-Zwang |
| **Generic** | Standard PWM 1000-2000µs |

## Netzwerk-Setup

Empfohlenes Setup für unterwegs:

1. **Handy-Hotspot** einschalten (feste SSID)
2. **ESP32** verbindet sich automatisch zum Hotspot
3. **Raspberry Pi** verbindet sich zum gleichen Hotspot
4. Pi findet ESP32 über `rc-esc.local` (mDNS)

Falls kein Hotspot verfügbar: ESP32 macht nach 7.5s Timeout seinen eigenen AP auf (`RC-ESC-Controller` / `rc123456`).

## Troubleshooting

| Problem | Lösung |
|---|---|
| ESP32 AP nicht sichtbar | `pio run -t erase` dann neu flashen |
| Lenkrad nicht erkannt (Pi) | `lsusb` prüfen, evdev installiert? `pip install evdev` |
| WebSocket Abbrüche | Zu viele Clients? Browser-Tab schließen wenn Pi steuert |
| Servo ruckelt | `input_scale` oder Smoothing-Werte im ESP32-Code anpassen |
| Pedal invertiert | In Web-UI "Invertieren" Checkbox nutzen |

## Lizenz

MIT
