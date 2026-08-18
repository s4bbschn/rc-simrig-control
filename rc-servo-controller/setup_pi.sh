#!/bin/bash
# ============================================================
# RC Servo Controller — Raspberry Pi Zero W2 Setup
# ============================================================
# Dieses Script installiert alles auf einem frischen Raspberry Pi OS.
# Ausfuehren: 
#   chmod +x setup_pi.sh && sudo ./setup_pi.sh
# ============================================================

set -e

# --- Konfiguration ---
APP_DIR="/opt/rc-servo-controller"
APP_USER="emc2"
PYTHON_VENV="${APP_DIR}/venv"

echo "========================================"
echo " RC Servo Controller — Pi Setup"
echo "========================================"
echo ""

# --- System aktualisieren ---
echo "[1/7] System aktualisieren..."
apt-get update -qq
apt-get upgrade -y -qq

# --- Pakete installieren ---
echo "[2/7] Pakete installieren..."
apt-get install -y -qq \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-hid \
    git \
    libhidapi-hidraw0 \
    libhidapi-libusb0 \
    libhidapi-dev \
    libudev-dev \
    cython3 \
    avahi-daemon \
    avahi-utils

# --- Projekt-Verzeichnis erstellen ---
echo "[3/7] Projekt einrichten unter ${APP_DIR}..."
if [ -d "${APP_DIR}" ]; then
    echo "  Verzeichnis existiert bereits, aktualisiere..."
else
    mkdir -p "${APP_DIR}"
fi

# Dateien kopieren (wenn Script aus dem Projekt-Ordner aufgerufen wird)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/app.py" ]; then
    echo "  Kopiere Projekt-Dateien..."
    cp -r "${SCRIPT_DIR}/app.py" "${APP_DIR}/"
    cp -r "${SCRIPT_DIR}/esp32_client.py" "${APP_DIR}/"
    cp -r "${SCRIPT_DIR}/servo_controller.py" "${APP_DIR}/"
    cp -r "${SCRIPT_DIR}/input_reader.py" "${APP_DIR}/" 2>/dev/null || true
    cp -r "${SCRIPT_DIR}/input_reader_hid.py" "${APP_DIR}/"
    cp -r "${SCRIPT_DIR}/input_reader_pygame.py" "${APP_DIR}/" 2>/dev/null || true
    cp -r "${SCRIPT_DIR}/requirements.txt" "${APP_DIR}/"
    cp -r "${SCRIPT_DIR}/config.json" "${APP_DIR}/"
    cp -r "${SCRIPT_DIR}/templates" "${APP_DIR}/" 2>/dev/null || true
else
    echo "  WARNUNG: Script nicht aus dem Projektordner aufgerufen."
    echo "  Bitte Dateien manuell nach ${APP_DIR} kopieren."
fi

chown -R ${APP_USER}:${APP_USER} "${APP_DIR}"

# --- Python Virtual Environment ---
echo "[4/7] Python venv erstellen..."
sudo -u ${APP_USER} python3 -m venv --system-site-packages "${PYTHON_VENV}"
sudo -u ${APP_USER} "${PYTHON_VENV}/bin/pip" install --upgrade pip -q
sudo -u ${APP_USER} "${PYTHON_VENV}/bin/pip" install -q \
    flask==3.0.0 \
    websocket-client==1.7.0 \
    evdev==1.7.1

# --- udev-Regel fuer HID-Geraete (Lenkrad ohne root) ---
echo "[5/7] udev-Regel fuer HID-Zugriff..."
cat > /etc/udev/rules.d/99-hid-steering.rules << 'EOF'
# Logitech Driving Force / G29 / etc. — HID-Zugriff ohne root
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE="0666"
EOF
udevadm control --reload-rules
udevadm trigger

# --- Systemd Service ---
echo "[6/7] Systemd Service einrichten..."
cat > /etc/systemd/system/rc-servo-controller.service << EOF
[Unit]
Description=RC Servo Controller
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${APP_USER}
Group=${APP_USER}
WorkingDirectory=${APP_DIR}
ExecStart=${PYTHON_VENV}/bin/python app.py
Restart=always
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable rc-servo-controller.service

# --- mDNS / Avahi ---
echo "[7/7] mDNS (Avahi) konfigurieren..."
# Pi ist erreichbar unter rc-pi.local
hostnamectl set-hostname rc-pi 2>/dev/null || true
systemctl enable avahi-daemon
systemctl start avahi-daemon

echo ""
echo "========================================"
echo " Setup abgeschlossen!"
echo "========================================"
echo ""
echo " Projekt:  ${APP_DIR}"
echo " Python:   ${PYTHON_VENV}/bin/python"
echo " Service:  rc-servo-controller.service"
echo ""
echo " Naechste Schritte:"
echo "  1. Config anpassen:  nano ${APP_DIR}/config.json"
echo "  2. WLAN zum Handy-Hotspot verbinden (falls noch nicht)"
echo "  3. Service starten:  sudo systemctl start rc-servo-controller"
echo "  4. Status pruefen:   sudo systemctl status rc-servo-controller"
echo "  5. Logs:             journalctl -u rc-servo-controller -f"
echo ""
echo " Web-UI:   http://rc-pi.local:8080"
echo " ESP32:    Verbindet sich automatisch zum gleichen WLAN"
echo ""
