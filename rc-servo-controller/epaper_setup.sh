#!/bin/bash
# ============================================================
# Waveshare 2.13" e-Paper HAT — Setup auf dem Raspberry Pi
# ============================================================
# Aktiviert SPI, installiert Abhängigkeiten und die Waveshare-Treiber.
# Ausfuehren:  chmod +x epaper_setup.sh && sudo ./epaper_setup.sh
# ============================================================

set -e

APP_DIR="/opt/rc-servo-controller"
VENV="${APP_DIR}/venv"

echo "=== Waveshare 2.13\" e-Paper Setup ==="

# --- SPI aktivieren ---
echo "[1/4] SPI aktivieren..."
if ! grep -q "^dtparam=spi=on" /boot/firmware/config.txt 2>/dev/null && \
   ! grep -q "^dtparam=spi=on" /boot/config.txt 2>/dev/null; then
    # Neuer Pfad (Bookworm) oder alter Pfad
    if [ -f /boot/firmware/config.txt ]; then
        echo "dtparam=spi=on" >> /boot/firmware/config.txt
    else
        echo "dtparam=spi=on" >> /boot/config.txt
    fi
    echo "  SPI aktiviert (Reboot noetig!)"
    SPI_CHANGED=1
else
    echo "  SPI bereits aktiv"
fi

# --- System-Pakete ---
echo "[2/4] System-Pakete installieren..."
apt-get install -y -qq python3-pil python3-numpy python3-spidev python3-gpiozero libopenjp2-7

# --- Waveshare Python-Lib ---
echo "[3/4] Waveshare e-Paper Treiber installieren..."
# Offizielle Lib per pip (enthaelt alle Treiber inkl. epd2in13_V4)
sudo -u emc2 "${VENV}/bin/pip" install -q waveshare-epaper 2>/dev/null || {
    echo "  pip-Paket nicht verfuegbar, klone GitHub-Repo..."
    if [ ! -d /home/emc2/e-Paper ]; then
        sudo -u emc2 git clone --depth 1 https://github.com/waveshare/e-Paper.git /home/emc2/e-Paper
    fi
}

# --- gpiozero/lgpio fuer Bookworm ---
echo "[4/4] GPIO-Backend..."
apt-get install -y -qq python3-lgpio 2>/dev/null || true

echo ""
echo "=== Fertig! ==="
if [ "${SPI_CHANGED}" = "1" ]; then
    echo ">>> WICHTIG: Bitte den Pi neustarten:  sudo reboot"
fi
echo "Danach Test:  ${VENV}/bin/python /home/emc2/rc-servo-controller/epaper_test.py"
