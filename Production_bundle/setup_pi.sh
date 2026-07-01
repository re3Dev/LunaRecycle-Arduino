#!/usr/bin/env bash
#
# LunaRecycle — Raspberry Pi installer
# Copies the production bundle to /opt/lunarecycle, builds a Python venv,
# grants serial access, and installs a systemd service that auto-starts on boot.
#
# Usage (run from inside the Production_bundle directory):
#   chmod +x setup_pi.sh
#   sudo ./setup_pi.sh
#
set -euo pipefail

APP_DIR="/opt/lunarecycle"
SERVICE="lunarecycle-backend.service"
# The account the service runs as. Defaults to the user who invoked sudo.
RUN_USER="${SUDO_USER:-pi}"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo: sudo ./setup_pi.sh" >&2
  exit 1
fi

echo "==> Installing LunaRecycle for user '${RUN_USER}' into ${APP_DIR}"

# 1. System dependencies -------------------------------------------------------
echo "==> Installing system packages (python3, venv, pip)"
apt-get update -qq
apt-get install -y python3 python3-venv python3-pip

# 2. Serial access -------------------------------------------------------------
echo "==> Adding '${RUN_USER}' to the 'dialout' group for serial port access"
usermod -aG dialout "${RUN_USER}"

# 3. Application files ----------------------------------------------------------
echo "==> Copying bundle to ${APP_DIR}"
mkdir -p "${APP_DIR}"
cp "${SRC_DIR}/lunarecycle_backend.py" "${APP_DIR}/"
cp "${SRC_DIR}/lunar_dashboard.html"   "${APP_DIR}/"
cp "${SRC_DIR}/lunar_viewer.html"      "${APP_DIR}/"
cp "${SRC_DIR}/lunar_model.html"       "${APP_DIR}/"
cp "${SRC_DIR}/requirements.txt"       "${APP_DIR}/"

# Preserve an existing env file so local port tweaks survive re-runs.
if [[ -f "${APP_DIR}/lunarecycle.env" ]]; then
  echo "==> Keeping existing ${APP_DIR}/lunarecycle.env"
else
  cp "${SRC_DIR}/lunarecycle.env" "${APP_DIR}/"
fi

# 4. Python virtual environment ------------------------------------------------
echo "==> Creating virtual environment and installing Python dependencies"
python3 -m venv "${APP_DIR}/venv"
"${APP_DIR}/venv/bin/pip" install --upgrade pip
"${APP_DIR}/venv/bin/pip" install -r "${APP_DIR}/requirements.txt"

chown -R "${RUN_USER}:${RUN_USER}" "${APP_DIR}"

# 5. systemd service -----------------------------------------------------------
echo "==> Installing systemd service"
# Substitute the run user into the unit before installing.
sed "s/^User=.*/User=${RUN_USER}/" "${SRC_DIR}/${SERVICE}" > "/etc/systemd/system/${SERVICE}"
systemctl daemon-reload
systemctl enable "${SERVICE}"
systemctl restart "${SERVICE}"

echo
echo "==> Done."
echo "    Service : systemctl status ${SERVICE}"
echo "    Logs    : journalctl -u ${SERVICE} -f"
echo "    Config  : ${APP_DIR}/lunarecycle.env"
echo "    Dashboard: http://$(hostname -I | awk '{print $1}'):5055/"
echo
echo "NOTE: if '${RUN_USER}' was just added to 'dialout', log out/in (or reboot)"
echo "      so the group change takes effect for interactive sessions."
