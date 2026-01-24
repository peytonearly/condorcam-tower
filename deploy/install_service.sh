#!/usr/bin/env bash
set -euo pipefail

# ---- must run as root because we write /etc/systemd/system and call systemctl enable ----
if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "ERROR: Please run with sudo:"
  echo "  sudo ./deploy/install_service.sh [options]"
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="tower"
ENABLE_PIGPIOD="false"
NO_OVERRIDE_EXECSTART="false"

usage() {
  cat <<EOF
Usage: sudo ./deploy/install_service.sh [options]

Options:
  --service-name <name>         Systemd unit base name (default: tower -> tower.service)
  --enable-pigpiod              Ensure pigpiod is installed and enabled (installs pigpio if missing)
  --no-override-execstart       Do not write a drop-in override for ExecStart/WorkingDirectory
  -h, --help                    Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --service-name)
      SERVICE_NAME="$2"
      shift 2
      ;;
    --enable-pigpiod)
      ENABLE_PIGPIOD="true"
      shift
      ;;
    --no-override-execstart)
      NO_OVERRIDE_EXECSTART="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

UNIT="${SERVICE_NAME}.service"
SYSTEMD_DIR="/etc/systemd/system"
DROPIN_DIR="${SYSTEMD_DIR}/${UNIT}.d"
DROPIN_FILE="${DROPIN_DIR}/override.conf"

have_unit_file() {
  # systemctl output includes headers unless we suppress them
  systemctl list-unit-files --type=service --no-legend 2>/dev/null | awk '{print $1}' | grep -qx "$1"
}

install_pigpio_via_apt() {
  if command -v apt-get >/dev/null 2>&1; then
    apt-get update
    apt-get install -y pigpio
    return 0
  elif command -v apt >/dev/null 2>&1; then
    apt update
    apt install -y pigpio
    return 0
  else
    echo "ERROR: Neither apt-get nor apt was found. Cannot install pigpio automatically."
    return 1
  fi
}

ensure_pigpiod() {
  if have_unit_file "pigpiod.service"; then
    echo "pigpiod.service is already present."
    return 0
  fi

  echo "pigpiod.service not found. Attempting to install pigpio (provides pigpiod on Raspberry Pi OS)..."
  if ! install_pigpio_via_apt; then
    return 1
  fi

  # Re-check for service after install
  if have_unit_file "pigpiod.service"; then
    echo "pigpiod.service is now present."
    return 0
  fi

  # If the daemon exists but no unit file, create one
  if command -v pigpiod >/dev/null 2>&1; then
    local pigpiod_bin
    pigpiod_bin="$(command -v pigpiod)"
    echo "pigpiod binary found at ${pigpiod_bin}, but pigpiod.service is missing. Creating a minimal systemd unit..."

    cat > "${SYSTEMD_DIR}/pigpiod.service" <<EOF
[Unit]
Description=Pigpio daemon
After=network.target

[Service]
Type=forking
ExecStart=${pigpiod_bin}
ExecStop=/usr/bin/killall pigpiod
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    return 0
  fi

  echo "ERROR: pigpiod is still not available after installing pigpio."
  echo "This can occur on some Debian/Trixie-based images where the pigpio *server* isn't packaged."
  echo "Recommended: use Raspberry Pi OS, or install pigpio/pigpiod from source per upstream instructions."
  return 1
}

# ---- Locate tower service file in repo (prefer deploy/systemd, fallback to repo root) ----
SRC_SERVICE=""
if [[ -f "${REPO_ROOT}/deploy/systemd/${UNIT}" ]]; then
  SRC_SERVICE="${REPO_ROOT}/deploy/systemd/${UNIT}"
elif [[ -f "${REPO_ROOT}/${UNIT}" ]]; then
  SRC_SERVICE="${REPO_ROOT}/${UNIT}"
else
  echo "ERROR: Could not find ${UNIT} in:"
  echo "  - ${REPO_ROOT}/deploy/systemd/${UNIT}"
  echo "  - ${REPO_ROOT}/${UNIT}"
  exit 1
fi

echo "Repo root: ${REPO_ROOT}"
echo "Installing ${UNIT} from: ${SRC_SERVICE}"

# ---- Ensure venv exists if we are overriding ExecStart to use it ----
VENV_PY="${REPO_ROOT}/.venv/bin/python"
if [[ "${NO_OVERRIDE_EXECSTART}" == "false" && ! -x "${VENV_PY}" ]]; then
  echo "ERROR: Expected venv python at ${VENV_PY} but it was not found/executable."
  echo "Run: ./deploy/bootstrap_pi.sh"
  exit 1
fi

# ---- Install base unit ----
install -m 0644 "${SRC_SERVICE}" "${SYSTEMD_DIR}/${UNIT}"

# ---- Optionally install/enable pigpiod ----
if [[ "${ENABLE_PIGPIOD}" == "true" ]]; then
  ensure_pigpiod
  echo "Enabling/starting pigpiod.service..."
  systemctl enable --now pigpiod.service
fi

# ---- Drop-in override so the unit runs THIS checkout + THIS venv ----
if [[ "${NO_OVERRIDE_EXECSTART}" == "false" ]]; then
  echo "Writing drop-in override: ${DROPIN_FILE}"
  mkdir -p "${DROPIN_DIR}"
  cat > "${DROPIN_FILE}" <<EOF
[Service]
WorkingDirectory=${REPO_ROOT}
Environment=PYTHONUNBUFFERED=1
ExecStart=
ExecStart=${VENV_PY} -m pi_runtime.main
EOF
fi

echo "Reloading systemd..."
systemctl daemon-reload

echo "Enabling/starting ${UNIT}..."
systemctl enable --now "${UNIT}"

echo
echo "Done."
echo "Useful commands:"
echo "  systemctl status ${UNIT} --no-pager"
echo "  journalctl -u ${UNIT} -n 200 --no-pager"
