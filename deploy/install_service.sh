#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# install_service.sh
#
# Installs/updates the tower systemd service, and optionally ensures pigpiod is
# installed + enabled (including a source-build fallback if APT has no pigpio).
#
# Usage:
#   sudo ./deploy/install_service.sh --enable-pigpiod
#   sudo ./deploy/install_service.sh --no-override-execstart
#   sudo ./deploy/install_service.sh --service-name tower
# =============================================================================

# ---- Must run as root because we write /etc/systemd/system and call systemctl ----
if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "ERROR: Please run with sudo:"
  echo "  sudo ./deploy/install_service.sh [options]"
  exit 1
fi

# ---- Resolve repo root robustly ----
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

SERVICE_NAME="tower"
ENABLE_PIGPIOD="false"
NO_OVERRIDE_EXECSTART="false"

usage() {
  cat <<EOF
Usage: sudo ./deploy/install_service.sh [options]

Options:
  --service-name <name>         Systemd unit base name (default: tower -> tower.service)
  --enable-pigpiod              Ensure pigpiod is installed and enabled (APT or source-build fallback)
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

# =============================================================================
# Helper functions
# =============================================================================

have_unit_file() {
  systemctl list-unit-files --type=service --no-legend 2>/dev/null \
    | awk '{print $1}' | grep -qx "$1"
}

apt_install() {
  # Usage: apt_install pkg1 pkg2 ...
  if command -v apt-get >/dev/null 2>&1; then
    apt-get update
    apt-get install -y "$@"
  elif command -v apt >/dev/null 2>&1; then
    apt update
    apt install -y "$@"
  else
    echo "ERROR: Neither apt-get nor apt was found; cannot install packages: $*"
    return 1
  fi
}

install_pigpio_via_apt() {
  # Best-effort. Returns nonzero if not available.
  echo "Attempting APT install: pigpio"
  if command -v apt-get >/dev/null 2>&1; then
    apt-get update
    apt-get install -y pigpio
  elif command -v apt >/dev/null 2>&1; then
    apt update
    apt install -y pigpio
  else
    return 1
  fi
}

install_pigpiod_from_source() {
  echo "APT package 'pigpio' unavailable. Attempting source build/install of pigpio (pigpiod)..."

  # Build dependencies
  apt_install git make gcc || return 1

  local tmpdir
  tmpdir="$(mktemp -d)"
  echo "Using temp build dir: ${tmpdir}"

  # Ensure we clean up temp dir
  cleanup() { rm -rf "${tmpdir}"; }
  trap cleanup RETURN

  git clone --depth=1 https://github.com/joan2937/pigpio.git "${tmpdir}/pigpio"
  make -C "${tmpdir}/pigpio"
  make -C "${tmpdir}/pigpio" install
  ldconfig || true

  if ! command -v pigpiod >/dev/null 2>&1; then
    echo "ERROR: Source install completed but pigpiod not found in PATH."
    return 1
  fi

  echo "Source install OK: $(command -v pigpiod)"
  return 0
}

ensure_pigpiod_unit_exists() {
  # If pigpiod exists but no systemd unit file, create a minimal unit.
  if have_unit_file "pigpiod.service"; then
    return 0
  fi

  if ! command -v pigpiod >/dev/null 2>&1; then
    return 1
  fi

  local pigpiod_bin
  pigpiod_bin="$(command -v pigpiod)"

  # Ensure killall exists (used for ExecStop). If not, try to install it.
  if ! command -v killall >/dev/null 2>&1; then
    echo "killall not found; installing psmisc (provides killall)..."
    apt_install psmisc || true
  fi

  local execstop="/bin/kill -TERM \$MAINPID"
  if command -v killall >/dev/null 2>&1; then
    execstop="$(command -v killall) pigpiod"
  fi

  echo "pigpiod found at ${pigpiod_bin}, but pigpiod.service is missing. Creating a minimal systemd unit..."

  cat > "${SYSTEMD_DIR}/pigpiod.service" <<EOF
[Unit]
Description=Pigpio daemon
After=network.target

[Service]
Type=forking
ExecStart=${pigpiod_bin}
ExecStop=${execstop}
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

  systemctl daemon-reload
  return 0
}

ensure_pigpiod() {
  if have_unit_file "pigpiod.service"; then
    echo "pigpiod.service is already present."
    return 0
  fi

  # First attempt: APT install pigpio (may be unavailable on some OS images)
  if install_pigpio_via_apt; then
    echo "APT install of pigpio completed."
  else
    echo "APT install of pigpio failed. Falling back to source build..."
    install_pigpiod_from_source
  fi

  # After install attempt, ensure we have a unit file
  if have_unit_file "pigpiod.service"; then
    echo "pigpiod.service is now present."
    return 0
  fi

  # If pigpiod exists but no unit file, create one
  ensure_pigpiod_unit_exists
}

# =============================================================================
# Locate tower service file in repo
# =============================================================================

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

# =============================================================================
# Ensure venv exists if we are overriding ExecStart to use it
# =============================================================================

VENV_PY="${REPO_ROOT}/.venv/bin/python"
if [[ "${NO_OVERRIDE_EXECSTART}" == "false" && ! -x "${VENV_PY}" ]]; then
  echo "ERROR: Expected venv python at ${VENV_PY} but it was not found/executable."
  echo "Run: ./deploy/bootstrap_pi.sh"
  exit 1
fi

# =============================================================================
# Install base unit
# =============================================================================

install -m 0644 "${SRC_SERVICE}" "${SYSTEMD_DIR}/${UNIT}"

# =============================================================================
# Optionally install/enable pigpiod
# =============================================================================

if [[ "${ENABLE_PIGPIOD}" == "true" ]]; then
  ensure_pigpiod
  echo "Enabling/starting pigpiod.service..."
  systemctl enable --now pigpiod.service
fi

# =============================================================================
# Drop-in override so the unit runs THIS checkout + THIS venv
# =============================================================================

if [[ "${NO_OVERRIDE_EXECSTART}" == "false" ]]; then
  echo "Writing drop-in override: ${DROPIN_FILE}"
  mkdir -p "${DROPIN_DIR}"

  cat > "${DROPIN_FILE}" <<EOF
[Service]
WorkingDirectory=${REPO_ROOT}
Environment=PYTHONUNBUFFERED=1
# Reset ExecStart from base unit and replace with venv invocation
ExecStart=
ExecStart=${VENV_PY} -m pi_runtime.main
EOF
fi

# =============================================================================
# Reload systemd and start tower service
# =============================================================================

echo "Reloading systemd..."
systemctl daemon-reload

echo "Enabling/starting ${UNIT}..."
systemctl enable --now "${UNIT}"

echo
echo "Done."
echo "Useful commands:"
echo "  systemctl status ${UNIT} --no-pager"
echo "  journalctl -u ${UNIT} -n 200 --no-pager"
echo "  systemctl status pigpiod.service --no-pager"
