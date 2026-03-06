#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/common.sh"

usage() {
    cat <<'EOF'
Usage:
  setup_arx_can.sh [CAN_DEV] [CAN_IF] [SLCAN_SPEED_CODE]

Examples:
  ./scripts/setup_arx_can.sh
  ./scripts/setup_arx_can.sh /dev/ttyACM0 can0 8
  CAN_DEV=/dev/ttyUSB0 CAN_IF=can1 SLCAN_SPEED_CODE=6 ./scripts/setup_arx_can.sh

Defaults:
  CAN_DEV=/dev/ttyACM0
  CAN_IF=can0
  SLCAN_SPEED_CODE=8
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

CAN_DEV="${1:-${CAN_DEV:-/dev/ttyACM0}}"
CAN_IF="${2:-${CAN_IF:-can0}}"
SLCAN_SPEED_CODE="${3:-${SLCAN_SPEED_CODE:-8}}"
WAIT_RETRIES="${WAIT_RETRIES:-20}"
WAIT_INTERVAL_SEC="${WAIT_INTERVAL_SEC:-0.2}"

print_header "[Setting up ARX SocketCAN]"
print_info "CAN_DEV=${CAN_DEV}"
print_info "CAN_IF=${CAN_IF}"
print_info "SLCAN_SPEED_CODE=${SLCAN_SPEED_CODE}"

if [[ ! -e "${CAN_DEV}" ]]; then
    print_error "Device not found: ${CAN_DEV}"
    exit 1
fi

for cmd in sudo modprobe slcand ip; do
    if ! command -v "${cmd%% *}" >/dev/null 2>&1; then
        print_error "Required command not found: ${cmd%% *}"
        exit 1
    fi
done

print_info "Loading CAN kernel modules..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe slcan

print_info "Restarting slcand on ${CAN_IF}..."
sudo pkill slcand || true
sudo ip link set "${CAN_IF}" down || true
sudo slcand -o -c -f "-s${SLCAN_SPEED_CODE}" "${CAN_DEV}" "${CAN_IF}"

sleep 1
for _ in $(seq 1 "${WAIT_RETRIES}"); do
    if ip link show "${CAN_IF}" >/dev/null 2>&1; then
        break
    fi
    sleep "${WAIT_INTERVAL_SEC}"
done

if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
    print_error "Interface ${CAN_IF} was not created by slcand."
    exit 1
fi

print_info "Bringing up ${CAN_IF}..."
sudo ip link set "${CAN_IF}" up

print_success "SocketCAN interface is ready."
print_separator
ip -details link show "${CAN_IF}"
print_separator
ip -s -d link show "${CAN_IF}"
print_separator

if command -v candump >/dev/null 2>&1; then
    print_info "Optional 2s candump probe on ${CAN_IF} (Ctrl+C to skip)..."
    timeout 2s candump "${CAN_IF}" || true
else
    print_warning "candump not found; skip passive bus probe."
fi
