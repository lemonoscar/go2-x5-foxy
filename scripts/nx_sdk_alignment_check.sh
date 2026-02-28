#!/usr/bin/env bash
set -euo pipefail

echo "=== NX SDK Alignment Check ==="
echo "time: $(date -Iseconds)"
echo "host: $(hostname)"
echo

echo "[1] ROS/Python/Conda"
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CONDA_DEFAULT_ENV=${CONDA_DEFAULT_ENV:-}"
command -v python3 >/dev/null && python3 -V || true
echo

echo "[2] Repo version"
if [ -d "${HOME}/rl_ras_n/.git" ]; then
  git -C "${HOME}/rl_ras_n" rev-parse --short HEAD || true
  git -C "${HOME}/rl_ras_n" status --short || true
  git -C "${HOME}/rl_ras_n" submodule status || true
else
  echo "~/rl_ras_n not found"
fi
echo

echo "[3] Unitree SDK2 source fingerprint"
SDK2_DIR="${HOME}/rl_ras_n/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2"
if [ -d "${SDK2_DIR}/.git" ]; then
  git -C "${SDK2_DIR}" rev-parse --short HEAD || true
  git -C "${SDK2_DIR}" branch --show-current || true
else
  echo "unitree_sdk2 git dir not found: ${SDK2_DIR}"
fi
echo

echo "[4] ARX SDK fingerprint"
ARX_DIR="${HOME}/arx5-sdk"
if [ -d "${ARX_DIR}/.git" ]; then
  git -C "${ARX_DIR}" rev-parse --short HEAD || true
  git -C "${ARX_DIR}" branch --show-current || true
else
  echo "arx5-sdk git dir not found: ${ARX_DIR}"
fi
python3 - <<'PY' || true
import os
try:
    import arx5_interface
    print("arx5_interface:", arx5_interface.__file__)
except Exception as e:
    print("arx5_interface import failed:", e)
PY
echo

echo "[5] Runtime linker check"
BIN_REAL="${HOME}/rl_ras_n/install/lib/rl_sar/rl_real_go2_x5"
BIN_BRIDGE="${HOME}/rl_ras_n/install/lib/rl_sar/arx_x5_bridge.py"
if [ -x "${BIN_REAL}" ]; then
  echo "--- ldd rl_real_go2_x5 ---"
  ldd "${BIN_REAL}" | egrep -i "unitree|dds|cyclone|fastrtps|iceoryx|yaml|protobuf|stdc\\+\\+" || true
else
  echo "binary not found: ${BIN_REAL}"
fi
python3 - <<'PY' || true
import os
try:
    import arx5_interface
    so = arx5_interface.__file__
    print("--- ldd arx5_interface ---")
    os.system(f"ldd {so} | egrep -i 'stdc\\+\\+|python|can|usb|pthread|gcc_s|glibc' || true")
except Exception as e:
    print("skip arx5_interface ldd:", e)
PY
echo

echo "[6] CAN and network"
ip -br a || true
ip -br link | egrep 'can|slcan' || true
ip -details link show can0 || true
lsusb | egrep -i "canable|16d0:117e" || true
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
echo

echo "[7] Memory baseline"
free -h || true
df -h /dev/shm || true
ps -eo pid,comm,rss,%mem --sort=-rss | head -n 20 || true
echo

echo "[8] Recent kernel OOM traces (requires sudo permission if restricted)"
if command -v journalctl >/dev/null; then
  journalctl -k -b | egrep -i "killed process|oom|out of memory|rl_real_go2_x5|ros2" | tail -n 120 || true
fi
echo

echo "=== Check finished ==="
