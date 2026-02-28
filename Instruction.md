# Instruction

本文件用于 `rl_ras_n`（ROS2 Foxy 专用）下的 Go2-X5 `sim2real` 部署。

## 0. 前置

- 系统已安装 ROS2 Foxy。
- 你有机械臂 SDK 仓库权限（`arx5-sdk`）。
- 机械臂总线接口为 `can0`（按实际调整）。

## 1. 进入 Foxy 环境

```bash
source /opt/ros/foxy/setup.bash
echo $ROS_DISTRO
# 需要输出: foxy
```

## 2. 安装 Unitree SDK2（机器狗底盘）

```bash
cd /home/lemon/Issac/rl_ras_n/src/rl_sar/library/thirdparty/robot_sdk/unitree
rm -rf unitree_sdk2
git clone -b 2.0.0 https://github.com/unitreerobotics/unitree_sdk2.git unitree_sdk2

cd unitree_sdk2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

验证：

```bash
test -f /home/lemon/Issac/rl_ras_n/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2/CMakeLists.txt && echo "unitree_sdk2 OK"
```

## 3. 安装 ARX5 SDK（机械臂）

```bash
cd /home/unitree
git clone https://github.com/real-stanford/arx5-sdk.git
cd arx5-sdk
```

### 3.1 创建环境（无 mamba 用 conda）

```bash
conda env create -f conda_environments/py310_environment.yaml
conda activate arx-py310
```

如果你要复用现有环境，例如 `go2x5_s2r`，可改成：

```bash
conda activate go2x5_s2r
```

### 3.2 编译 C++ 与 Python 绑定

```bash
cd /home/unitree/arx5-sdk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

python -m pip install pybind11
PYBIND_PATH=$(python -m pip show pybind11 | awk '/Location:/{print $2}')
cmake -S python -B python/build -DPYBIND_PATH=$PYBIND_PATH
cmake --build python/build -j$(nproc)
```

### 3.3 设置运行时路径

```bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk
export PYTHONPATH=$ARX5_SDK_ROOT/python:$PYTHONPATH
export LD_LIBRARY_PATH=$ARX5_SDK_ROOT/lib/aarch64:$LD_LIBRARY_PATH
python -c "import arx5_interface; print('arx5_interface OK')"
```

建议写入 `~/.bashrc`：

```bash
echo 'export ARX5_SDK_ROOT=/home/unitree/arx5-sdk' >> ~/.bashrc
echo 'export PYTHONPATH=$ARX5_SDK_ROOT/python:$PYTHONPATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$ARX5_SDK_ROOT/lib/aarch64:$LD_LIBRARY_PATH' >> ~/.bashrc
```

## 4. 构建 `rl_ras_n`

```bash
source /opt/ros/foxy/setup.bash
cd /home/lemon/Issac/rl_ras_n
./build.sh
source /home/lemon/Issac/rl_ras_n/install/setup.bash
```

## 5. 确认双通道配置（12腿 + 6臂）

检查以下文件：

- `policy/go2_x5/base.yaml`
- `policy/go2_x5/robot_lab/config.yaml`

关键字段：

```yaml
arm_control_mode: "split"
arm_joint_start_index: 12
arm_joint_count: 6
arm_bridge_require_state: true
arm_bridge_cmd_topic: "/arx_x5/joint_cmd"
arm_bridge_state_topic: "/arx_x5/joint_state"
```

## 6. 启动双通道真机（推荐）

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk
# 默认 RMW 已拆分：
# - arx_x5_bridge.py 使用 rmw_cyclonedds_cpp
# - rl_real_go2_x5 使用 rmw_fastrtps_cpp

ros2 launch rl_sar go2_x5_real_dual.launch.py \
  network_interface:=<YOUR_NETWORK_INTERFACE> \
  arm_interface_name:=can0
```

## 7. 运行前检查

```bash
ip -br a
ip -details link show can0
ros2 topic hz /arx_x5/joint_cmd
ros2 topic hz /arx_x5/joint_state
```

`/arx_x5/joint_state` 有稳定频率后，再执行动作流程。

## 8. 上机动作顺序

1. 机器人急停可触发。
2. 按 `0` 起身。
3. 观察稳定后按 `1` 进入 RL。
4. 再落地低速测试（先小速度）。

## 9. 失败回退

1. 立即按 `P` 或硬急停。
2. 保留日志与配置快照。
3. 优先排查 bridge 状态链路，再排查策略参数。

## 10. 本次联调经验总结（机械臂 + 机器狗）

1. Python/ROS 环境必须纯净。若在 conda 环境里直接跑 ROS2，可能出现 `rclpy._rclpy` 不匹配（如 cp313 vs Foxy cp38）。
2. ARX bridge 在 NX 上使用 `rmw_fastrtps_cpp` 可能触发异常；`rmw_cyclonedds_cpp` 可稳定启动。
3. 不能全局强制所有进程都用 Cyclone。`rl_real_go2_x5` 与 Unitree 通道组合下建议维持 `rmw_fastrtps_cpp`。
4. 双通道 launch 已按进程拆分 RMW（已写入仓库）：
   - `arx_x5_bridge.py` -> `rmw_cyclonedds_cpp`
   - `rl_real_go2_x5` -> `rmw_fastrtps_cpp`
5. `can0` 不是固定指 USB-CAN。NX 上常见 `can0` 是板载 `mttcan`，USB-CAN 可能是 `can1` 或需用 `slcand` 从 `/dev/ttyACM0` 映射。
6. 若 bridge 日志报 `None of the motors are initialized`，表示 SDK 已加载但机械臂总线无有效响应，优先查接口/波特率/供电/布线。
7. 稳定流程是先单独跑桥，再拉双进程：
   - 先确认 `arx_x5_bridge.py` 正常发布 `/arx_x5/joint_state`
   - 再启动 `go2_x5_real_dual.launch.py`

## 11. 每次调试前准备清单（强制执行）

### 11.1 终端环境

```bash
conda deactivate
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
unset RMW_IMPLEMENTATION
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk
```

### 11.2 机器狗网络检查

```bash
ip -br a
# 确认 network_interface (例如 eth0) 存在且 UP
```

### 11.3 机械臂 CAN 接口检查（USB 连接场景）

1. 查看 USB 设备是否识别（例如 CANable2）：

```bash
lsusb | grep -i -E "canable|16d0:117e"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

2. 若无可用 `canX`，通过 `slcand` 映射（示例映射为 `can0`）：

```bash
sudo modprobe can can_raw can_dev slcan
sudo pkill slcand || true
sudo slcand -o -c -f -s8 /dev/ttyACM0 can0   # -s8: 1Mbps
sudo ip link set can0 up
ip -details link show can0
```

3. 若 1Mbps 不通，改 500kbps 再试：

```bash
sudo ip link set can0 down
sudo pkill slcand || true
sudo slcand -o -c -f -s6 /dev/ttyACM0 can0   # -s6: 500kbps
sudo ip link set can0 up
```

### 11.4 先单独验证 ARX bridge（推荐）

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run rl_sar arx_x5_bridge.py --ros-args \
  -p interface_name:=can0 -p require_initial_state:=false
```

看到以下关键日志后再进行双通道联调：

- `ARX backend ready: arx5_interface`
- `ARX bridge started`

### 11.5 双通道标准启动命令

```bash
ros2 launch rl_sar go2_x5_real_dual.launch.py \
  network_interface:=eth0 \
  arm_interface_name:=can0 \
  bridge_rmw_implementation:=rmw_cyclonedds_cpp \
  go2_rmw_implementation:=rmw_fastrtps_cpp
```

### 11.6 快速故障定位顺序

1. bridge 起不来：先看 `interface_name` 和 CAN 链路。
2. bridge 能起但 `rl_real_go2_x5` 被 `-9`：查系统日志（OOM/内核杀进程）与独立进程运行差异。
3. `/arx_x5/joint_state` 无频率：先解决 bridge 和 CAN，再看策略侧。
