# go2-x5-foxy

面向 Unitree **Go2** 与 **Go2-X5** 的 ROS2 Foxy 仿真/真机部署仓库。

## 仓库范围

本仓库已做部署导向精简：

- 支持机器人：`go2`、`go2_x5`
- 支持 ROS 版本：`foxy`
- 保留策略数据：
  - `policy/go2/`
  - `policy/go2_x5/`
- 其余机器人策略目录已移除。

## 环境要求

推荐目标平台：**Jetson**（Ubuntu 20.04 + ROS2 Foxy）。

基础依赖：

```bash
sudo apt update
sudo apt install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

ROS 环境：

```bash
source /opt/ros/foxy/setup.bash
```

## 编译方式

### 标准 ROS 编译（推荐）

```bash
cd /home/lemon/Issac/rl_ras_n
./build.sh
source /home/lemon/Issac/rl_ras_n/install/setup.bash
```

本分支 `build.sh` 已限定为 Foxy-only。

### 独立 CMake 编译（偏硬件部署）

```bash
cd /home/lemon/Issac/rl_ras_n
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON
cmake --build cmake_build -j$(nproc)
```

## Gazebo 仿真

终端 1：

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=go2_x5
```

终端 2：

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 run rl_sar rl_sim
```

## Go2-X5 真机运行

### 单进程

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
/home/lemon/Issac/rl_ras_n/install/lib/rl_sar/rl_real_go2_x5 eth0
```

### 双进程（腿 + 机械臂桥）

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk
ros2 launch rl_sar go2_x5_real_dual.launch.py \
  network_interface:=eth0 \
  arm_interface_name:=can0 \
  bridge_rmw_implementation:=rmw_cyclonedds_cpp \
  go2_rmw_implementation:=rmw_fastrtps_cpp
```

## 运行控制（RL 进程按键）

- `0`：起身
- `1`：进入 RL 行走
  - `go2_x5/robot_lab/config.yaml` 默认：开启导航模式，采用 `/cmd_vel`
  - 若设 `key1_prefer_navigation_mode: false`，则改为使用 `fixed_cmd_*`
- `9`：趴下
- `P`：被动模式
- `W/A/S/D/Q/E`：速度指令
- `Space`：速度清零
- `N`：切换 `/cmd_vel` 导航模式（手动覆盖）

Go2-X5 机械臂快捷键：

- `2`：机械臂保持在最近一次 `/arm_joint_pos_cmd` 发布位姿（若无则回退 `arm_key_pose`，再回退 `arm_hold_pose`）
- `3`：机械臂恢复默认姿态
- `4`：机械臂保持 ON/OFF

## Go2-X5 Sim2Real 按键流程

推荐部署流程：

1. 按 `0`，机器人起身。
2. 按 `1`，进入 RL 状态，默认从 `/cmd_vel` 取指令。
3. 向 `/arm_joint_pos_cmd` 发布机械臂目标后按 `2`，机械臂到达并保持该目标。
4. 按 `3`，机械臂恢复默认位姿。

## 最小可复现 Sim2Real 流程（Jetson + Go2-X5）

前置假设：

- Jetson 与 Go2 通过 `eth0` 连接。
- X5 机械臂 CAN 口为 `can0`。
- 已配置 `ARX5_SDK_ROOT`（例如 `/home/unitree/arx5-sdk`）。
- 仓库已通过 `./build.sh` 完成编译。

终端 1（启动真机双进程：腿 + 机械臂桥）：

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk
ros2 launch rl_sar go2_x5_real_dual.launch.py \
  network_interface:=eth0 \
  arm_interface_name:=can0 \
  bridge_rmw_implementation:=rmw_cyclonedds_cpp \
  go2_rmw_implementation:=rmw_fastrtps_cpp
```

终端 2（发布机身指令）：

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.00, z: 0.00}, angular: {x: 0.00, y: 0.00, z: 0.00}}" -r 20
```

终端 3（发布机械臂目标）：

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 topic pub /arm_joint_pos_cmd std_msgs/msg/Float32MultiArray \
  "{data: [0.8, 2.4, 1.6, 0.0, 0.3, 0.3]}" -r 10
```

在终端 1 按键（严格按顺序）：

1. 按 `0`（起身）。
2. 按 `1`（进入 RL，默认采用 `/cmd_vel`）。
3. 按 `2`（机械臂执行最近一次 `/arm_joint_pos_cmd` 目标）。
4. 按 `3`（机械臂恢复默认位姿）。

安全接管（Ctrl+C）：

- 在终端 1 按 `Ctrl+C` 停止部署进程。
- 退出前会执行平滑接管流程，再交还机身控制：
  - 腿部平滑软着陆到下趴姿态
  - 机械臂回收到 `arm_shutdown_pose`（若未配置则回退 `arm_hold_pose`）
  - 恢复机身内置运动服务（`normal/sport_mode`）
- 可调参数（`policy/go2_x5/robot_lab/config.yaml`）：
  - `shutdown_soft_land_sec`
  - `shutdown_hold_sec`
  - `arm_shutdown_pose`

快速检查：

```bash
ros2 topic hz /arx_x5/joint_state
ros2 topic echo --once /arx_x5/joint_cmd
```

## Jetson 注意事项

- 本分支优先优化了高频控制循环计时精度与并发安全。
- 建议将系统调到性能模式，减少控制周期抖动。
- 双节点模式下建议保持 DDS 拆分：
  - bridge：`rmw_cyclonedds_cpp`
  - go2 主节点：`rmw_fastrtps_cpp`

## 测试

在独立 CMake 模式下执行：

```bash
cd /home/lemon/Issac/rl_ras_n
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON -DBUILD_TESTING=ON
cmake --build cmake_build -j$(nproc)
ctest --test-dir cmake_build --output-on-failure
```

当前测试项：

- `test_loop_timing_precision`
- `test_joint_mapping_validation`
- `test_go2_x5_control_logic`

## 目录说明

- `src/rl_sar/`：核心运行逻辑与 FSM
- `src/robot_msgs/`：ROS 消息定义
- `src/robot_joint_controller/`：关节控制器
- `policy/go2/`：Go2 策略与配置
- `policy/go2_x5/`：Go2-X5 策略与配置
- `scripts/`：依赖安装与辅助脚本
- `rl_sar.md`：工程改动记录
