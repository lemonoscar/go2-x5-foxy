# go2-x5-foxy

ROS2 Foxy deployment repository for Unitree **Go2** and **Go2-X5** (sim + real).

## Scope

This repository is intentionally trimmed for deployment stability and size:

- Supported robots: `go2`, `go2_x5`
- Supported ROS distro: `foxy`
- Kept policy data:
  - `policy/go2/`
  - `policy/go2_x5/`
- Removed policy data for other robots.

## Environment

Recommended runtime target: **Jetson** (Ubuntu 20.04 + ROS2 Foxy).

Minimum dependencies:

```bash
sudo apt update
sudo apt install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

ROS setup:

```bash
source /opt/ros/foxy/setup.bash
```

## Build

### Standard ROS build (recommended)

```bash
cd /home/lemon/Issac/rl_ras_n
./build.sh
source /home/lemon/Issac/rl_ras_n/install/setup.bash
```

`build.sh` in this fork is Foxy-only.

### Standalone CMake build (hardware-focused tools)

```bash
cd /home/lemon/Issac/rl_ras_n
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON
cmake --build cmake_build -j$(nproc)
```

## Gazebo Simulation

Terminal 1:

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=go2_x5
```

Terminal 2:

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 run rl_sar rl_sim
```

## Go2-X5 Real Robot

### Single process

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
/home/lemon/Issac/rl_ras_n/install/lib/rl_sar/rl_real_go2_x5 eth0
```

### Dual process (leg + arm bridge)

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

## Runtime Controls (RL process)

- `0`: Get up
- `1`: Enter RL locomotion
  - default in `go2_x5/robot_lab/config.yaml`: enables navigation mode and consumes `/cmd_vel`
  - set `key1_prefer_navigation_mode: false` to use `fixed_cmd_*` instead
- `9`: Get down
- `P`: Passive mode
- `W/A/S/D/Q/E`: velocity commands
- `Space`: clear velocity
- `N`: toggle `/cmd_vel` navigation mode (manual override)

Go2-X5 arm shortcuts:

- `2`: hold arm at latest `/arm_joint_pos_cmd` (fallback to `arm_key_pose` then `arm_hold_pose`)
- `3`: restore default arm pose
- `4`: arm hold ON/OFF

## Go2-X5 Sim2Real Key Flow

Target flow for deployment:

1. Press `0` -> robot gets up.
2. Press `1` -> enter RL state; by default commands are from `/cmd_vel`.
3. Publish arm target to `/arm_joint_pos_cmd`, then press `2` -> arm moves/holds at that published target.
4. Press `3` -> arm returns to default pose.

## Minimal Reproducible Sim2Real (Jetson, Go2-X5)

Assumptions:

- Jetson is connected to Go2 via `eth0`.
- X5 arm CAN interface is `can0`.
- `ARX5_SDK_ROOT` is available (example: `/home/unitree/arx5-sdk`).
- Repository has been built by `./build.sh`.

Terminal 1 (start real deployment, leg + arm bridge):

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

Terminal 2 (publish locomotion command):

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.00, z: 0.00}, angular: {x: 0.00, y: 0.00, z: 0.00}}" -r 20
```

Terminal 3 (publish arm target):

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 topic pub /arm_joint_pos_cmd std_msgs/msg/Float32MultiArray \
  "{data: [0.8, 2.4, 1.6, 0.0, 0.3, 0.3]}" -r 10
```

Keyboard actions on Terminal 1 (strict order):

1. Press `0` (get up).
2. Press `1` (enter RL, default uses `/cmd_vel`).
3. Press `2` (arm takes latest `/arm_joint_pos_cmd` target).
4. Press `3` (arm restore default pose).

Safe takeover (Ctrl+C):

- Press `Ctrl+C` in Terminal 1 to stop deployment.
- Runtime performs a graceful exit sequence before handing control back:
  - smooth leg soft-landing to lying pose
  - arm retract to `arm_shutdown_pose` (fallback: `arm_hold_pose`)
  - restore built-in motion service (`normal/sport_mode`)
- Tunable params (`policy/go2_x5/robot_lab/config.yaml`):
  - `shutdown_soft_land_sec`
  - `shutdown_hold_sec`
  - `arm_shutdown_pose`

Quick checks:

```bash
ros2 topic hz /arx_x5/joint_state
ros2 topic echo --once /arx_x5/joint_cmd
```

## Jetson Notes

- This branch prioritizes control-loop timing precision and thread safety for high-frequency loops.
- Keep CPU governor/performance mode configured for deterministic control timing.
- Verify DDS split config in dual-node mode (`CycloneDDS` for bridge, `FastDDS` for go2 node) before field runs.

## Tests

Standalone tests (CMake mode):

```bash
cd /home/lemon/Issac/rl_ras_n
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON -DBUILD_TESTING=ON
cmake --build cmake_build -j$(nproc)
ctest --test-dir cmake_build --output-on-failure
```

Current tests:

- `test_loop_timing_precision`
- `test_joint_mapping_validation`
- `test_go2_x5_control_logic`

## Repository Layout

- `src/rl_sar/`: core runtime and FSM
- `src/robot_msgs/`: ROS message definitions
- `src/robot_joint_controller/`: robot joint controller
- `policy/go2/`: Go2 policies/config
- `policy/go2_x5/`: Go2-X5 policies/config
- `scripts/`: dependency/install helper scripts
- `rl_sar.md`: engineering change log
