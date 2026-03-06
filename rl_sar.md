# rl_sar Change Log

## 2026-03-06

### 冷启动恢复与再联通成功经验

#### 结论
- 机器狗断电或 Jetson 重启后，需要重新完成：
  - ROS 环境 `source`
  - USB-CAN 枚举确认
  - `can0` bringup
  - `arx_x5_bridge.py`
  - `rl_real_go2_x5`
- 这轮成功链路已经明确：
  - `arm_hold_enabled: true`
  - `arm_lock: false`
  - 主控启动后必须看到 `Arm bridge state stream detected`
  - 联动主控时，用户只发布 `/arm_joint_pos_cmd`，不要再手工发布 `/arx_x5/joint_cmd`

#### 冷启动标准流程

1. 进入工作区并更新代码（如果远端有新提交）
   - `cd ~/rl_ras_n`
   - `git pull`

2. 重新加载 ROS 与工作区环境
   - `source /opt/ros/foxy/setup.bash`
   - `source ~/rl_ras_n/install/setup.bash`

3. 确认 USB-CAN 实际设备名
   - `ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null`
   - 不要写死 `/dev/ttyACM0`
   - 这轮现场成功设备是 `/dev/ttyACM1`

4. 重新 bringup `can0`
   - `./scripts/setup_arx_can.sh /dev/ttyACM1 can0 8`
   - `ip -s -d link show can0`

5. 终端 A 启动 X5 bridge
   - `export ROS_LOCALHOST_ONLY=1`
   - `export ROS_DOMAIN_ID=42`
   - `export ARX5_SDK_ROOT=/home/unitree/arx5-sdk`
   - `export ARX5_SDK_PYTHON_PATH=/home/unitree/arx5-sdk/python`
   - `export ARX5_SDK_LIB_PATH=/home/unitree/arx5-sdk/lib/aarch64`
   - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run rl_sar arx_x5_bridge.py --ros-args -p interface_name:=can0 -p require_sdk:=true -p require_initial_state:=true`

6. 终端 B 启动主控
   - `export ROS_LOCALHOST_ONLY=1`
   - `export ROS_DOMAIN_ID=42`
   - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run rl_sar rl_real_go2_x5 eth0`

#### 主控联动判定条件
- 主控启动日志必须出现：
  - `arm_joint_command_topic: /arm_joint_pos_cmd, arm_hold_enabled: true, arm_lock: false`
  - `Arm bridge state stream detected: topic=/arx_x5/joint_state, dof=6`
- 若一直只有：
  - `Arm bridge state is missing or stale. Using shadow arm state until bridge recovers.`
  - 说明主控没有拿到机械臂真实状态，不能继续做 arm 联动测试

#### 主控 + 机械臂联动操作顺序

1. 主控终端按 `0`
   - 机器人起身

2. 如果先测机械臂，不要先按 `1`
   - 先保持站立，确认 arm 联动稳定

3. 新终端发布 6 维机械臂目标到 `/arm_joint_pos_cmd`
   - 示例：
   - `ros2 topic pub --once /arm_joint_pos_cmd std_msgs/msg/Float32MultiArray "{data: [0.60,3.00,1.00,0.20,0.00,0.00]}"`

4. 回到主控终端按 `2`
   - 主控会把最近一次 `/arm_joint_pos_cmd` 接管为新的 `arm_hold_pose`
   - 然后通过 `/arx_x5/joint_cmd` 发给 bridge
   - bridge 再下发到真实机械臂

5. 机械臂运动稳定后，再按 `1`
   - 底座进入 RL
   - 机械臂继续保持刚才通过 `2` 设进去的姿态

6. 按 `3`
   - 机械臂恢复默认姿态

#### 本轮收敛出的关键经验
- 机械臂单独通过 bridge 可动，不代表主控联动已经通；主控必须拿到 `/arx_x5/joint_state`
- 联动主控时：
  - `/arx_x5/joint_cmd` 是内部桥接话题
  - 用户应该操作的是 `/arm_joint_pos_cmd` + 主控按键 `2`
- 之前“bridge 一接上就乱飞”的根因不是 CAN，而是主控在 `Passive/GetUp` 阶段把不安全的 arm 指令透传给了 bridge
- 当前已修复：
  - `WriteArmCommandToExternal()` 在 `arm_hold_enabled=true` 时，优先发送 `arm_hold_position + fixed_kp/fixed_kd`
  - 启动阶段不再把脏 `q/kd` 直接推给机械臂
- 仍需注意：
  - 如果现场机械臂当前物理姿态与 `arm_hold_pose` 差很大，主控启动后会主动回到 hold pose；这属于受控回位，不是随机乱飞

### WARNING
- USB-CAN 设备的 `/dev/ttyACM*` 编号不是固定值，不要写死成 `/dev/ttyACM0`。
- 现场联调前先执行：
  - `ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null`
  - `ls -l /dev/serial/by-id/ 2>/dev/null`
- 推荐优先使用 `/dev/serial/by-id/...` 的稳定路径；如果只能用 `ttyACM*`，必须先确认当前实际编号，再执行 `./scripts/setup_arx_can.sh <实际设备> can0 8`。

### 背景
- Go2 + X5 的 sim2real 排障继续推进，机械臂 CAN 链路从“无响应”推进到“ARX SDK 可成功初始化，bridge 可稳定运行”。
- `go2_x5` 双进程链路已明确需要拆分 DDS：
  - `arx_x5_bridge.py` 使用 `rmw_cyclonedds_cpp`
  - `rl_real_go2_x5` 使用 `rmw_fastrtps_cpp`
- 当前剩余主问题已从机械臂桥接收敛到 `rl_real_go2_x5` 自身：在正确 RMW 组合下，主控仍会持续打印 `bad_alloc caught: std::bad_alloc`。

### 改动

#### 1) X5 桥接链路验证完成
- 结论：
  - `setup_arx_can.sh` + `can0` + ARX SDK 在 Jetson 侧可成功 bringup。
  - `arx_x5_bridge.py` 在 strict 模式下可成功通过 probe 并进入真实 backend。
- 结果：
  - 机械臂桥接不再是当前主阻塞点。

#### 2) DDS 运行约束进一步固化
- 结论：
  - 不能把 `go2_x5` 全流程都强制成 `rmw_cyclonedds_cpp`。
  - 单独直跑 `rl_real_go2_x5` 时，`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 会触发 ROS2 domain 初始化失败。
- 建议固定组合：
  - `arx_x5_bridge.py` -> `rmw_cyclonedds_cpp`
  - `rl_real_go2_x5` -> `rmw_fastrtps_cpp`

#### 3) Go2-X5 真机键盘流程澄清
- 结论：
  - `0` 只负责从 passive 切到 `GetUp`
  - `1` 才负责进入 `RLLocomotion`
- 结果：
  - “按下 0 没有直接开始走”不是控制语义错误，而是当前 FSM 设计如此。

#### 4) `rl_real_go2_x5` 启动诊断日志增强
- 文件：`src/rl_sar/src/rl_real_go2_x5.cpp`
- 改动：
  - 新增 `[Boot] ...` 启动阶段日志，覆盖：
    - `ChannelFactory` 初始化
    - `rclcpp::init`
    - ROS2 node 和 `/cmd_vel` 订阅创建
    - arm ROS interface 建立
    - Unitree DDS channel 建立
    - `MotionSwitcherClient` 初始化
    - control loop 启动
- 结果：
  - 下一轮 Jetson 复现实验时，可以直接判断 `bad_alloc` 卡在启动链路的哪一步，而不是只看到最终刷屏。

#### 5) 当前未闭环问题
- `rl_real_go2_x5` 在正确的 RMW 组合下仍会持续输出 `bad_alloc caught: std::bad_alloc`。
- 从最新日志看，问题更像 `go2` 主控侧早期启动链路，而不是 X5 bridge。
- 下一步排查需要基于新的 `[Boot]` 日志继续收敛。

## 2026-03-05

### 背景
- 代码审查发现了 4 类问题：
  - 高频控制循环按毫秒截断，`dt=0.002/0.005` 会出现系统性频率误差。
  - `rl_real_go2_x5` 与 `rl_sim` 的机械臂平滑状态存在跨线程并发读写风险。
  - `rl_sim` 对 `joint_mapping` 访问缺少强校验，异常配置可能越界。
  - 当前仓库测试未接入 CTest，回归保护不足。
- 用户新增要求：仓库只保留 `go2` 和 `go2_x5` 的机器人数据。

### 改动

#### 1) 控制循环精度修复（面向 Jetson 高频循环）
- 文件：`src/rl_sar/library/core/loop/loop.hpp`
- 改动：
  - 新增 `LoopFunc::ComputeSleepDuration(...)`，使用微秒级计算剩余睡眠时间。
  - 循环等待逻辑从“整数毫秒”改为“微秒”。
- 结果：
  - 避免毫秒截断导致的周期漂移，降低 200Hz~500Hz 控制环的时序误差。

#### 2) `rl_sim` 机械臂状态并发安全修复
- 文件：
  - `src/rl_sar/src/rl_sim.cpp`
  - `src/rl_sar/include/rl_sim.hpp`
- 改动：
  - 将机械臂关键运行态（hold/sequence/smoothing/size/latest）统一纳入 `arm_command_mutex` 保护。
  - `RobotControl`、`RunModel`、`StartArmSequence`、`StopArmSequence`、`ArmJointCommandCallback` 的共享字段访问改为加锁。
  - 输出覆盖阶段先拷贝本地快照（`arm_hold_enabled_local` 等）再使用，避免跨线程竞争。
- 结果：
  - 消除已识别的数据竞争点，减少偶发抖动和未定义行为风险。

#### 3) `rl_sim` `joint_mapping` 校验与防越界修复
- 新增文件：`src/rl_sar/library/core/rl_sdk/rl_validation.hpp`
- 改动：
  - 新增 `RLValidation::ValidateJointMapping(...)`。
  - `RL_Sim` 构造时对 `base.yaml` 的 `joint_mapping` 做强校验（不合法直接抛错）。
  - `GetState`/`SetCommand` 中增加映射边界检查与一次性告警，防止运行时越界崩溃。
- 结果：
  - 配置错误从“潜在崩溃”转为“启动期显式失败/运行期受控降级”。

#### 4) `rl_real_go2_x5` 机械臂并发安全修复
- 文件：`src/rl_sar/src/rl_real_go2_x5.cpp`
- 改动：
  - `InitializeArmCommandState()` 增加 `arm_command_mutex` 保护。
  - `RunModel` 内对 arm 平滑状态读写统一在同一锁内处理，再写入 `obs.arm_joint_command`。
  - 键盘路径与 callback 读取 `arm_command_size` 改为锁保护后的本地快照。
- 结果：
  - 修复 `loop_control` 与 `loop_rl` 对 arm 平滑状态的竞争。

#### 5) TDD 与测试接入
- 文件：
  - `src/rl_sar/CMakeLists.txt`
  - `src/rl_sar/test/test_loop_timing_precision.cpp`
  - `src/rl_sar/test/test_joint_mapping_validation.cpp`
- 过程（TDD）：
  - 先新增测试（红灯）：编译失败，提示缺失 `ComputeSleepDuration` 和 `rl_validation.hpp`。
  - 再实现代码（绿灯）：编译通过，`ctest` 通过。
- 结果：
  - 新增 2 个可自动执行的回归测试，用于守护本次修复。

#### 6) 机器人数据精简（按用户要求）
- 删除目录：
  - `policy/a1`
  - `policy/b2`
  - `policy/b2w`
  - `policy/g1`
  - `policy/go2w`
  - `policy/gr1t1`
  - `policy/gr1t2`
  - `policy/l4w4`
  - `policy/lite3`
  - `policy/tita`
- 保留目录：
  - `policy/go2`
  - `policy/go2_x5`

#### 7) sim2sim 模式对齐到 go2_x5 真机按键流
- 文件：
  - `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`
  - `src/rl_sar/src/rl_real_go2_x5.cpp`
  - `src/rl_sar/include/rl_real_go2_x5.hpp`
  - `src/rl_sar/include/go2_x5_control_logic.hpp`
  - `policy/go2_x5/robot_lab/config.yaml`
  - `src/rl_sar/test/test_go2_x5_control_logic.cpp`
  - `src/rl_sar/CMakeLists.txt`
- 改动：
  - `Key[1]` 增加可配置行为：
    - `key1_prefer_navigation_mode: true` 时进入 RL 后默认开启导航模式（`/cmd_vel`）。
    - `false` 时保持原固定速度 `fixed_cmd_*`。
  - `Key[2]` 改为“话题优先 + 回退链路”：
    - 优先使用最近收到的 `/arm_joint_pos_cmd`。
    - 无可用话题时回退 `arm_key_pose`，再回退 `arm_hold_pose`。
  - 保留 `Key[3]` 恢复默认机械臂姿态逻辑不变。
  - 新增纯函数控制选择层 `go2_x5_control_logic.hpp`，并由真机逻辑复用。
  - 新增单测 `test_go2_x5_control_logic`，覆盖 `Key[1]/Key[2]` 语义与回退链路。
- 结果：
  - 真机按键流程与 sim2sim 风格对齐，更接近“0 起身 / 1 RL指令驱动 / 2 机械臂到发布位姿 / 3 机械臂复原”的部署诉求。

#### 8) Ctrl+C 安全接管（软着陆 + 机械臂回收）
- 文件：
  - `src/rl_sar/src/rl_real_go2_x5.cpp`
  - `src/rl_sar/include/rl_real_go2_x5.hpp`
  - `policy/go2_x5/robot_lab/config.yaml`
  - `README.md`
  - `README_CN.md`
- 改动：
  - `RL_Real_Go2X5` 析构时新增 `ExecuteSafeShutdownSequence()`：
    - 先平滑插值到下趴姿态（默认腿部使用 go2_x5 lying pose）。
    - 机械臂回收到 `arm_shutdown_pose`（回退 `arm_hold_pose`）。
    - 保持短时间后再恢复机身内置运动服务。
  - 新增参数：
    - `shutdown_soft_land_sec`
    - `shutdown_hold_sec`
    - `arm_shutdown_pose`
  - ROS1 信号处理移除 `exit(0)`，仅 `ros::shutdown()`，确保析构链执行安全退出序列。
- 结果：
  - 在 `Ctrl+C` 退出时，具备可接管的平滑退场流程，降低大体型机器人突停风险。

### 文档同步
- 更新：
  - `README.md`
  - `README_CN.md`
- 说明：
  - 明确策略数据仅保留 `go2` 与 `go2_x5`，降低 Jetson 部署体积和维护复杂度。

### 证据（命令与结果）
- 构建：
  - `cmake --build cmake_build -j4` 通过
- 测试：
  - `ctest --test-dir cmake_build --output-on-failure`
  - 结果：`3/3` 通过

### Jetson 运行注意
- 本次改动优先保证高频循环精度与并发稳定性，适合 Jetson 上 200Hz~500Hz 控制回路。
- 当前环境未执行 ROS2/Unitree 真机目标编译（本地缺少对应 SDK 依赖）；真机联调时需在 Jetson 环境完成 `colcon` 构建与实测。
