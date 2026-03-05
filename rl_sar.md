# rl_sar Change Log

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
