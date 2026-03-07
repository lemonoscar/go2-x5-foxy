/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2_x5.hpp"
#include "go2_x5_control_logic.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <thread>

RL_Real_Go2X5::RL_Real_Go2X5(int argc, char **argv)
{
    std::cout << LOGGER::INFO << "[Boot] go2_x5 constructor begin" << std::endl;

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "go2_x5";
    this->ReadYaml(this->robot_name, "base.yaml");
    this->InitializeArmCommandState();
    this->InitializeArmChannelConfig();
    this->InitializeRealDeploySafetyConfig();
    this->ValidateJointMappingOrThrow("base.yaml");
    std::cout << LOGGER::INFO << "[Boot] base.yaml loaded and arm config validated" << std::endl;

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }
    std::cout << LOGGER::INFO << "[Boot] FSM initialization complete" << std::endl;

#if defined(USE_ROS1) && defined(USE_ROS)
    std::cout << LOGGER::INFO << "[Boot] Creating ROS1 node handle" << std::endl;
    this->ros1_nh = std::make_shared<ros::NodeHandle>();
    std::cout << LOGGER::INFO << "[Boot] Creating /cmd_vel publisher" << std::endl;
    this->cmd_vel_publisher = this->ros1_nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->cmd_vel_subscriber = this->ros1_nh->subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real_Go2X5::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    std::cout << LOGGER::INFO << "[Boot] Creating ROS2 node" << std::endl;
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_go2_x5_node");
    std::cout << LOGGER::INFO << "[Boot] Creating /cmd_vel publisher" << std::endl;
    this->cmd_vel_publisher = ros2_node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS());
    std::cout << LOGGER::INFO << "[Boot] Creating /cmd_vel subscription" << std::endl;
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); });
#endif
    std::cout << LOGGER::INFO << "[Boot] ROS command channel ready" << std::endl;

    std::cout << LOGGER::INFO << "[Boot] Setting up arm command subscriber" << std::endl;
    this->SetupArmCommandSubscriber();
    std::cout << LOGGER::INFO << "[Boot] Setting up arm bridge interface" << std::endl;
    this->SetupArmBridgeInterface();
    std::cout << LOGGER::INFO << "[Boot] Arm ROS interfaces ready" << std::endl;

    // init robot
    std::cout << LOGGER::INFO << "[Boot] Initializing low-level command/state buffers" << std::endl;
    this->InitLowCmd();
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    std::cout << LOGGER::INFO << "[Boot] Low-level buffers ready" << std::endl;

    // create lowcmd publisher
    std::cout << LOGGER::INFO << "[Boot] Creating Unitree lowcmd publisher" << std::endl;
    this->lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    this->lowcmd_publisher->InitChannel();
    // create lowstate subscriber
    std::cout << LOGGER::INFO << "[Boot] Creating Unitree lowstate subscriber" << std::endl;
    this->lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    this->lowstate_subscriber->InitChannel(std::bind(&RL_Real_Go2X5::LowStateMessageHandler, this, std::placeholders::_1), 1);
    // create joystick subscriber
    std::cout << LOGGER::INFO << "[Boot] Creating Unitree joystick subscriber" << std::endl;
    this->joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    this->joystick_subscriber->InitChannel(std::bind(&RL_Real_Go2X5::JoystickHandler, this, std::placeholders::_1), 1);
    std::cout << LOGGER::INFO << "[Boot] Unitree DDS channels ready" << std::endl;

    // init MotionSwitcherClient
    std::cout << LOGGER::INFO << "[Boot] Initializing MotionSwitcherClient" << std::endl;
    this->msc.SetTimeout(10.0f);
    this->msc.Init();
    std::cout << LOGGER::INFO << "[Boot] MotionSwitcherClient ready" << std::endl;
    // Shut down motion control-related service
    while (this->QueryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = this->msc.ReleaseMode();
        if (ret == 0)
        {
            std::cout << "ReleaseMode succeeded." << std::endl;
        }
        else
        {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(1);
    }
    std::cout << LOGGER::INFO << "[Boot] Motion control-related service released" << std::endl;

    std::cout << LOGGER::INFO << "Real deploy target: go2_x5" << std::endl;
    std::cout << LOGGER::INFO << "arm_joint_command_topic: " << this->arm_joint_command_topic
              << ", arm_hold_enabled: " << (this->arm_hold_enabled ? "true" : "false")
              << ", arm_lock: " << (this->params.Get<bool>("arm_lock", false) ? "true" : "false")
              << std::endl;

    // loop
    std::cout << LOGGER::INFO << "[Boot] Starting control loops" << std::endl;
    auto loop_exception_handler = [this](const std::string& loop_name, const std::string& error)
    {
        this->HandleLoopException(loop_name, error);
    };
    this->loop_keyboard = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RL_Real_Go2X5::KeyboardInterface, this), -1, loop_exception_handler);
    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real_Go2X5::RobotControl, this), -1, loop_exception_handler);
    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl",
        this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Real_Go2X5::RunModel, this),
        -1,
        loop_exception_handler);
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();
    std::cout << LOGGER::INFO << "[Boot] Control loops started" << std::endl;

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>(
        "loop_plot", 0.002, std::bind(&RL_Real_Go2X5::Plot, this), -1, loop_exception_handler);
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real_Go2X5::~RL_Real_Go2X5()
{
    this->SafeShutdownNow();
    // Restore built-in motion service so wireless controller can take over after process exit.
    if (!this->QueryMotionStatus())
    {
        int32_t ret = this->msc.SelectMode("normal");
        if (ret != 0)
        {
            ret = this->msc.SelectMode("sport_mode");
        }
        if (ret == 0)
        {
            std::cout << LOGGER::INFO << "Restored motion service: normal(sport_mode)" << std::endl;
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Failed to restore motion service with alias 'normal', error: " << ret << std::endl;
        }
    }
    std::cout << LOGGER::INFO << "RL_Real_Go2X5 exit" << std::endl;
}

void RL_Real_Go2X5::SafeShutdownNow()
{
    std::lock_guard<std::mutex> guard(this->safe_shutdown_mutex);
    if (this->safe_shutdown_done)
    {
        return;
    }

    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_control) this->loop_control->shutdown();
    if (this->loop_rl) this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot) this->loop_plot->shutdown();
#endif

    try
    {
        this->arm_safe_shutdown_active.store(true);
        this->ExecuteSafeShutdownSequence();
    }
    catch (const std::exception& e)
    {
        std::cout << LOGGER::WARNING << "Safe shutdown sequence failed: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << LOGGER::WARNING << "Safe shutdown sequence failed with unknown error." << std::endl;
    }
    this->arm_safe_shutdown_active.store(false);
    this->safe_shutdown_done = true;
}

std::vector<float> RL_Real_Go2X5::BuildSafeShutdownTargetPose(const std::vector<float>& default_pos) const
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    std::vector<float> target = this->params.Get<std::vector<float>>("shutdown_lie_pose", {});
    if (target.size() != static_cast<size_t>(num_dofs))
    {
        target = default_pos;
    }
    if (target.size() != static_cast<size_t>(num_dofs))
    {
        target.assign(static_cast<size_t>(num_dofs), 0.0f);
    }

    // Safe fallback pose: go2_x5 lying posture for legs.
    static const std::vector<float> kLegLyingPose = {
        0.00f, 1.36f, -2.65f,
        0.00f, 1.36f, -2.65f,
        0.00f, 1.36f, -2.65f,
        0.00f, 1.36f, -2.65f
    };
    const int leg_dofs = std::min(12, num_dofs);
    for (int i = 0; i < leg_dofs && i < static_cast<int>(kLegLyingPose.size()); ++i)
    {
        target[static_cast<size_t>(i)] = kLegLyingPose[static_cast<size_t>(i)];
    }

    const int arm_size = std::max(0, std::min(this->arm_joint_count, num_dofs));
    const int arm_start = std::max(0, std::min(this->arm_joint_start_index, num_dofs));
    if (arm_size > 0 && arm_start + arm_size <= num_dofs)
    {
        std::vector<float> arm_target = this->params.Get<std::vector<float>>("arm_shutdown_pose", {});
        if (arm_target.size() != static_cast<size_t>(arm_size))
        {
            arm_target = this->params.Get<std::vector<float>>("arm_hold_pose", {});
        }
        if (arm_target.size() != static_cast<size_t>(arm_size) &&
            default_pos.size() >= static_cast<size_t>(arm_start + arm_size))
        {
            arm_target.assign(
                default_pos.begin() + static_cast<long>(arm_start),
                default_pos.begin() + static_cast<long>(arm_start + arm_size));
        }
        if (arm_target.size() == static_cast<size_t>(arm_size))
        {
            for (int i = 0; i < arm_size; ++i)
            {
                target[static_cast<size_t>(arm_start + i)] = arm_target[static_cast<size_t>(i)];
            }
        }
    }
    return target;
}

void RL_Real_Go2X5::PublishWholeBodyPose(const std::vector<float>& pose,
                                         const std::vector<float>& kp,
                                         const std::vector<float>& kd)
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    if (pose.size() < static_cast<size_t>(num_dofs) || joint_mapping.size() < static_cast<size_t>(num_dofs))
    {
        return;
    }

    RobotCommand<float> command_local;
    command_local.motor_command.resize(static_cast<size_t>(num_dofs));
    for (int i = 0; i < num_dofs; ++i)
    {
        command_local.motor_command.q[static_cast<size_t>(i)] = pose[static_cast<size_t>(i)];
        command_local.motor_command.dq[static_cast<size_t>(i)] = 0.0f;
        command_local.motor_command.tau[static_cast<size_t>(i)] = 0.0f;
        command_local.motor_command.kp[static_cast<size_t>(i)] =
            (i < static_cast<int>(kp.size())) ? kp[static_cast<size_t>(i)] : 40.0f;
        command_local.motor_command.kd[static_cast<size_t>(i)] =
            (i < static_cast<int>(kd.size())) ? kd[static_cast<size_t>(i)] : 3.0f;
    }

    this->ClipWholeBodyCommand(&command_local, "Whole-body pose publish");

    for (int i = 0; i < num_dofs; ++i)
    {
        const int mapped = joint_mapping[static_cast<size_t>(i)];
        if (mapped < 0 || mapped >= 20)
        {
            continue;
        }
        if (this->arm_split_control_enabled && this->IsArmJointIndex(i))
        {
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x00;
            this->unitree_low_command.motor_cmd()[mapped].q() = PosStopF;
            this->unitree_low_command.motor_cmd()[mapped].dq() = VelStopF;
            this->unitree_low_command.motor_cmd()[mapped].kp() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].kd() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
        }
        else
        {
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x01;
            this->unitree_low_command.motor_cmd()[mapped].q() = command_local.motor_command.q[static_cast<size_t>(i)];
            this->unitree_low_command.motor_cmd()[mapped].dq() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].kp() = command_local.motor_command.kp[static_cast<size_t>(i)];
            this->unitree_low_command.motor_cmd()[mapped].kd() = command_local.motor_command.kd[static_cast<size_t>(i)];
            this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
        }
    }

    this->WriteArmCommandToExternal(&command_local);
    this->unitree_low_command.crc() = Crc32Core((uint32_t *)&unitree_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    if (this->lowcmd_publisher)
    {
        this->lowcmd_publisher->Write(this->unitree_low_command);
    }
}

void RL_Real_Go2X5::ExecuteSafeShutdownSequence()
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    if (num_dofs <= 0)
    {
        this->safe_shutdown_done = true;
        return;
    }

    const auto default_pos = this->params.Get<std::vector<float>>("default_dof_pos", {});
    std::vector<float> start_pose = default_pos;
    if (start_pose.size() != static_cast<size_t>(num_dofs))
    {
        start_pose.assign(static_cast<size_t>(num_dofs), 0.0f);
    }
    {
        const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        if (joint_mapping.size() >= static_cast<size_t>(num_dofs))
        {
            for (int i = 0; i < num_dofs; ++i)
            {
                const int mapped = joint_mapping[static_cast<size_t>(i)];
                if (mapped >= 0 && mapped < static_cast<int>(this->unitree_motor_q.size()))
                {
                    start_pose[static_cast<size_t>(i)] = this->unitree_motor_q[static_cast<size_t>(mapped)];
                }
            }
        }
    }

    const std::vector<float> target_pose = this->BuildSafeShutdownTargetPose(default_pos);
    if (target_pose.size() != static_cast<size_t>(num_dofs))
    {
        this->safe_shutdown_done = true;
        return;
    }

    auto kp = this->params.Get<std::vector<float>>("fixed_kp", {});
    auto kd = this->params.Get<std::vector<float>>("fixed_kd", {});
    if (kp.size() < static_cast<size_t>(num_dofs))
    {
        kp.resize(static_cast<size_t>(num_dofs), 40.0f);
    }
    if (kd.size() < static_cast<size_t>(num_dofs))
    {
        kd.resize(static_cast<size_t>(num_dofs), 3.0f);
    }

    const float soft_land_sec = std::max(0.2f, this->params.Get<float>("shutdown_soft_land_sec", 2.0f));
    const float hold_sec = std::max(0.0f, this->params.Get<float>("shutdown_hold_sec", 0.6f));
    const int step_ms = 10;
    const int land_steps = std::max(1, static_cast<int>(std::lround((soft_land_sec * 1000.0f) / static_cast<float>(step_ms))));
    const int hold_steps = std::max(1, static_cast<int>(std::lround((hold_sec * 1000.0f) / static_cast<float>(step_ms))));

    std::cout << LOGGER::INFO
              << "Safe shutdown: soft land + arm retract (land=" << soft_land_sec
              << "s, hold=" << hold_sec << "s)" << std::endl;

    for (int step = 1; step <= land_steps; ++step)
    {
        const float alpha = static_cast<float>(step) / static_cast<float>(land_steps);
        std::vector<float> pose(static_cast<size_t>(num_dofs), 0.0f);
        for (int i = 0; i < num_dofs; ++i)
        {
            const float q0 = start_pose[static_cast<size_t>(i)];
            const float q1 = target_pose[static_cast<size_t>(i)];
            pose[static_cast<size_t>(i)] = (1.0f - alpha) * q0 + alpha * q1;
        }
        this->PublishWholeBodyPose(pose, kp, kd);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }

    for (int i = 0; i < hold_steps; ++i)
    {
        this->PublishWholeBodyPose(target_pose, kp, kd);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        const int arm_size = std::max(0, std::min(this->arm_joint_count, num_dofs));
        if (arm_size > 0 &&
            this->arm_joint_start_index >= 0 &&
            (this->arm_joint_start_index + arm_size) <= num_dofs)
        {
            this->arm_hold_position.assign(
                target_pose.begin() + static_cast<long>(this->arm_joint_start_index),
                target_pose.begin() + static_cast<long>(this->arm_joint_start_index + arm_size));
            this->arm_joint_command_latest = this->arm_hold_position;
            this->arm_topic_command_latest = this->arm_hold_position;
            this->arm_topic_command_received = true;
            this->arm_hold_enabled = true;
        }
    }

    this->safe_shutdown_done = true;
}

void RL_Real_Go2X5::InitializeArmCommandState()
{
    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    this->cmd_vel_alpha = this->params.Get<float>("cmd_vel_alpha", this->cmd_vel_alpha);
    this->joystick_deadband = std::max(0.0f, this->params.Get<float>("joystick_deadband", this->joystick_deadband));
    this->arm_command_size = this->params.Get<int>("arm_command_size", 0);
    this->arm_joint_command_topic = this->params.Get<std::string>("arm_joint_command_topic", "/arm_joint_pos_cmd");
    this->arm_hold_enabled = this->params.Get<bool>("arm_hold_enabled", true);

    const float step_dt = this->params.Get<float>("dt") * this->params.Get<int>("decimation");
    const float smooth_time = this->params.Get<float>("arm_command_smoothing_time", 0.2f);
    if (step_dt > 0.0f)
    {
        this->arm_command_smoothing_ticks = std::max(0, static_cast<int>(std::lround(smooth_time / step_dt)));
    }
    else
    {
        this->arm_command_smoothing_ticks = 0;
    }
    this->arm_command_smoothing_counter = 0;

    if (this->arm_command_size <= 0)
    {
        this->arm_joint_command_latest.clear();
        this->arm_topic_command_latest.clear();
        this->arm_topic_command_received = false;
        this->arm_hold_position.clear();
        this->arm_command_smoothing_start.clear();
        this->arm_command_smoothing_target.clear();
        this->arm_command_smoothed.clear();
        this->arm_command_initialized = false;
        return;
    }

    this->arm_hold_position.assign(static_cast<size_t>(this->arm_command_size), 0.0f);
    const auto hold_pose = this->params.Get<std::vector<float>>("arm_hold_pose", {});
    if (hold_pose.size() == static_cast<size_t>(this->arm_command_size))
    {
        this->arm_hold_position = hold_pose;
    }
    else
    {
        const auto default_pos = this->params.Get<std::vector<float>>("default_dof_pos", {});
        const int arm_start = std::max(0, this->arm_joint_start_index);
        if (default_pos.size() >= static_cast<size_t>(arm_start + this->arm_command_size))
        {
            this->arm_hold_position.assign(
                default_pos.begin() + static_cast<long>(arm_start),
                default_pos.begin() + static_cast<long>(arm_start + this->arm_command_size)
            );
        }
        else if (default_pos.size() >= static_cast<size_t>(this->arm_command_size))
        {
            const size_t fallback_arm_start = default_pos.size() - static_cast<size_t>(this->arm_command_size);
            this->arm_hold_position.assign(
                default_pos.begin() + static_cast<long>(fallback_arm_start),
                default_pos.end()
            );
        }
    }

    this->arm_joint_command_latest = this->arm_hold_position;
    this->arm_topic_command_latest = this->arm_hold_position;
    this->arm_topic_command_received = false;
    this->arm_command_smoothing_start = this->arm_hold_position;
    this->arm_command_smoothing_target = this->arm_hold_position;
    this->arm_command_smoothed = this->arm_hold_position;
    this->arm_command_initialized = true;
}

void RL_Real_Go2X5::InitializeRealDeploySafetyConfig()
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    this->real_deploy_exclusive_keyboard_control =
        this->params.Get<bool>("real_deploy_exclusive_keyboard_control", true);
    this->policy_inference_log_enabled =
        this->params.Get<bool>("policy_inference_log_enabled", true);
    this->cmd_vel_input_ignored_warned = false;
    this->last_policy_inference_hz = 0.0f;
    this->last_policy_inference_stamp = std::chrono::steady_clock::time_point{};

    auto lower_limits = this->params.Get<std::vector<float>>("joint_lower_limits", {});
    if (lower_limits.size() != static_cast<size_t>(num_dofs))
    {
        lower_limits = this->GetDefaultWholeBodyLowerLimits();
    }
    if (lower_limits.size() != static_cast<size_t>(num_dofs))
    {
        lower_limits.assign(static_cast<size_t>(num_dofs), -std::numeric_limits<float>::infinity());
    }
    this->whole_body_joint_lower_limits = std::move(lower_limits);

    auto upper_limits = this->params.Get<std::vector<float>>("joint_upper_limits", {});
    if (upper_limits.size() != static_cast<size_t>(num_dofs))
    {
        upper_limits = this->GetDefaultWholeBodyUpperLimits();
    }
    if (upper_limits.size() != static_cast<size_t>(num_dofs))
    {
        upper_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_joint_upper_limits = std::move(upper_limits);

    auto velocity_limits = this->params.Get<std::vector<float>>("joint_velocity_limits", {});
    if (velocity_limits.size() != static_cast<size_t>(num_dofs))
    {
        velocity_limits = this->GetDefaultWholeBodyVelocityLimits();
    }
    if (velocity_limits.size() != static_cast<size_t>(num_dofs))
    {
        velocity_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_velocity_limits = std::move(velocity_limits);

    auto effort_limits = this->params.Get<std::vector<float>>("joint_effort_limits", {});
    if (effort_limits.size() != static_cast<size_t>(num_dofs))
    {
        effort_limits = this->GetDefaultWholeBodyEffortLimits();
    }
    if (effort_limits.size() != static_cast<size_t>(num_dofs))
    {
        effort_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_effort_limits = std::move(effort_limits);

    auto kp_limits = this->params.Get<std::vector<float>>("joint_kp_limits", {});
    if (kp_limits.size() != static_cast<size_t>(num_dofs))
    {
        kp_limits = this->GetDefaultWholeBodyKpLimits();
    }
    if (kp_limits.size() != static_cast<size_t>(num_dofs))
    {
        kp_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_kp_limits = std::move(kp_limits);

    auto kd_limits = this->params.Get<std::vector<float>>("joint_kd_limits", {});
    if (kd_limits.size() != static_cast<size_t>(num_dofs))
    {
        kd_limits = this->GetDefaultWholeBodyKdLimits();
    }
    if (kd_limits.size() != static_cast<size_t>(num_dofs))
    {
        kd_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_kd_limits = std::move(kd_limits);
}

void RL_Real_Go2X5::InitializeArmChannelConfig()
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    this->arm_joint_count = this->params.Get<int>("arm_joint_count", this->arm_command_size > 0 ? this->arm_command_size : 6);
    this->arm_joint_start_index = this->params.Get<int>("arm_joint_start_index", std::max(0, num_dofs - this->arm_joint_count));

    if (this->arm_joint_start_index < 0)
    {
        this->arm_joint_start_index = 0;
    }
    if (this->arm_joint_start_index > num_dofs)
    {
        this->arm_joint_start_index = num_dofs;
    }
    if (this->arm_joint_count < 0)
    {
        this->arm_joint_count = 0;
    }
    if (this->arm_joint_start_index + this->arm_joint_count > num_dofs)
    {
        this->arm_joint_count = std::max(0, num_dofs - this->arm_joint_start_index);
    }

    this->arm_control_mode = this->params.Get<std::string>("arm_control_mode", "unitree");
    std::transform(this->arm_control_mode.begin(), this->arm_control_mode.end(), this->arm_control_mode.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    this->arm_split_control_enabled =
        (this->arm_control_mode == "split" || this->arm_control_mode == "arx_x5" || this->arm_control_mode == "bridge");

#if !defined(USE_ARX_X5_SDK)
    if (this->arm_control_mode == "arx_x5")
    {
        std::cout << LOGGER::WARNING
                  << "arm_control_mode=arx_x5 but binary was built without USE_ARX_X5_SDK. "
                  << "Falling back to bridge transport." << std::endl;
    }
#endif

    this->arm_bridge_cmd_topic = this->params.Get<std::string>("arm_bridge_cmd_topic", "/arx_x5/joint_cmd");
    this->arm_bridge_state_topic = this->params.Get<std::string>("arm_bridge_state_topic", "/arx_x5/joint_state");
    this->arm_bridge_require_state = this->params.Get<bool>("arm_bridge_require_state", this->arm_split_control_enabled);
    this->arm_bridge_require_live_state =
        this->params.Get<bool>("arm_bridge_require_live_state", this->arm_bridge_require_state);
    this->arm_bridge_state_timeout_sec = this->params.Get<float>("arm_bridge_state_timeout_sec", 0.25f);
    if (this->arm_bridge_state_timeout_sec < 0.0f)
    {
        this->arm_bridge_state_timeout_sec = 0.0f;
    }
    this->arm_bridge_state_timeout_warned = false;
    this->arm_bridge_shadow_mode_warned = false;

    if (this->arm_external_state_q.size() != static_cast<size_t>(this->arm_joint_count))
    {
        this->arm_external_state_q.assign(static_cast<size_t>(this->arm_joint_count), 0.0f);
        this->arm_external_state_dq.assign(static_cast<size_t>(this->arm_joint_count), 0.0f);
        this->arm_external_state_tau.assign(static_cast<size_t>(this->arm_joint_count), 0.0f);
        this->arm_bridge_state_valid = false;
        this->arm_bridge_state_from_backend = false;
    }
    if (this->arm_external_shadow_q.size() != static_cast<size_t>(this->arm_joint_count))
    {
        this->arm_external_shadow_q.assign(static_cast<size_t>(this->arm_joint_count), 0.0f);
        this->arm_external_shadow_dq.assign(static_cast<size_t>(this->arm_joint_count), 0.0f);
        const int expected_arm_size = std::max(0, this->arm_command_size);
        if (expected_arm_size > 0 && this->arm_hold_position.size() == static_cast<size_t>(expected_arm_size))
        {
            const int count = std::min(this->arm_joint_count, expected_arm_size);
            for (int i = 0; i < count; ++i)
            {
                this->arm_external_shadow_q[static_cast<size_t>(i)] = this->arm_hold_position[static_cast<size_t>(i)];
            }
        }
    }

    auto lower_limits = this->params.Get<std::vector<float>>("arm_joint_lower_limits", {});
    if (lower_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        lower_limits = this->GetDefaultArmLowerLimits();
    }
    if (lower_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        lower_limits.assign(static_cast<size_t>(this->arm_joint_count), -std::numeric_limits<float>::infinity());
    }
    this->arm_joint_lower_limits = std::move(lower_limits);

    auto upper_limits = this->params.Get<std::vector<float>>("arm_joint_upper_limits", {});
    if (upper_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        upper_limits = this->GetDefaultArmUpperLimits();
    }
    if (upper_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        upper_limits.assign(static_cast<size_t>(this->arm_joint_count), std::numeric_limits<float>::infinity());
    }
    this->arm_joint_upper_limits = std::move(upper_limits);

    std::cout << LOGGER::INFO << "Arm control mode: " << this->arm_control_mode
              << " (start=" << this->arm_joint_start_index
              << ", count=" << this->arm_joint_count << ")" << std::endl;
    if (this->arm_split_control_enabled)
    {
        std::cout << LOGGER::INFO << "Arm bridge guard: require_state="
                  << (this->arm_bridge_require_state ? "true" : "false")
                  << ", require_live_state=" << (this->arm_bridge_require_live_state ? "true" : "false")
                  << ", timeout=" << this->arm_bridge_state_timeout_sec << "s" << std::endl;
    }
}

bool RL_Real_Go2X5::UseExclusiveRealDeployControl() const
{
    return this->real_deploy_exclusive_keyboard_control;
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyLowerLimits() const
{
    if (this->params.Get<int>("num_of_dofs") == 18)
    {
        return {
            -1.0472f, -1.5708f, -2.7227f,
            -1.0472f, -1.5708f, -2.7227f,
            -1.0472f, -0.5236f, -2.7227f,
            -1.0472f, -0.5236f, -2.7227f,
            -3.14f, -0.05f, -0.1f,
            -1.6f, -1.57f, -2.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, this->params.Get<int>("num_of_dofs"))),
                              -std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyUpperLimits() const
{
    if (this->params.Get<int>("num_of_dofs") == 18)
    {
        return {
            1.0472f, 3.4907f, -0.83776f,
            1.0472f, 3.4907f, -0.83776f,
            1.0472f, 4.5379f, -0.83776f,
            1.0472f, 4.5379f, -0.83776f,
            2.618f, 3.50f, 3.20f,
            1.55f, 1.57f, 2.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, this->params.Get<int>("num_of_dofs"))),
                              std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyVelocityLimits() const
{
    if (this->params.Get<int>("num_of_dofs") == 18)
    {
        return {
            30.1f, 30.1f, 15.7f,
            30.1f, 30.1f, 15.7f,
            30.1f, 30.1f, 15.7f,
            30.1f, 30.1f, 15.7f,
            3.0f, 3.0f, 3.0f,
            3.0f, 3.0f, 3.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, this->params.Get<int>("num_of_dofs"))),
                              std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyEffortLimits() const
{
    auto torque_limits = this->params.Get<std::vector<float>>("torque_limits", {});
    if (torque_limits.size() == static_cast<size_t>(std::max(0, this->params.Get<int>("num_of_dofs"))))
    {
        return torque_limits;
    }
    if (this->params.Get<int>("num_of_dofs") == 18)
    {
        return {
            23.7f, 23.7f, 45.43f,
            23.7f, 23.7f, 45.43f,
            23.7f, 23.7f, 45.43f,
            23.7f, 23.7f, 45.43f,
            15.0f, 15.0f, 15.0f,
            3.0f, 3.0f, 3.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, this->params.Get<int>("num_of_dofs"))),
                              std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyKpLimits() const
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    std::vector<float> limits(static_cast<size_t>(std::max(0, num_dofs)), 0.0f);
    const auto fixed_kp = this->params.Get<std::vector<float>>("fixed_kp", {});
    const auto rl_kp = this->params.Get<std::vector<float>>("rl_kp", {});
    for (int i = 0; i < num_dofs; ++i)
    {
        float value = 0.0f;
        if (i < static_cast<int>(fixed_kp.size()))
        {
            value = std::max(value, std::fabs(fixed_kp[static_cast<size_t>(i)]));
        }
        if (i < static_cast<int>(rl_kp.size()))
        {
            value = std::max(value, std::fabs(rl_kp[static_cast<size_t>(i)]));
        }
        limits[static_cast<size_t>(i)] = std::max(1.0f, value);
    }
    return limits;
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyKdLimits() const
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    std::vector<float> limits(static_cast<size_t>(std::max(0, num_dofs)), 0.0f);
    const auto fixed_kd = this->params.Get<std::vector<float>>("fixed_kd", {});
    const auto rl_kd = this->params.Get<std::vector<float>>("rl_kd", {});
    for (int i = 0; i < num_dofs; ++i)
    {
        float value = 0.0f;
        if (i < static_cast<int>(fixed_kd.size()))
        {
            value = std::max(value, std::fabs(fixed_kd[static_cast<size_t>(i)]));
        }
        if (i < static_cast<int>(rl_kd.size()))
        {
            value = std::max(value, std::fabs(rl_kd[static_cast<size_t>(i)]));
        }
        limits[static_cast<size_t>(i)] = std::max(1.0f, value);
    }
    return limits;
}

bool RL_Real_Go2X5::ClipWholeBodyCommand(RobotCommand<float> *command, const char* context) const
{
    if (!command)
    {
        return false;
    }

    const int num_dofs = this->params.Get<int>("num_of_dofs");
    if (num_dofs <= 0)
    {
        return false;
    }

    const auto default_pos = this->params.Get<std::vector<float>>("default_dof_pos", {});
    const auto fixed_kp = this->params.Get<std::vector<float>>("fixed_kp", {});
    const auto fixed_kd = this->params.Get<std::vector<float>>("fixed_kd", {});
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names", {});

    int clipped_count = 0;
    std::string first_joint;
    for (int i = 0; i < num_dofs; ++i)
    {
        const float q_fallback =
            (i < static_cast<int>(default_pos.size())) ? default_pos[static_cast<size_t>(i)] : 0.0f;
        const float dq_fallback = 0.0f;
        const float kp_fallback =
            (i < static_cast<int>(fixed_kp.size())) ? fixed_kp[static_cast<size_t>(i)] : 0.0f;
        const float kd_fallback =
            (i < static_cast<int>(fixed_kd.size())) ? fixed_kd[static_cast<size_t>(i)] : 0.0f;
        const float q_lo =
            (i < static_cast<int>(this->whole_body_joint_lower_limits.size()))
                ? this->whole_body_joint_lower_limits[static_cast<size_t>(i)]
                : -std::numeric_limits<float>::infinity();
        const float q_hi =
            (i < static_cast<int>(this->whole_body_joint_upper_limits.size()))
                ? this->whole_body_joint_upper_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float dq_limit =
            (i < static_cast<int>(this->whole_body_velocity_limits.size()))
                ? this->whole_body_velocity_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float tau_limit =
            (i < static_cast<int>(this->whole_body_effort_limits.size()))
                ? this->whole_body_effort_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float kp_limit =
            (i < static_cast<int>(this->whole_body_kp_limits.size()))
                ? this->whole_body_kp_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float kd_limit =
            (i < static_cast<int>(this->whole_body_kd_limits.size()))
                ? this->whole_body_kd_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();

        auto clip_position = [&](float value) -> float
        {
            float sanitized = std::isfinite(value) ? value : q_fallback;
            if (std::isfinite(q_lo) && std::isfinite(q_hi))
            {
                const float lo = std::min(q_lo, q_hi);
                const float hi = std::max(q_lo, q_hi);
                sanitized = std::clamp(sanitized, lo, hi);
            }
            return sanitized;
        };
        auto clip_abs = [](float value, float fallback, float abs_limit, bool non_negative) -> float
        {
            float sanitized = std::isfinite(value) ? value : fallback;
            if (std::isfinite(abs_limit))
            {
                const float lo = non_negative ? 0.0f : -std::fabs(abs_limit);
                const float hi = std::fabs(abs_limit);
                sanitized = std::clamp(sanitized, lo, hi);
            }
            return sanitized;
        };

        const float old_q = command->motor_command.q[static_cast<size_t>(i)];
        const float old_dq = command->motor_command.dq[static_cast<size_t>(i)];
        const float old_tau = command->motor_command.tau[static_cast<size_t>(i)];
        const float old_kp = command->motor_command.kp[static_cast<size_t>(i)];
        const float old_kd = command->motor_command.kd[static_cast<size_t>(i)];

        command->motor_command.q[static_cast<size_t>(i)] = clip_position(old_q);
        command->motor_command.dq[static_cast<size_t>(i)] = clip_abs(old_dq, dq_fallback, dq_limit, false);
        command->motor_command.tau[static_cast<size_t>(i)] = clip_abs(old_tau, 0.0f, tau_limit, false);
        command->motor_command.kp[static_cast<size_t>(i)] = clip_abs(old_kp, kp_fallback, kp_limit, true);
        command->motor_command.kd[static_cast<size_t>(i)] = clip_abs(old_kd, kd_fallback, kd_limit, true);

        const bool joint_clipped =
            !std::isfinite(old_q) || !std::isfinite(old_dq) || !std::isfinite(old_tau) ||
            !std::isfinite(old_kp) || !std::isfinite(old_kd) ||
            std::fabs(command->motor_command.q[static_cast<size_t>(i)] - old_q) > 1e-6f ||
            std::fabs(command->motor_command.dq[static_cast<size_t>(i)] - old_dq) > 1e-6f ||
            std::fabs(command->motor_command.tau[static_cast<size_t>(i)] - old_tau) > 1e-6f ||
            std::fabs(command->motor_command.kp[static_cast<size_t>(i)] - old_kp) > 1e-6f ||
            std::fabs(command->motor_command.kd[static_cast<size_t>(i)] - old_kd) > 1e-6f;
        if (joint_clipped)
        {
            ++clipped_count;
            if (first_joint.empty())
            {
                first_joint = (i < static_cast<int>(joint_names.size())) ? joint_names[static_cast<size_t>(i)]
                                                                         : std::to_string(i);
            }
        }
    }

    if (clipped_count > 0)
    {
        std::cout << LOGGER::WARNING << context << " clipped " << clipped_count
                  << " joint command(s), first=" << first_joint << std::endl;
        return true;
    }
    return false;
}

bool RL_Real_Go2X5::ClipArmPoseTargetInPlace(
    std::vector<float>& target,
    const std::vector<float>& fallback,
    const char* context) const
{
    if (target.size() != static_cast<size_t>(this->arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: expect "
                  << this->arm_joint_count << " values, got " << target.size() << std::endl;
        return false;
    }

    bool clipped = false;
    for (size_t i = 0; i < target.size(); ++i)
    {
        float value = target[i];
        const float fallback_value =
            (i < fallback.size()) ? fallback[i] : ((i < this->arm_joint_lower_limits.size() &&
                                                    i < this->arm_joint_upper_limits.size())
                                                       ? std::clamp(0.0f,
                                                                    std::min(this->arm_joint_lower_limits[i],
                                                                             this->arm_joint_upper_limits[i]),
                                                                    std::max(this->arm_joint_lower_limits[i],
                                                                             this->arm_joint_upper_limits[i]))
                                                       : 0.0f);
        if (!std::isfinite(value))
        {
            value = fallback_value;
            clipped = true;
        }
        if (i < this->arm_joint_lower_limits.size() && i < this->arm_joint_upper_limits.size())
        {
            const float lo = std::min(this->arm_joint_lower_limits[i], this->arm_joint_upper_limits[i]);
            const float hi = std::max(this->arm_joint_lower_limits[i], this->arm_joint_upper_limits[i]);
            const float clamped = std::clamp(value, lo, hi);
            if (std::fabs(clamped - value) > 1e-6f)
            {
                clipped = true;
            }
            value = clamped;
        }
        target[i] = value;
    }

    if (clipped)
    {
        std::cout << LOGGER::WARNING << context << " clipped to arm joint limits." << std::endl;
    }
    return true;
}

bool RL_Real_Go2X5::ClipArmBridgeCommandInPlace(
    std::vector<float>& q,
    std::vector<float>& dq,
    std::vector<float>& kp,
    std::vector<float>& kd,
    std::vector<float>& tau,
    const std::vector<float>& q_fallback,
    const char* context) const
{
    if (q.size() != static_cast<size_t>(this->arm_joint_count) ||
        dq.size() != static_cast<size_t>(this->arm_joint_count) ||
        kp.size() != static_cast<size_t>(this->arm_joint_count) ||
        kd.size() != static_cast<size_t>(this->arm_joint_count) ||
        tau.size() != static_cast<size_t>(this->arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: arm bridge command size mismatch" << std::endl;
        return false;
    }

    if (!this->ClipArmPoseTargetInPlace(q, q_fallback, context))
    {
        return false;
    }

    bool clipped = false;
    for (int i = 0; i < this->arm_joint_count; ++i)
    {
        const int idx = this->arm_joint_start_index + i;
        const float dq_limit =
            (idx >= 0 && idx < static_cast<int>(this->whole_body_velocity_limits.size()))
                ? this->whole_body_velocity_limits[static_cast<size_t>(idx)]
                : 3.0f;
        const float kp_limit =
            (idx >= 0 && idx < static_cast<int>(this->whole_body_kp_limits.size()))
                ? this->whole_body_kp_limits[static_cast<size_t>(idx)]
                : 100.0f;
        const float kd_limit =
            (idx >= 0 && idx < static_cast<int>(this->whole_body_kd_limits.size()))
                ? this->whole_body_kd_limits[static_cast<size_t>(idx)]
                : 20.0f;
        const float tau_limit =
            (idx >= 0 && idx < static_cast<int>(this->whole_body_effort_limits.size()))
                ? this->whole_body_effort_limits[static_cast<size_t>(idx)]
                : 15.0f;

        auto clip_abs = [&](float value, float fallback, float abs_limit, bool non_negative) -> float
        {
            float sanitized = std::isfinite(value) ? value : fallback;
            if (std::isfinite(abs_limit))
            {
                const float lo = non_negative ? 0.0f : -std::fabs(abs_limit);
                const float hi = std::fabs(abs_limit);
                const float clamped = std::clamp(sanitized, lo, hi);
                if (std::fabs(clamped - sanitized) > 1e-6f || !std::isfinite(value))
                {
                    clipped = true;
                }
                return clamped;
            }
            if (!std::isfinite(value))
            {
                clipped = true;
            }
            return sanitized;
        };

        dq[static_cast<size_t>(i)] = clip_abs(dq[static_cast<size_t>(i)], 0.0f, dq_limit, false);
        kp[static_cast<size_t>(i)] = clip_abs(kp[static_cast<size_t>(i)], 0.0f, kp_limit, true);
        kd[static_cast<size_t>(i)] = clip_abs(kd[static_cast<size_t>(i)], 0.0f, kd_limit, true);
        tau[static_cast<size_t>(i)] = clip_abs(tau[static_cast<size_t>(i)], 0.0f, tau_limit, false);
    }

    if (clipped)
    {
        std::cout << LOGGER::WARNING << context << " clipped to real arm deploy limits." << std::endl;
    }
    return true;
}

void RL_Real_Go2X5::ValidateJointMappingOrThrow(const char* stage) const
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    if (num_dofs <= 0)
    {
        throw std::runtime_error(std::string("Invalid num_of_dofs in ") + stage);
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping", {});
    if (joint_mapping.size() != static_cast<size_t>(num_dofs))
    {
        throw std::runtime_error(
            std::string("joint_mapping size mismatch in ") + stage +
            ": expect " + std::to_string(num_dofs) +
            ", got " + std::to_string(joint_mapping.size()));
    }

    std::vector<bool> used(20, false);
    for (size_t i = 0; i < joint_mapping.size(); ++i)
    {
        const int mapped = joint_mapping[i];
        if (mapped < 0 || mapped >= 20)
        {
            throw std::runtime_error(
                std::string("joint_mapping out of range in ") + stage +
                " at index " + std::to_string(i) +
                ", value=" + std::to_string(mapped) + ", expected [0,19]");
        }
        if (used[static_cast<size_t>(mapped)])
        {
            std::cout << LOGGER::WARNING
                      << "Duplicate joint_mapping value " << mapped
                      << " detected in " << stage << "." << std::endl;
        }
        used[static_cast<size_t>(mapped)] = true;
    }

    if (this->arm_joint_start_index < 0 || this->arm_joint_count < 0 ||
        (this->arm_joint_start_index + this->arm_joint_count) > num_dofs)
    {
        throw std::runtime_error(
            std::string("Invalid arm index window in ") + stage +
            ": start=" + std::to_string(this->arm_joint_start_index) +
            ", count=" + std::to_string(this->arm_joint_count) +
            ", num_of_dofs=" + std::to_string(num_dofs));
    }
}

bool RL_Real_Go2X5::IsArmBridgeStateFreshLocked() const
{
    if (!this->arm_bridge_state_valid)
    {
        return false;
    }

    if (this->arm_bridge_require_live_state && !this->arm_bridge_state_from_backend)
    {
        return false;
    }

    if (this->arm_bridge_state_timeout_sec <= 1e-6f)
    {
        return true;
    }

    const auto age = std::chrono::duration_cast<std::chrono::duration<float>>(
        std::chrono::steady_clock::now() - this->arm_bridge_state_stamp).count();
    return age <= this->arm_bridge_state_timeout_sec;
}

void RL_Real_Go2X5::SetupArmCommandSubscriber()
{
#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (this->arm_joint_command_topic.empty())
    {
        return;
    }
    if (this->arm_joint_command_topic == this->arm_joint_command_topic_active)
    {
        return;
    }
#if defined(USE_ROS1) && defined(USE_ROS)
    if (!this->ros1_nh)
    {
        return;
    }
    this->arm_joint_command_subscriber = this->ros1_nh->subscribe<std_msgs::Float32MultiArray>(
        this->arm_joint_command_topic, 10, &RL_Real_Go2X5::ArmJointCommandCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    if (!this->ros2_node)
    {
        return;
    }
    this->arm_joint_command_subscriber = this->ros2_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        this->arm_joint_command_topic, rclcpp::SystemDefaultsQoS(),
        [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) { this->ArmJointCommandCallback(msg); });
#endif
    this->arm_joint_command_topic_active = this->arm_joint_command_topic;
    std::cout << LOGGER::INFO << "arm_joint_command_topic subscribed: "
              << this->arm_joint_command_topic_active << std::endl;
#endif
}

void RL_Real_Go2X5::SetupArmBridgeInterface()
{
#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }
    if (this->arm_bridge_cmd_topic.empty() || this->arm_bridge_state_topic.empty())
    {
        std::cout << LOGGER::WARNING << "Arm split control enabled but bridge topics are empty." << std::endl;
        return;
    }
#if defined(USE_ROS1) && defined(USE_ROS)
    if (!this->ros1_nh)
    {
        return;
    }
    this->arm_bridge_cmd_publisher =
        this->ros1_nh->advertise<std_msgs::Float32MultiArray>(this->arm_bridge_cmd_topic, 1);
    this->arm_bridge_state_subscriber =
        this->ros1_nh->subscribe<std_msgs::Float32MultiArray>(
            this->arm_bridge_state_topic, 10, &RL_Real_Go2X5::ArmBridgeStateCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    if (!this->ros2_node)
    {
        return;
    }
    this->arm_bridge_cmd_publisher =
        this->ros2_node->create_publisher<std_msgs::msg::Float32MultiArray>(
            this->arm_bridge_cmd_topic, rclcpp::SystemDefaultsQoS());
    this->arm_bridge_state_subscriber =
        this->ros2_node->create_subscription<std_msgs::msg::Float32MultiArray>(
            this->arm_bridge_state_topic, rclcpp::SystemDefaultsQoS(),
            [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) { this->ArmBridgeStateCallback(msg); });
#endif
    std::cout << LOGGER::INFO << "Arm bridge ready. cmd_topic=" << this->arm_bridge_cmd_topic
              << ", state_topic=" << this->arm_bridge_state_topic << std::endl;
#else
    if (this->arm_split_control_enabled)
    {
        std::cout << LOGGER::WARNING
                  << "Arm split control enabled in non-ROS build. External arm bridge is unavailable." << std::endl;
    }
#endif
}

bool RL_Real_Go2X5::IsArmJointIndex(int idx) const
{
    return idx >= this->arm_joint_start_index &&
           idx < (this->arm_joint_start_index + this->arm_joint_count);
}

bool RL_Real_Go2X5::IsInRLLocomotionState() const
{
    return this->fsm.current_state_ &&
           this->fsm.current_state_->GetStateName().find("RLLocomotion") != std::string::npos;
}

std::vector<float> RL_Real_Go2X5::GetDefaultArmLowerLimits() const
{
    if (this->arm_joint_count == 6)
    {
        return {-3.14f, -0.05f, -0.1f, -1.6f, -1.57f, -2.0f};
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, this->arm_joint_count)),
                              -std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultArmUpperLimits() const
{
    if (this->arm_joint_count == 6)
    {
        return {2.618f, 3.50f, 3.20f, 1.55f, 1.57f, 2.0f};
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, this->arm_joint_count)),
                              std::numeric_limits<float>::infinity());
}

bool RL_Real_Go2X5::ValidateArmPoseTarget(const std::vector<float>& target, const char* context) const
{
    if (target.size() != static_cast<size_t>(this->arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: expect "
                  << this->arm_joint_count << " values, got " << target.size() << std::endl;
        return false;
    }

    for (size_t i = 0; i < target.size(); ++i)
    {
        const float value = target[i];
        if (!std::isfinite(value))
        {
            std::cout << LOGGER::WARNING << context << " rejected: non-finite joint["
                      << i << "]=" << value << std::endl;
            return false;
        }
        if (i < this->arm_joint_lower_limits.size() && value < this->arm_joint_lower_limits[i])
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << value << " below lower limit "
                      << this->arm_joint_lower_limits[i] << std::endl;
            return false;
        }
        if (i < this->arm_joint_upper_limits.size() && value > this->arm_joint_upper_limits[i])
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << value << " above upper limit "
                      << this->arm_joint_upper_limits[i] << std::endl;
            return false;
        }
    }
    return true;
}

bool RL_Real_Go2X5::ValidateArmBridgeStateSample(const std::vector<float>& q,
                                                 const std::vector<float>& dq,
                                                 const std::vector<float>& tau,
                                                 const char* context) const
{
    if (q.size() != static_cast<size_t>(this->arm_joint_count) ||
        dq.size() != static_cast<size_t>(this->arm_joint_count) ||
        tau.size() != static_cast<size_t>(this->arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: state size mismatch" << std::endl;
        return false;
    }

    constexpr float kStateMargin = 0.35f;
    for (size_t i = 0; i < q.size(); ++i)
    {
        if (!std::isfinite(q[i]) || !std::isfinite(dq[i]) || !std::isfinite(tau[i]))
        {
            std::cout << LOGGER::WARNING << context << " rejected: non-finite state sample at joint "
                      << i << std::endl;
            return false;
        }
        if (i < this->arm_joint_lower_limits.size() &&
            q[i] < (this->arm_joint_lower_limits[i] - kStateMargin))
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << q[i] << " below plausible state lower bound "
                      << (this->arm_joint_lower_limits[i] - kStateMargin) << std::endl;
            return false;
        }
        if (i < this->arm_joint_upper_limits.size() &&
            q[i] > (this->arm_joint_upper_limits[i] + kStateMargin))
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << q[i] << " above plausible state upper bound "
                      << (this->arm_joint_upper_limits[i] + kStateMargin) << std::endl;
            return false;
        }
    }
    return true;
}

void RL_Real_Go2X5::ReadArmStateFromExternal(RobotState<float> *state)
{
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
    const bool bridge_state_fresh = this->IsArmBridgeStateFreshLocked();
    const bool use_bridge_state = bridge_state_fresh &&
                                  this->arm_external_state_q.size() >= static_cast<size_t>(this->arm_joint_count);
    const bool shadow_only_state =
        this->arm_bridge_require_live_state && this->arm_bridge_state_valid && !this->arm_bridge_state_from_backend;

    if (this->arm_split_control_enabled && this->arm_bridge_require_state)
    {
        if (shadow_only_state)
        {
            if (!this->arm_bridge_shadow_mode_warned)
            {
                this->arm_bridge_shadow_mode_warned = true;
                std::cout << LOGGER::WARNING
                          << "Arm bridge topic is alive but reports shadow-only state. "
                          << "Treating arm feedback as invalid until a real backend state arrives."
                          << std::endl;
            }
        }
        else if (!bridge_state_fresh && !this->arm_bridge_state_timeout_warned)
        {
            this->arm_bridge_state_timeout_warned = true;
            std::cout << LOGGER::WARNING
                      << "Arm bridge state is missing or stale. Using shadow arm state until bridge recovers."
                      << std::endl;
        }
        else if (bridge_state_fresh)
        {
            this->arm_bridge_state_timeout_warned = false;
            this->arm_bridge_shadow_mode_warned = false;
        }
    }

    for (int i = 0; i < this->arm_joint_count; ++i)
    {
        const int idx = this->arm_joint_start_index + i;
        if (idx < 0 || idx >= static_cast<int>(state->motor_state.q.size()))
        {
            continue;
        }

        if (use_bridge_state)
        {
            state->motor_state.q[idx] = this->arm_external_state_q[static_cast<size_t>(i)];
            state->motor_state.dq[idx] = this->arm_external_state_dq[static_cast<size_t>(i)];
            state->motor_state.tau_est[idx] = this->arm_external_state_tau[static_cast<size_t>(i)];
        }
        else if (this->arm_external_shadow_q.size() >= static_cast<size_t>(this->arm_joint_count))
        {
            state->motor_state.q[idx] = this->arm_external_shadow_q[static_cast<size_t>(i)];
            state->motor_state.dq[idx] = this->arm_external_shadow_dq[static_cast<size_t>(i)];
            state->motor_state.tau_est[idx] = 0.0f;
        }
    }
}

void RL_Real_Go2X5::WriteArmCommandToExternal(const RobotCommand<float> *command)
{
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }

    std::vector<float> arm_q(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_dq(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_kp(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_kd(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_tau(static_cast<size_t>(this->arm_joint_count), 0.0f);
    const auto fixed_kp = this->params.Get<std::vector<float>>("fixed_kp", {});
    const auto fixed_kd = this->params.Get<std::vector<float>>("fixed_kd", {});
    for (int i = 0; i < this->arm_joint_count; ++i)
    {
        const int idx = this->arm_joint_start_index + i;
        if (idx < 0 || idx >= static_cast<int>(command->motor_command.q.size()))
        {
            continue;
        }
        arm_q[static_cast<size_t>(i)] = command->motor_command.q[idx];
        arm_dq[static_cast<size_t>(i)] = command->motor_command.dq[idx];
        arm_kp[static_cast<size_t>(i)] = command->motor_command.kp[idx];
        arm_kd[static_cast<size_t>(i)] = command->motor_command.kd[idx];
        arm_tau[static_cast<size_t>(i)] = command->motor_command.tau[idx];
    }

    std::vector<float> arm_hold_local;
    bool arm_hold_enabled_local = false;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_hold_enabled_local = this->arm_hold_enabled;
        arm_hold_local = this->arm_hold_position;
    }
    const bool in_rl_locomotion = this->IsInRLLocomotionState();
    const bool allow_passthrough = in_rl_locomotion || this->arm_safe_shutdown_active.load();
    if (allow_passthrough)
    {
        this->arm_non_rl_guard_warned = false;
    }
    else if (!this->arm_non_rl_guard_warned)
    {
        this->arm_non_rl_guard_warned = true;
        std::cout << LOGGER::WARNING
                  << "Arm passthrough blocked outside RL locomotion. Holding arm position until RL or shutdown sequence."
                  << std::endl;
    }

    const bool force_hold = !allow_passthrough;
    const bool use_hold_command = force_hold || arm_hold_enabled_local;
    if (force_hold && arm_hold_local.size() != static_cast<size_t>(this->arm_joint_count))
    {
        std::cout << LOGGER::WARNING
                  << "Arm passthrough blocked outside RL, but no valid arm hold pose is available."
                  << std::endl;
        return;
    }
    if (use_hold_command && arm_hold_local.size() == static_cast<size_t>(this->arm_joint_count))
    {
        for (int i = 0; i < this->arm_joint_count; ++i)
        {
            const int idx = this->arm_joint_start_index + i;
            arm_q[static_cast<size_t>(i)] = arm_hold_local[static_cast<size_t>(i)];
            arm_dq[static_cast<size_t>(i)] = 0.0f;
            arm_tau[static_cast<size_t>(i)] = 0.0f;
            if (idx >= 0 && idx < static_cast<int>(fixed_kp.size()))
            {
                arm_kp[static_cast<size_t>(i)] = fixed_kp[static_cast<size_t>(idx)];
            }
            if (idx >= 0 && idx < static_cast<int>(fixed_kd.size()))
            {
                arm_kd[static_cast<size_t>(i)] = fixed_kd[static_cast<size_t>(idx)];
            }
        }
    }

    if (!this->ClipArmBridgeCommandInPlace(
            arm_q, arm_dq, arm_kp, arm_kd, arm_tau, arm_hold_local, "Arm bridge command"))
    {
        if (arm_hold_local.size() == static_cast<size_t>(this->arm_joint_count))
        {
            for (int i = 0; i < this->arm_joint_count; ++i)
            {
                const int idx = this->arm_joint_start_index + i;
                arm_q[static_cast<size_t>(i)] = arm_hold_local[static_cast<size_t>(i)];
                arm_dq[static_cast<size_t>(i)] = 0.0f;
                arm_tau[static_cast<size_t>(i)] = 0.0f;
                arm_kp[static_cast<size_t>(i)] =
                    (idx >= 0 && idx < static_cast<int>(fixed_kp.size())) ? fixed_kp[static_cast<size_t>(idx)] : 0.0f;
                arm_kd[static_cast<size_t>(i)] =
                    (idx >= 0 && idx < static_cast<int>(fixed_kd.size())) ? fixed_kd[static_cast<size_t>(idx)] : 0.0f;
            }
            this->ClipArmBridgeCommandInPlace(
                arm_q, arm_dq, arm_kp, arm_kd, arm_tau, arm_hold_local, "Arm hold fallback");
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Arm bridge command rejected and no valid hold fallback is available."
                      << std::endl;
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        this->arm_external_shadow_q = arm_q;
        this->arm_external_shadow_dq = arm_dq;
    }

#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (!this->arm_bridge_cmd_topic.empty())
    {
#if defined(USE_ROS1) && defined(USE_ROS)
        if (this->arm_bridge_cmd_publisher)
        {
            std_msgs::Float32MultiArray msg;
            msg.data.reserve(static_cast<size_t>(this->arm_joint_count) * 5);
            msg.data.insert(msg.data.end(), arm_q.begin(), arm_q.end());
            msg.data.insert(msg.data.end(), arm_dq.begin(), arm_dq.end());
            msg.data.insert(msg.data.end(), arm_kp.begin(), arm_kp.end());
            msg.data.insert(msg.data.end(), arm_kd.begin(), arm_kd.end());
            msg.data.insert(msg.data.end(), arm_tau.begin(), arm_tau.end());
            this->arm_bridge_cmd_publisher.publish(msg);
        }
#elif defined(USE_ROS2) && defined(USE_ROS)
        if (this->arm_bridge_cmd_publisher)
        {
            std_msgs::msg::Float32MultiArray msg;
            msg.data.reserve(static_cast<size_t>(this->arm_joint_count) * 5);
            msg.data.insert(msg.data.end(), arm_q.begin(), arm_q.end());
            msg.data.insert(msg.data.end(), arm_dq.begin(), arm_dq.end());
            msg.data.insert(msg.data.end(), arm_kp.begin(), arm_kp.end());
            msg.data.insert(msg.data.end(), arm_kd.begin(), arm_kd.end());
            msg.data.insert(msg.data.end(), arm_tau.begin(), arm_tau.end());
            this->arm_bridge_cmd_publisher->publish(msg);
        }
#endif
    }
#endif
}

void RL_Real_Go2X5::ApplyArmHold(const std::vector<float>& target, const char* reason)
{
    if (this->arm_command_size <= 0)
    {
        return;
    }

    if (target.size() != static_cast<size_t>(this->arm_command_size))
    {
        std::cout << LOGGER::WARNING << "Ignore arm hold update: target size mismatch" << std::endl;
        return;
    }

    std::vector<float> target_local = target;
    std::vector<float> fallback_local;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        fallback_local = this->arm_hold_position;
    }
    if (!this->ClipArmPoseTargetInPlace(target_local, fallback_local, reason))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    this->arm_hold_position = target_local;
    this->arm_joint_command_latest = target_local;
    this->arm_hold_enabled = true;
    if (!this->arm_command_initialized || this->arm_command_smoothed.size() != target_local.size())
    {
        this->arm_command_smoothing_start = target_local;
        this->arm_command_smoothing_target = target_local;
        this->arm_command_smoothed = target_local;
        this->arm_command_initialized = true;
        this->arm_command_smoothing_counter = this->arm_command_smoothing_ticks;
    }
    else
    {
        this->arm_command_smoothing_start = this->arm_command_smoothed;
        this->arm_command_smoothing_target = target_local;
        this->arm_command_smoothing_counter = 0;
    }

    std::cout << LOGGER::INFO << reason << " (smooth=" << this->params.Get<float>("arm_command_smoothing_time", 0.2f) << "s)" << std::endl;
}

bool RL_Real_Go2X5::ArmCommandDifferent(const std::vector<float>& a, const std::vector<float>& b) const
{
    if (a.size() != b.size())
    {
        return true;
    }
    for (size_t i = 0; i < a.size(); ++i)
    {
        if (std::fabs(a[i] - b[i]) > 1e-5f)
        {
            return true;
        }
    }
    return false;
}

void RL_Real_Go2X5::GetState(RobotState<float> *state)
{
    std::array<float, 4> imu_quat{};
    std::array<float, 3> imu_gyro{};
    std::array<float, 20> motor_q{};
    std::array<float, 20> motor_dq{};
    std::array<float, 20> motor_tau{};
    float joy_lx = 0.0f;
    float joy_ly = 0.0f;
    float joy_rx = 0.0f;
    xKeySwitchUnion joy_bits;
    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        imu_quat = this->unitree_imu_quaternion;
        imu_gyro = this->unitree_imu_gyroscope;
        motor_q = this->unitree_motor_q;
        motor_dq = this->unitree_motor_dq;
        motor_tau = this->unitree_motor_tau;
        joy_lx = this->unitree_joy_lx;
        joy_ly = this->unitree_joy_ly;
        joy_rx = this->unitree_joy_rx;
        joy_bits = this->unitree_joy;
    }

    if (!this->UseExclusiveRealDeployControl())
    {
        if (joy_bits.components.A) this->control.SetGamepad(Input::Gamepad::A);
        if (joy_bits.components.B) this->control.SetGamepad(Input::Gamepad::B);
        if (joy_bits.components.X) this->control.SetGamepad(Input::Gamepad::X);
        if (joy_bits.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
        if (joy_bits.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
        if (joy_bits.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
        if (joy_bits.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
        if (joy_bits.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
        if (joy_bits.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
        if (joy_bits.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
        if (joy_bits.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
        if (joy_bits.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
        if (joy_bits.components.L1 && joy_bits.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
        if (joy_bits.components.L1 && joy_bits.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
        if (joy_bits.components.L1 && joy_bits.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
        if (joy_bits.components.L1 && joy_bits.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
        if (joy_bits.components.L1 && joy_bits.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
        if (joy_bits.components.L1 && joy_bits.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
        if (joy_bits.components.L1 && joy_bits.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
        if (joy_bits.components.L1 && joy_bits.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
        if (joy_bits.components.L1 && joy_bits.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
        if (joy_bits.components.L1 && joy_bits.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
        if (joy_bits.components.R1 && joy_bits.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
        if (joy_bits.components.R1 && joy_bits.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
        if (joy_bits.components.R1 && joy_bits.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
        if (joy_bits.components.R1 && joy_bits.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
        if (joy_bits.components.R1 && joy_bits.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
        if (joy_bits.components.R1 && joy_bits.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
        if (joy_bits.components.R1 && joy_bits.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
        if (joy_bits.components.R1 && joy_bits.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
        if (joy_bits.components.R1 && joy_bits.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
        if (joy_bits.components.R1 && joy_bits.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
        if (joy_bits.components.L1 && joy_bits.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

        const bool fixed_cmd_latched =
            (this->control.current_keyboard == Input::Keyboard::Num1 &&
             this->control.last_keyboard == Input::Keyboard::Num1);
        const bool joystick_active =
            (std::fabs(joy_ly) > this->joystick_deadband) ||
            (std::fabs(joy_lx) > this->joystick_deadband) ||
            (std::fabs(joy_rx) > this->joystick_deadband);
        if (!fixed_cmd_latched || joystick_active)
        {
            this->control.x = joy_ly;
            this->control.y = -joy_lx;
            this->control.yaw = -joy_rx;
        }

        if (this->control.navigation_mode)
        {
            std::lock_guard<std::mutex> cmd_lock(this->cmd_vel_mutex);
            if (this->cmd_vel_has_filtered)
            {
                this->control.x = static_cast<float>(this->cmd_vel_filtered.linear.x);
                this->control.y = static_cast<float>(this->cmd_vel_filtered.linear.y);
                this->control.yaw = static_cast<float>(this->cmd_vel_filtered.angular.z);
            }
        }
    }

    state->imu.quaternion[0] = imu_quat[0]; // w
    state->imu.quaternion[1] = imu_quat[1]; // x
    state->imu.quaternion[2] = imu_quat[2]; // y
    state->imu.quaternion[3] = imu_quat[3]; // z

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = imu_gyro[static_cast<size_t>(i)];
    }
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const int mapped = this->params.Get<std::vector<int>>("joint_mapping")[i];
        if (mapped >= 0 && mapped < static_cast<int>(motor_q.size()))
        {
            state->motor_state.q[i] = motor_q[static_cast<size_t>(mapped)];
            state->motor_state.dq[i] = motor_dq[static_cast<size_t>(mapped)];
            state->motor_state.tau_est[i] = motor_tau[static_cast<size_t>(mapped)];
        }
        else
        {
            state->motor_state.q[i] = 0.0f;
            state->motor_state.dq[i] = 0.0f;
            state->motor_state.tau_est[i] = 0.0f;
        }
    }

    this->ReadArmStateFromExternal(state);
}

void RL_Real_Go2X5::SetCommand(const RobotCommand<float> *command)
{
    RobotCommand<float> command_local = *command;
    this->ClipWholeBodyCommand(&command_local, "Real deploy whole-body command");

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const int mapped = this->params.Get<std::vector<int>>("joint_mapping")[i];
        if (this->arm_split_control_enabled && this->IsArmJointIndex(i))
        {
            // Arm joints are controlled by an external channel in split mode.
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x00;
            this->unitree_low_command.motor_cmd()[mapped].q() = PosStopF;
            this->unitree_low_command.motor_cmd()[mapped].dq() = VelStopF;
            this->unitree_low_command.motor_cmd()[mapped].kp() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].kd() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
        }
        else
        {
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x01;
            this->unitree_low_command.motor_cmd()[mapped].q() = command_local.motor_command.q[i];
            this->unitree_low_command.motor_cmd()[mapped].dq() = command_local.motor_command.dq[i];
            this->unitree_low_command.motor_cmd()[mapped].kp() = command_local.motor_command.kp[i];
            this->unitree_low_command.motor_cmd()[mapped].kd() = command_local.motor_command.kd[i];
            this->unitree_low_command.motor_cmd()[mapped].tau() = command_local.motor_command.tau[i];
        }
    }

    this->WriteArmCommandToExternal(&command_local);

    this->unitree_low_command.crc() = Crc32Core((uint32_t *)&unitree_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(unitree_low_command);
}

void RL_Real_Go2X5::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);
    this->MaybePublishKey1CmdVel();

    if (this->control.current_keyboard == Input::Keyboard::Num2)
    {
        int arm_command_size_local = 0;
        std::vector<float> topic_pose;
        bool topic_received = false;
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            arm_command_size_local = this->arm_command_size;
            topic_pose = this->arm_topic_command_latest;
            topic_received = this->arm_topic_command_received;
        }

        const bool key2_prefer_topic_command =
            this->params.Get<bool>("key2_prefer_topic_command", true);
        const auto key_pose = this->params.Get<std::vector<float>>("arm_key_pose", {});
        const auto hold_pose = this->params.Get<std::vector<float>>("arm_hold_pose", {});
        const auto selected = Go2X5ControlLogic::SelectKey2ArmPose(
            arm_command_size_local,
            key2_prefer_topic_command,
            topic_received,
            topic_pose,
            key_pose,
            hold_pose);

        if (!selected.pose.empty())
        {
            const char* reason = "Key[2] pressed: arm hold pose";
            switch (selected.source)
            {
                case Go2X5ControlLogic::ArmPoseSource::TopicCommand:
                    reason = "Key[2] pressed: arm topic command hold";
                    break;
                case Go2X5ControlLogic::ArmPoseSource::KeyPose:
                    reason = "Key[2] pressed: arm key pose hold";
                    break;
                case Go2X5ControlLogic::ArmPoseSource::HoldPose:
                    reason = "Key[2] pressed: arm hold pose";
                    break;
                case Go2X5ControlLogic::ArmPoseSource::None:
                    break;
            }
            this->ApplyArmHold(selected.pose, reason);
        }
        else if (arm_command_size_local > 0)
        {
            std::cout << LOGGER::WARNING
                      << "Key[2] pressed: no valid arm target (topic/key/hold) for arm_command_size="
                      << arm_command_size_local << std::endl;
        }
        this->control.current_keyboard = this->control.last_keyboard;
    }

    if (!this->UseExclusiveRealDeployControl() && this->control.current_keyboard == Input::Keyboard::Num3)
    {
        int arm_command_size_local = 0;
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            arm_command_size_local = this->arm_command_size;
        }
        const auto default_pos = this->params.Get<std::vector<float>>("default_dof_pos", {});
        const int arm_start = std::max(0, this->arm_joint_start_index);
        if (arm_command_size_local > 0 &&
            default_pos.size() >= static_cast<size_t>(arm_start + arm_command_size_local))
        {
            std::vector<float> pose(
                default_pos.begin() + static_cast<long>(arm_start),
                default_pos.begin() + static_cast<long>(arm_start + arm_command_size_local)
            );
            this->ApplyArmHold(pose, "Key[3] pressed: arm restore default");
        }
        this->control.current_keyboard = this->control.last_keyboard;
    }

    if (!this->UseExclusiveRealDeployControl() && this->control.current_keyboard == Input::Keyboard::Num4)
    {
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            this->arm_hold_enabled = !this->arm_hold_enabled;
            std::cout << LOGGER::INFO << "Key[4] pressed: arm hold "
                      << (this->arm_hold_enabled ? "ON" : "OFF") << std::endl;
        }
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real_Go2X5::MaybePublishKey1CmdVel()
{
#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (this->UseExclusiveRealDeployControl())
    {
        return;
    }
    if (!this->control.navigation_mode)
    {
        this->key1_navigation_cmd_published = false;
        return;
    }
    if (this->control.current_keyboard != Input::Keyboard::Num1)
    {
        return;
    }
    if (this->key1_navigation_cmd_published)
    {
        return;
    }
    if (!this->params.Get<bool>("key1_publish_cmd_vel_on_navigation", true))
    {
        return;
    }

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist msg;
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist msg;
#endif
    msg.linear.x = this->params.Get<float>("key1_navigation_cmd_x", 0.5f);
    msg.linear.y = this->params.Get<float>("key1_navigation_cmd_y", 0.0f);
    msg.linear.z = 0.0f;
    msg.angular.x = 0.0f;
    msg.angular.y = 0.0f;
    msg.angular.z = this->params.Get<float>("key1_navigation_cmd_yaw", 0.0f);

    {
        std::lock_guard<std::mutex> lock(this->cmd_vel_mutex);
        this->cmd_vel = msg;
        this->cmd_vel_filtered = msg;
        this->cmd_vel_has_filtered = true;
    }
    this->control.x = static_cast<float>(msg.linear.x);
    this->control.y = static_cast<float>(msg.linear.y);
    this->control.yaw = static_cast<float>(msg.angular.z);
    this->key1_navigation_cmd_published = true;

#if defined(USE_ROS1) && defined(USE_ROS)
    if (this->cmd_vel_publisher)
    {
        this->cmd_vel_publisher.publish(msg);
    }
#elif defined(USE_ROS2) && defined(USE_ROS)
    if (this->cmd_vel_publisher)
    {
        this->cmd_vel_publisher->publish(msg);
    }
#endif

    std::cout << std::endl << LOGGER::INFO
              << "Key[1] pressed: publish /cmd_vel x=" << msg.linear.x
              << " y=" << msg.linear.y
              << " yaw=" << msg.angular.z << std::endl;
#endif
}

void RL_Real_Go2X5::RecordPolicyInferenceTick()
{
    const auto now = std::chrono::steady_clock::now();
    if (this->last_policy_inference_stamp.time_since_epoch().count() > 0)
    {
        const double dt_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - this->last_policy_inference_stamp).count();
        if (dt_sec > 1e-6)
        {
            this->last_policy_inference_hz = static_cast<float>(1.0 / dt_sec);
        }
    }
    this->last_policy_inference_stamp = now;

    if (this->policy_inference_log_enabled && this->last_policy_inference_hz > 0.0f)
    {
        std::cout << "\r\033[K" << LOGGER::INFO << "Policy inference frequency: "
                  << std::fixed << std::setprecision(2) << this->last_policy_inference_hz
                  << " Hz" << std::flush;
    }
}

void RL_Real_Go2X5::HandleLoopException(const std::string& loop_name, const std::string& error)
{
    bool expected = false;
    if (!this->loop_exception_requested.compare_exchange_strong(expected, true))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(this->loop_exception_mutex);
        this->loop_exception_message = loop_name + ": " + error;
    }
    std::cout << LOGGER::ERROR << "Loop exception captured, requesting safe shutdown: "
              << this->loop_exception_message << std::endl;
}

bool RL_Real_Go2X5::LoopExceptionRequested() const
{
    return this->loop_exception_requested.load();
}

void RL_Real_Go2X5::RunModel()
{
    if (this->LoopExceptionRequested())
    {
        return;
    }
    if (!this->rl_init_done)
    {
        this->arm_runtime_params_ready = false;
        return;
    }
    if (!this->arm_runtime_params_ready)
    {
        // Re-sync arm settings after policy config (go2_x5/robot_lab/config.yaml) is loaded.
        this->InitializeArmCommandState();
        this->InitializeArmChannelConfig();
        this->InitializeRealDeploySafetyConfig();
        this->ValidateJointMappingOrThrow("go2_x5/robot_lab/config.yaml");
        this->SetupArmCommandSubscriber();
        this->SetupArmBridgeInterface();
        this->arm_runtime_params_ready = true;
    }

    this->episode_length_buf += 1;
    this->obs.ang_vel = this->robot_state.imu.gyroscope;
    this->obs.commands = {this->control.x, this->control.y, this->control.yaw};

#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (!this->UseExclusiveRealDeployControl() && this->control.navigation_mode)
    {
        std::lock_guard<std::mutex> lock(this->cmd_vel_mutex);
        const auto cmd = this->cmd_vel_has_filtered ? this->cmd_vel_filtered : this->cmd_vel;
#if defined(USE_ROS1) && defined(USE_ROS)
        this->obs.commands = {(float)cmd.linear.x, (float)cmd.linear.y, (float)cmd.angular.z};
#elif defined(USE_ROS2) && defined(USE_ROS)
        this->obs.commands = {(float)cmd.linear.x, (float)cmd.linear.y, (float)cmd.angular.z};
#endif
    }
#endif

    this->obs.base_quat = this->robot_state.imu.quaternion;
    this->obs.dof_pos = this->robot_state.motor_state.q;
    this->obs.dof_vel = this->robot_state.motor_state.dq;

    int arm_command_size_local = 0;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_command_size_local = this->arm_command_size;
    }

    std::vector<float> arm_hold_local;
    bool arm_hold_enabled_local = false;
    if (arm_command_size_local > 0)
    {
        std::vector<float> arm_obs_local;
        std::vector<float> desired_arm_command;
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            desired_arm_command = this->arm_joint_command_latest;
            arm_hold_local = this->arm_hold_position;
            arm_hold_enabled_local = this->arm_hold_enabled;
            if (desired_arm_command.size() != static_cast<size_t>(arm_command_size_local))
            {
                desired_arm_command = arm_hold_local;
            }

            if (!desired_arm_command.empty())
            {
                if (!this->arm_command_initialized || this->arm_command_smoothed.size() != desired_arm_command.size())
                {
                    this->arm_command_smoothed = desired_arm_command;
                    this->arm_command_smoothing_start = desired_arm_command;
                    this->arm_command_smoothing_target = desired_arm_command;
                    this->arm_command_smoothing_counter = this->arm_command_smoothing_ticks;
                    this->arm_command_initialized = true;
                }

                if (this->ArmCommandDifferent(desired_arm_command, this->arm_command_smoothing_target))
                {
                    this->arm_command_smoothing_start = this->arm_command_smoothed;
                    this->arm_command_smoothing_target = desired_arm_command;
                    this->arm_command_smoothing_counter = 0;
                }

                if (this->arm_command_smoothing_ticks <= 1)
                {
                    this->arm_command_smoothed = this->arm_command_smoothing_target;
                }
                else
                {
                    const int ticks = this->arm_command_smoothing_ticks;
                    int c = std::min(this->arm_command_smoothing_counter + 1, ticks);
                    const float alpha = static_cast<float>(c) / static_cast<float>(ticks);
                    for (int i = 0; i < arm_command_size_local; ++i)
                    {
                        const float start = this->arm_command_smoothing_start[static_cast<size_t>(i)];
                        const float target = this->arm_command_smoothing_target[static_cast<size_t>(i)];
                        this->arm_command_smoothed[static_cast<size_t>(i)] = (1.0f - alpha) * start + alpha * target;
                    }
                    if (this->arm_command_smoothing_counter < ticks)
                    {
                        this->arm_command_smoothing_counter += 1;
                    }
                }

                arm_obs_local = this->arm_command_smoothed;
            }
        }
        if (!arm_obs_local.empty())
        {
            this->obs.arm_joint_command = arm_obs_local;
        }
    }

    this->obs.actions = this->Forward();
    this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);
    this->RecordPolicyInferenceTick();

    if (!this->output_dof_pos.empty() && arm_hold_enabled_local && !arm_hold_local.empty())
    {
        const int num_dofs = this->params.Get<int>("num_of_dofs");
        const int arm_start = std::max(0, this->arm_joint_start_index);
        if (arm_command_size_local > 0 &&
            (arm_start + arm_command_size_local) <= num_dofs)
        {
            for (int i = 0; i < arm_command_size_local; ++i)
            {
                const size_t idx = static_cast<size_t>(arm_start + i);
                if (idx < this->output_dof_pos.size() && i < static_cast<int>(arm_hold_local.size()))
                {
                    this->output_dof_pos[idx] = arm_hold_local[static_cast<size_t>(i)];
                }
                if (idx < this->output_dof_vel.size())
                {
                    this->output_dof_vel[idx] = 0.0f;
                }
            }
        }
    }

    if (!this->output_dof_pos.empty() && this->arm_split_control_enabled && this->arm_bridge_require_state)
    {
        bool bridge_state_fresh = false;
        {
            std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
            bridge_state_fresh = this->IsArmBridgeStateFreshLocked();
        }
        if (!bridge_state_fresh && !arm_hold_local.empty() && arm_command_size_local > 0)
        {
            const int num_dofs = this->params.Get<int>("num_of_dofs");
            const int arm_start = std::max(0, this->arm_joint_start_index);
            if ((arm_start + arm_command_size_local) <= num_dofs)
            {
                for (int i = 0; i < arm_command_size_local; ++i)
                {
                    const size_t idx = static_cast<size_t>(arm_start + i);
                    if (idx < this->output_dof_pos.size() && i < static_cast<int>(arm_hold_local.size()))
                    {
                        this->output_dof_pos[idx] = arm_hold_local[static_cast<size_t>(i)];
                    }
                    if (idx < this->output_dof_vel.size())
                    {
                        this->output_dof_vel[idx] = 0.0f;
                    }
                }
            }
        }
    }

    if (!this->output_dof_pos.empty() || !this->output_dof_vel.empty() || !this->output_dof_tau.empty())
    {
        RLCommandOutput output_cmd;
        output_cmd.pos = this->output_dof_pos;
        output_cmd.vel = this->output_dof_vel;
        output_cmd.tau = this->output_dof_tau;
        output_cmd_queue.push(std::move(output_cmd));
    }

    // this->TorqueProtect(this->output_dof_tau);
    // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
    std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
    this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
}

std::vector<float> RL_Real_Go2X5::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    const auto action_scale = this->params.Get<std::vector<float>>("action_scale", {});
    const size_t expected_action_dim =
        !action_scale.empty()
            ? action_scale.size()
            : static_cast<size_t>(std::max(0, this->params.Get<int>("num_of_dofs")));
    if (expected_action_dim > 0 && actions.size() != expected_action_dim)
    {
        std::cout << LOGGER::ERROR
                  << "Policy action dimension mismatch: expect " << expected_action_dim
                  << ", got " << actions.size()
                  << ". Use zero action fallback." << std::endl;
        actions.assign(expected_action_dim, 0.0f);
    }

    bool non_finite_action = false;
    for (float& value : actions)
    {
        if (!std::isfinite(value))
        {
            value = 0.0f;
            non_finite_action = true;
        }
    }
    if (non_finite_action)
    {
        std::cout << LOGGER::WARNING
                  << "Policy produced non-finite action. Replace invalid entries with zero."
                  << std::endl;
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real_Go2X5::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    std::array<float, 20> motor_q_snapshot{};
    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        motor_q_snapshot = this->unitree_motor_q;
    }
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        const int mapped = this->params.Get<std::vector<int>>("joint_mapping")[i];
        const float real_q = (mapped >= 0 && mapped < static_cast<int>(motor_q_snapshot.size()))
            ? motor_q_snapshot[static_cast<size_t>(mapped)]
            : 0.0f;
        this->plot_real_joint_pos[i].push_back(real_q);
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

uint32_t RL_Real_Go2X5::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RL_Real_Go2X5::InitLowCmd()
{
    this->unitree_low_command.head()[0] = 0xFE;
    this->unitree_low_command.head()[1] = 0xEF;
    this->unitree_low_command.level_flag() = 0xFF;
    this->unitree_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = (0x01);
        this->unitree_low_command.motor_cmd()[i].q() = (PosStopF);
        this->unitree_low_command.motor_cmd()[i].kp() = (0);
        this->unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
        this->unitree_low_command.motor_cmd()[i].kd() = (0);
        this->unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

int RL_Real_Go2X5::QueryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = this->msc.CheckMode(robotForm, motionName);
    if (ret == 0)
    {
        std::cout << "CheckMode succeeded." << std::endl;
    }
    else
    {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if (motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = QueryServiceName(robotForm, motionName);
        std::cout << "Service: " << serviceName << " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string RL_Real_Go2X5::QueryServiceName(std::string form, std::string name)
{
    if (form == "0")
    {
        if (name == "normal") return "sport_mode";
        if (name == "ai") return "ai_sport";
        if (name == "advanced") return "advanced_sport";
    }
    else
    {
        if (name == "ai-w") return "wheeled_sport(go2W)";
        if (name == "normal-w") return "wheeled_sport(b2W)";
    }
    return "";
}

void RL_Real_Go2X5::LowStateMessageHandler(const void *message)
{
    const auto *msg = (const unitree_go::msg::dds_::LowState_ *)message;
    if (!msg)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
    for (size_t i = 0; i < this->unitree_imu_quaternion.size(); ++i)
    {
        this->unitree_imu_quaternion[i] = msg->imu_state().quaternion()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < this->unitree_imu_gyroscope.size(); ++i)
    {
        this->unitree_imu_gyroscope[i] = msg->imu_state().gyroscope()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < this->unitree_motor_q.size(); ++i)
    {
        const auto &m = msg->motor_state()[static_cast<int>(i)];
        this->unitree_motor_q[i] = m.q();
        this->unitree_motor_dq[i] = m.dq();
        this->unitree_motor_tau[i] = m.tau_est();
    }
}

void RL_Real_Go2X5::JoystickHandler(const void *message)
{
    const auto *msg = (const unitree_go::msg::dds_::WirelessController_ *)message;
    if (!msg)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
    this->unitree_joy.value = msg->keys();
    this->unitree_joy_lx = msg->lx();
    this->unitree_joy_ly = msg->ly();
    this->unitree_joy_rx = msg->rx();
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real_Go2X5::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    if (this->UseExclusiveRealDeployControl())
    {
        if (!this->cmd_vel_input_ignored_warned)
        {
            this->cmd_vel_input_ignored_warned = true;
            std::cout << LOGGER::WARNING
                      << "Ignore /cmd_vel in exclusive real deploy control mode. Use keyboard 0/1/2 only."
                      << std::endl;
        }
        return;
    }

    std::lock_guard<std::mutex> lock(this->cmd_vel_mutex);
    this->cmd_vel = *msg;
    if (!this->cmd_vel_has_filtered)
    {
        this->cmd_vel_filtered = *msg;
        this->cmd_vel_has_filtered = true;
        return;
    }

    const float a = this->cmd_vel_alpha;
    this->cmd_vel_filtered.linear.x = a * msg->linear.x + (1.0f - a) * this->cmd_vel_filtered.linear.x;
    this->cmd_vel_filtered.linear.y = a * msg->linear.y + (1.0f - a) * this->cmd_vel_filtered.linear.y;
    this->cmd_vel_filtered.linear.z = a * msg->linear.z + (1.0f - a) * this->cmd_vel_filtered.linear.z;
    this->cmd_vel_filtered.angular.x = a * msg->angular.x + (1.0f - a) * this->cmd_vel_filtered.angular.x;
    this->cmd_vel_filtered.angular.y = a * msg->angular.y + (1.0f - a) * this->cmd_vel_filtered.angular.y;
    this->cmd_vel_filtered.angular.z = a * msg->angular.z + (1.0f - a) * this->cmd_vel_filtered.angular.z;
}

void RL_Real_Go2X5::ArmJointCommandCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const std_msgs::Float32MultiArray::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const std_msgs::msg::Float32MultiArray::SharedPtr msg
#endif
)
{
    if (!msg)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    if (this->arm_command_size <= 0)
    {
        return;
    }
    if (msg->data.size() < static_cast<size_t>(this->arm_command_size))
    {
        std::cout << LOGGER::WARNING
                  << "Ignore /arm_joint_pos_cmd: expect " << this->arm_command_size
                  << " values, got " << msg->data.size() << std::endl;
        return;
    }
    std::vector<float> target(static_cast<size_t>(this->arm_command_size), 0.0f);
    const size_t count = static_cast<size_t>(this->arm_command_size);
    for (size_t i = 0; i < count; ++i)
    {
        target[i] = msg->data[i];
    }
    if (!this->ClipArmPoseTargetInPlace(target, this->arm_hold_position, "/arm_joint_pos_cmd"))
    {
        return;
    }

    if (this->arm_joint_command_latest.size() != static_cast<size_t>(this->arm_command_size))
    {
        this->arm_joint_command_latest.assign(static_cast<size_t>(this->arm_command_size), 0.0f);
    }
    if (this->arm_topic_command_latest.size() != static_cast<size_t>(this->arm_command_size))
    {
        this->arm_topic_command_latest.assign(static_cast<size_t>(this->arm_command_size), 0.0f);
    }

    for (size_t i = 0; i < count; ++i)
    {
        this->arm_joint_command_latest[i] = target[i];
        this->arm_topic_command_latest[i] = target[i];
    }
    this->arm_topic_command_received = true;
}

void RL_Real_Go2X5::ArmBridgeStateCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const std_msgs::Float32MultiArray::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const std_msgs::msg::Float32MultiArray::SharedPtr msg
#endif
)
{
    if (!msg || this->arm_joint_count <= 0)
    {
        return;
    }

    const size_t n = static_cast<size_t>(this->arm_joint_count);
    std::vector<float> q(n, 0.0f);
    std::vector<float> dq(n, 0.0f);
    std::vector<float> tau(n, 0.0f);
    bool state_from_backend = false;

    if (msg->data.size() >= 3 * n)
    {
        std::copy_n(msg->data.begin(), n, q.begin());
        std::copy_n(msg->data.begin() + n, n, dq.begin());
        std::copy_n(msg->data.begin() + 2 * n, n, tau.begin());
    }
    else if (msg->data.size() >= n)
    {
        std::copy_n(msg->data.begin(), n, q.begin());
    }
    else
    {
        return;
    }

    if (msg->data.size() == (3 * n + 1) || msg->data.size() == (n + 1))
    {
        state_from_backend = msg->data.back() > 0.5f;
    }

    if (!this->ValidateArmBridgeStateSample(q, dq, tau, "Arm bridge state"))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
    this->arm_external_state_q = std::move(q);
    this->arm_external_state_dq = std::move(dq);
    this->arm_external_state_tau = std::move(tau);
    this->arm_bridge_state_valid = true;
    this->arm_bridge_state_from_backend = state_from_backend;
    this->arm_bridge_state_stamp = std::chrono::steady_clock::now();
    this->arm_bridge_state_timeout_warned = false;
    if (this->arm_bridge_state_from_backend)
    {
        this->arm_bridge_shadow_mode_warned = false;
        if (!this->arm_bridge_state_stream_logged)
        {
            this->arm_bridge_state_stream_logged = true;
            std::cout << LOGGER::INFO << "Arm bridge state stream detected: topic="
                      << this->arm_bridge_state_topic << ", dof=" << this->arm_joint_count
                      << std::endl;
        }
    }
    else if (this->arm_bridge_require_live_state && !this->arm_bridge_shadow_mode_warned)
    {
        this->arm_bridge_shadow_mode_warned = true;
        std::cout << LOGGER::WARNING
                  << "Arm bridge state topic is publishing shadow-only feedback. "
                  << "Real arm feedback is still considered unavailable."
                  << std::endl;
    }
}
#endif

volatile sig_atomic_t g_shutdown_requested_go2_x5 = 0;

void signalHandlerGo2X5(int signum)
{
    (void)signum;
    g_shutdown_requested_go2_x5 = 1;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " networkInterface" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }

    std::cout << LOGGER::INFO << "[Boot] ChannelFactory init begin, iface=" << argv[1] << std::endl;
    ChannelFactory::Instance()->Init(0, argv[1]);
    std::cout << LOGGER::INFO << "[Boot] ChannelFactory init complete" << std::endl;

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::init(argc, argv, "rl_sar_go2_x5", ros::init_options::NoSigintHandler);
    signal(SIGINT, signalHandlerGo2X5);
    RL_Real_Go2X5 rl_sar(argc, argv);
    ros::Rate rate(200.0);
    while (ros::ok() && !g_shutdown_requested_go2_x5 && !rl_sar.LoopExceptionRequested())
    {
        ros::spinOnce();
        rate.sleep();
    }
    rl_sar.SafeShutdownNow();
    if (ros::ok())
    {
        ros::shutdown();
    }
#elif defined(USE_ROS2) && defined(USE_ROS)
    std::cout << LOGGER::INFO << "[Boot] rclcpp init begin" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << LOGGER::INFO << "[Boot] rclcpp init complete" << std::endl;
    signal(SIGINT, signalHandlerGo2X5);
    std::cout << LOGGER::INFO << "[Boot] Constructing RL_Real_Go2X5" << std::endl;
    auto rl_sar = std::make_shared<RL_Real_Go2X5>(argc, argv);
    std::cout << LOGGER::INFO << "[Boot] RL_Real_Go2X5 constructed" << std::endl;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rl_sar->ros2_node);
    while (rclcpp::ok() && !g_shutdown_requested_go2_x5 && !rl_sar->LoopExceptionRequested())
    {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    rl_sar->SafeShutdownNow();
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    signal(SIGINT, signalHandlerGo2X5);
    RL_Real_Go2X5 rl_sar(argc, argv);
    while (!g_shutdown_requested_go2_x5 && !rl_sar.LoopExceptionRequested()) { sleep(1); }
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;
#endif

    return 0;
}
