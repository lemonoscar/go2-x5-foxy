/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <vector>

#include "go2_x5_control_logic.hpp"

int main()
{
    using namespace Go2X5ControlLogic;

    {
        const auto mode = ResolveKey1Mode(true);
        if (mode != Key1Mode::Navigation)
        {
            std::cerr << "Expected Key1Mode::Navigation when prefer_navigation_mode=true\n";
            return 1;
        }
    }

    {
        const auto mode = ResolveKey1Mode(false);
        if (mode != Key1Mode::FixedCommand)
        {
            std::cerr << "Expected Key1Mode::FixedCommand when prefer_navigation_mode=false\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        const std::vector<float> key = {1, 1, 1, 1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, true, true, topic, key, hold);
        if (selected.source != ArmPoseSource::TopicCommand || selected.pose != topic)
        {
            std::cerr << "Expected key2 to prefer topic arm command\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {0.1f, 0.2f, 0.3f};
        const std::vector<float> key = {1, 1, 1, 1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, true, false, topic, key, hold);
        if (selected.source != ArmPoseSource::KeyPose || selected.pose != key)
        {
            std::cerr << "Expected key pose fallback when topic is unavailable\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {};
        const std::vector<float> key = {1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, true, false, topic, key, hold);
        if (selected.source != ArmPoseSource::HoldPose || selected.pose != hold)
        {
            std::cerr << "Expected hold pose fallback when key pose is too short\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        const std::vector<float> key = {};
        const std::vector<float> hold = {};
        const auto selected = SelectKey2ArmPose(6, false, true, topic, key, hold);
        if (selected.source != ArmPoseSource::None || !selected.pose.empty())
        {
            std::cerr << "Expected no key2 target when topic is disabled and no fallback poses exist\n";
            return 1;
        }
    }

    std::cout << "test_go2_x5_control_logic passed\n";
    return 0;
}
