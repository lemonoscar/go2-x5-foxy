#ifndef GO2_X5_CONTROL_LOGIC_HPP
#define GO2_X5_CONTROL_LOGIC_HPP

#include <vector>

namespace Go2X5ControlLogic
{

enum class Key1Mode
{
    FixedCommand,
    Navigation
};

enum class ArmPoseSource
{
    None,
    TopicCommand,
    KeyPose,
    HoldPose
};

struct ArmPoseSelection
{
    std::vector<float> pose;
    ArmPoseSource source = ArmPoseSource::None;
};

inline Key1Mode ResolveKey1Mode(bool prefer_navigation_mode)
{
    return prefer_navigation_mode ? Key1Mode::Navigation : Key1Mode::FixedCommand;
}

inline bool HasEnoughPose(const std::vector<float>& pose, int arm_size)
{
    return arm_size > 0 && pose.size() >= static_cast<size_t>(arm_size);
}

inline std::vector<float> TrimPose(const std::vector<float>& pose, int arm_size)
{
    if (!HasEnoughPose(pose, arm_size))
    {
        return {};
    }
    return std::vector<float>(pose.begin(), pose.begin() + arm_size);
}

inline ArmPoseSelection SelectKey2ArmPose(
    int arm_size,
    bool prefer_topic_command,
    bool topic_command_received,
    const std::vector<float>& topic_command_pose,
    const std::vector<float>& key_pose,
    const std::vector<float>& hold_pose)
{
    ArmPoseSelection selection;
    if (arm_size <= 0)
    {
        return selection;
    }

    if (prefer_topic_command && topic_command_received && HasEnoughPose(topic_command_pose, arm_size))
    {
        selection.pose = TrimPose(topic_command_pose, arm_size);
        selection.source = ArmPoseSource::TopicCommand;
        return selection;
    }

    if (HasEnoughPose(key_pose, arm_size))
    {
        selection.pose = TrimPose(key_pose, arm_size);
        selection.source = ArmPoseSource::KeyPose;
        return selection;
    }

    if (HasEnoughPose(hold_pose, arm_size))
    {
        selection.pose = TrimPose(hold_pose, arm_size);
        selection.source = ArmPoseSource::HoldPose;
        return selection;
    }

    return selection;
}

} // namespace Go2X5ControlLogic

#endif // GO2_X5_CONTROL_LOGIC_HPP
