#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string ReadAll(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open: " << path << std::endl;
        std::abort();
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

void RequireContains(const std::string& content, const std::string& needle, const std::string& file)
{
    if (content.find(needle) == std::string::npos)
    {
        std::cerr << "Missing expected snippet in " << file << ": " << needle << std::endl;
        std::abort();
    }
}

void RequireNotContains(const std::string& content, const std::string& needle, const std::string& file)
{
    if (content.find(needle) != std::string::npos)
    {
        std::cerr << "Unexpected snippet in " << file << ": " << needle << std::endl;
        std::abort();
    }
}

}  // namespace

int main()
{
    const std::string source_dir = RL_SAR_SOURCE_DIR;
    const std::string real_file = source_dir + "/src/rl_real_go2_x5.cpp";
    const std::string sdk_file = source_dir + "/library/core/rl_sdk/rl_sdk.cpp";
    const std::string bridge_file = source_dir + "/scripts/arx_x5_bridge.py";
    const std::string base_file = source_dir + "/../../policy/go2_x5/base.yaml";
    const std::string config_file = source_dir + "/../../policy/go2_x5/robot_lab/config.yaml";

    const std::string real_content = ReadAll(real_file);
    const std::string sdk_content = ReadAll(sdk_file);
    const std::string bridge_content = ReadAll(bridge_file);
    const std::string base_content = ReadAll(base_file);
    const std::string config_content = ReadAll(config_file);

    RequireContains(real_content, "ClipWholeBodyCommand", real_file);
    RequireContains(real_content, "Arm passthrough blocked outside RL locomotion", real_file);
    RequireContains(real_content, "arm_safe_shutdown_active.store(true)", real_file);
    RequireContains(real_content, "Policy action dimension mismatch", real_file);
    RequireContains(real_content, "Policy inference frequency:", real_file);
    RequireContains(real_content, "HandleLoopException", real_file);
    RequireContains(real_content, "Ignore /cmd_vel in exclusive real deploy control mode", real_file);
    RequireContains(real_content, "arm_bridge_state_from_backend", real_file);
    RequireContains(sdk_content, "real_deploy_exclusive_keyboard_control", sdk_file);
    RequireContains(sdk_content, "Key[1] pressed: RL policy mode ON (exclusive real deploy control)", sdk_file);
    RequireContains(sdk_content, "this->control.navigation_mode = false;", sdk_file);
    RequireContains(bridge_content, "def _clip_command(", bridge_file);
    RequireContains(bridge_content, "Clip arm cmd to deploy limits", bridge_file);
    RequireContains(bridge_content, "self.state_from_backend = state_from_backend", bridge_file);
    RequireContains(bridge_content, "self.declare_parameter(\"joint_pos_min\", [])", bridge_file);
    RequireContains(base_content, "joint_lower_limits", base_file);
    RequireContains(base_content, "joint_upper_limits", base_file);
    RequireContains(base_content, "joint_velocity_limits", base_file);
    RequireContains(base_content, "policy_inference_log_enabled: true", base_file);
    RequireNotContains(base_content, "real_deploy_exclusive_keyboard_control: true", base_file);
    RequireContains(base_content, "arm_bridge_require_live_state: true", base_file);
    RequireContains(config_content, "arm_bridge_require_live_state: true", config_file);
    RequireContains(config_content, "real_deploy_exclusive_keyboard_control: true", config_file);
    RequireContains(config_content, "policy_inference_log_enabled: true", config_file);

    std::cout << "go2_x5 real deploy safety guards test passed." << std::endl;
    return 0;
}
