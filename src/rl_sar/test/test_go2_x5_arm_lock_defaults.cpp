#include <cassert>
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

}  // namespace

int main()
{
    const std::string source_dir = RL_SAR_SOURCE_DIR;
    const std::string config_file = source_dir + "/../../policy/go2_x5/robot_lab/config.yaml";
    const std::string fsm_file = source_dir + "/fsm_robot/fsm_go2_x5.hpp";
    const std::string sdk_file = source_dir + "/library/core/rl_sdk/rl_sdk.cpp";

    const std::string config_content = ReadAll(config_file);
    const std::string fsm_content = ReadAll(fsm_file);
    const std::string sdk_content = ReadAll(sdk_file);

    RequireContains(config_content, "arm_lock: false", config_file);
    RequireContains(fsm_content, "arm_joint_start_index", fsm_file);
    RequireContains(fsm_content, "arm_lock_pose_runtime.assign(", fsm_file);
    RequireContains(sdk_content, "arm_lock_pose_runtime_valid", sdk_file);
    RequireContains(sdk_content, "arm_joint_start_index", sdk_file);

    std::cout << "go2_x5 arm lock defaults test passed." << std::endl;
    return 0;
}
