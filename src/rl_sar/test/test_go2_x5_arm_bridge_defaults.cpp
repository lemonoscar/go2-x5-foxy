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
    const std::string launch_file = source_dir + "/launch/go2_x5_real_dual.launch.py";
    const std::string bridge_file = source_dir + "/scripts/arx_x5_bridge.py";

    const std::string launch_content = ReadAll(launch_file);
    const std::string bridge_content = ReadAll(bridge_file);

    RequireContains(launch_content, "on_exit=Shutdown(reason=\"arx_x5_bridge exited\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_require_sdk\", default_value=\"true\")", launch_file);
    RequireContains(
        launch_content,
        "DeclareLaunchArgument(\"arm_require_initial_state\", default_value=\"true\")",
        launch_file);
    RequireContains(
        launch_content,
        "DeclareLaunchArgument(\"arm_probe_backend_before_init\", default_value=\"true\")",
        launch_file);
    RequireContains(
        launch_content,
        "DeclareLaunchArgument(\"arm_probe_timeout_sec\", default_value=\"5.0\")",
        launch_file);
    RequireContains(
        launch_content,
        "DeclareLaunchArgument(\"arm_accept_commands\", default_value=\"false\")",
        launch_file);

    RequireContains(bridge_content, "self.declare_parameter(\"require_sdk\", False)", bridge_file);
    RequireContains(bridge_content, "self.declare_parameter(\"require_initial_state\", False)", bridge_file);
    RequireContains(bridge_content, "self.declare_parameter(\"probe_backend_before_init\", True)", bridge_file);
    RequireContains(bridge_content, "self.declare_parameter(\"probe_timeout_sec\", 5.0)", bridge_file);
    RequireContains(bridge_content, "self.declare_parameter(\"accept_commands\", False)", bridge_file);
    RequireContains(bridge_content, "self.state_from_backend = False", bridge_file);
    RequireContains(bridge_content, "msg.data = list(self.last_q) + list(self.last_dq) + list(self.last_tau) + [", bridge_file);

    std::cout << "go2_x5 arm bridge defaults test passed." << std::endl;
    return 0;
}
