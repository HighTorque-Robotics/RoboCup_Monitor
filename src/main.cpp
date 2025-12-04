// #include <ros/ros.h> // 修改：移除 ROS 核心头文件
#include <tclap/CmdLine.h>
#include "dmonitor/monitor.hpp"
#include <iostream> // 修改：添加 <iostream> 以便使用 std::cerr
#include <cstddef> // 【新增】为了使用 offsetof

int main(int argc, char **argv) {
  // ros::init(argc, argv, "dmonitor_node"); // 修改：移除 ROS 初始化
  // ros::NodeHandle nh("~"); // 修改：移除 ROS 节点句柄
  
  int camera_device(-1);
  std::string log_root;
  bool quiet_mode = false;

  // Wrap everything in a try block.
  // Do this every time, because exceptions will be thrown for problems.
  // (这部分 TCLAP 代码保持不变，因为它与 ROS 无关)
  try {
    TCLAP::CmdLine cmd("Monitor for RoboCup Humanoid League", ' ', "1.0.0");
    TCLAP::ValueArg<int> cam_arg("c", "camera", "ID of camera device", false,
                                 -1, "int");
    cmd.add(cam_arg);
    TCLAP::ValueArg<std::string> log_arg("l", "log_path",
                                         "Path to log directory for replaying",
                                         false, "", "string");
    cmd.add(log_arg);
    TCLAP::SwitchArg quiet_arg("q", "quiet", "Quiet mode (no logging)", false);
    cmd.add(quiet_arg);

    // Parse the argv array.
    cmd.parse(argc, argv);

    // Get the value parsed by each arg.
    camera_device = cam_arg.getValue();
    log_root = log_arg.getValue();
    quiet_mode = quiet_arg.getValue();
  } catch (TCLAP::ArgException &e)  // catch any exceptions
  {
    std::cerr << "error: " << e.error() << " for arg " << e.argId()
              << std::endl;
  }

  dmonitor::Monitor monitor(argv[0], camera_device, log_root, quiet_mode);
  monitor.Start();
  return 0;
}