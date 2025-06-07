#include "gflags/gflags.h"
#include "glog/logging.h"
#include "kimera_vio_ros/interfaces/stereo_vio_interface.hpp"

using StereoVio = kimera_vio_ros::interfaces::StereoVioInterface;

int main(int argc, char * argv[])
{
  auto g_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  int g_argc = g_args.size();

  // Convert g_args to char* array for gflags
  std::vector<char*> g_argv;
  for (auto& arg : g_args) {
    g_argv.push_back(const_cast<char*>(arg.c_str()));
  }

  // Initialize Google's flags library with the filtered arguments.
  char** g_argv_ptr = g_argv.data();
  google::ParseCommandLineFlags(&g_argc, &g_argv_ptr, true);

  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  auto node = rclcpp::Node::make_shared("stereo_vio_node");

  rclcpp::executors::MultiThreadedExecutor executor;
  auto stereo_vio_node = std::make_shared<StereoVio>(node);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
