#include "gflags/gflags.h"
#include "glog/logging.h"
#include "kimera_vio_ros/components/stereo_vio.hpp"

#include <vector>

using StereoVio = kimera_vio_ros::components::StereoVio;

int main(int argc, char * argv[])
{
  auto g_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::vector<char *> g_argv;
  g_argv.reserve(g_args.size());
  for (auto & arg : g_args) {
    g_argv.push_back(arg.data());
  }
  int g_argc = static_cast<int>(g_argv.size());
  char ** g_argv_data = g_argv.data();

  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&g_argc, &g_argv_data, true);

  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto stereo_vio_node = std::make_shared<StereoVio>();
  executor.add_node(stereo_vio_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
