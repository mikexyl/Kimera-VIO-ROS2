#include "gflags/gflags.h"
#include "glog/logging.h"
#include "kimera_vio_ros/components/stereo_vio.hpp"

using StereoVio = kimera_vio_ros::components::StereoVio;

int main(int argc, char * argv[])
{
  auto g_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  int g_argc = g_args.size();

  // Convert g_args to char* array for gflags
  std::vector<char*> g_argv;
  for (auto& arg : g_args) {
    g_argv.push_back(const_cast<char*>(arg.c_str()));
  }

  // print the command line arguments
  for (int i = 0; i < g_argc; ++i) {
    LOG(INFO) << "Argument " << i << ": " << g_argv[i];
  }

  // Initialize Google's flags library with the filtered arguments.
  char** g_argv_ptr = g_argv.data();
  google::ParseCommandLineFlags(&g_argc, &g_argv_ptr, true);

  LOG(INFO) << "FLAGS_use_lcd: " << FLAGS_use_lcd;
  CHECK(not FLAGS_use_lcd);

  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto stereo_vio_node = std::make_shared<StereoVio>();
  executor.add_node(stereo_vio_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
