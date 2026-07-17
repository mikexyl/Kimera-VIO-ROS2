#pragma once

#include "dense_mapping/msg/da3_run.hpp"
#include "dense_mapping/msg/keyframe_state.hpp"
#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pose_graph_tools_msgs/msg/pose_graph.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace kimera_vio_ros {
namespace interfaces {

class DenseMappingPublisher {
 public:
  DenseMappingPublisher(rclcpp::Node::SharedPtr node,
                        const VIO::CameraParams& camera_params);

  void publish(const VIO::BackendOutput::Ptr& output,
               const nav_msgs::msg::Odometry& odometry) const;

 private:
  dense_mapping::msg::KeyframeState makeKeyframeState(
      const VIO::BackendOutput& output,
      const nav_msgs::msg::Odometry& odometry) const;
  pose_graph_tools_msgs::msg::PoseGraph makeLocalWindowState(
      const VIO::BackendOutput& output,
      const nav_msgs::msg::Odometry& odometry) const;
  bool makeDa3Run(const VIO::MonoDepthRawPacket::ConstPtr& packet,
                  dense_mapping::msg::Da3Run* message,
                  std::string* failure_reason) const;

  rclcpp::Node::SharedPtr node_;
  VIO::CameraParams camera_params_;
  std::string camera_frame_id_;
  rclcpp::Publisher<dense_mapping::msg::Da3Run>::SharedPtr da3_run_publisher_;
  rclcpp::Publisher<dense_mapping::msg::KeyframeState>::SharedPtr
      keyframe_publisher_;
  rclcpp::Publisher<pose_graph_tools_msgs::msg::PoseGraph>::SharedPtr
      local_window_publisher_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros
