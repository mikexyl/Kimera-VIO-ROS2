/**
 * @file local_loop_closure_publisher.hpp
 * @brief Local trajectory, pose graph, odometry, and TF publication.
 */

#pragma once

#include <memory>
#include <string>

#include <kimera-vio/loopclosure/LcdOutputPacket.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace kimera_vio_ros::interfaces {

class LocalLoopClosurePublisher {
 public:
  explicit LocalLoopClosurePublisher(const rclcpp::Node::SharedPtr& node);
  void publishLcdOutput(const VIO::LcdOutput::ConstPtr& output);

 private:
  void publishTrajectory(const VIO::LcdOutput& output);
  void publishPoseGraph(const VIO::LcdOutput& output);
  void publishTf(const VIO::LcdOutput& output);
  static void poseToMsg(const gtsam::Pose3& pose,
                        geometry_msgs::msg::Pose* msg);

  rclcpp::Node::SharedPtr node_;
  uint16_t robot_id_{0};
  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<pose_graph_tools_msgs::msg::PoseGraph>::SharedPtr
      pose_graph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace kimera_vio_ros::interfaces
