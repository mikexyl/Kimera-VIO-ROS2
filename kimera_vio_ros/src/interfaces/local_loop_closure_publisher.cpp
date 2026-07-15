#include "kimera_vio_ros/interfaces/local_loop_closure_publisher.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>

#include "kimera_vio_ros/utils/geometry.hpp"

namespace kimera_vio_ros::interfaces {
namespace {

template <typename T>
T declareOrGet(const rclcpp::Node::SharedPtr& node,
               const std::string& name,
               const T& default_value) {
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

gtsam::Matrix factorCovariance(const gtsam::SharedNoiseModel& model) {
  const gtsam::noiseModel::Base* base = model.get();
  if (const auto* robust =
          dynamic_cast<const gtsam::noiseModel::Robust*>(base)) {
    base = robust->noise().get();
  }
  if (const auto* gaussian =
          dynamic_cast<const gtsam::noiseModel::Gaussian*>(base)) {
    return gaussian->covariance();
  }
  return gtsam::Matrix::Identity(6, 6) * 1e-2;
}

}  // namespace

LocalLoopClosurePublisher::LocalLoopClosurePublisher(
    const rclcpp::Node::SharedPtr& node)
    : node_(node) {
  const int robot_id = declareOrGet(node_, "robot_id", 0);
  robot_id_ = static_cast<uint16_t>(std::max(robot_id, 0));
  base_link_frame_id_ =
      declareOrGet(node_, "frame_id.base_link", std::string("base_link"));
  odom_frame_id_ =
      declareOrGet(node_, "frame_id.odom", std::string("odom"));
  map_frame_id_ =
      declareOrGet(node_, "frame_id.map", std::string("map"));
  trajectory_pub_ =
      node_->create_publisher<nav_msgs::msg::Path>("optimized_trajectory", 1);
  pose_graph_pub_ = node_->create_publisher<
      pose_graph_tools_msgs::msg::PoseGraph>(
      "pose_graph", rclcpp::QoS(1).reliable().transient_local());
  odometry_pub_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("optimized_odometry", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void LocalLoopClosurePublisher::publishLcdOutput(
    const VIO::LcdOutput::ConstPtr& output) {
  if (!output) {
    return;
  }
  publishTrajectory(*output);
  publishPoseGraph(*output);
  publishTf(*output);
}

void LocalLoopClosurePublisher::publishTrajectory(
    const VIO::LcdOutput& output) {
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Time(output.timestamp_);
  path.header.frame_id = map_frame_id_;
  for (const auto key : output.states_.keys()) {
    if (!output.states_.exists(key)) {
      continue;
    }
    geometry_msgs::msg::PoseStamped stamped;
    const auto time_it = output.timestamp_map_.find(key);
    stamped.header.stamp = rclcpp::Time(
        time_it == output.timestamp_map_.end() ? output.timestamp_
                                                : time_it->second);
    stamped.header.frame_id = map_frame_id_;
    poseToMsg(output.states_.at<gtsam::Pose3>(key), &stamped.pose);
    path.poses.push_back(std::move(stamped));
  }
  trajectory_pub_->publish(path);
  if (!path.poses.empty()) {
    nav_msgs::msg::Odometry odometry;
    odometry.header = path.header;
    odometry.child_frame_id = base_link_frame_id_;
    odometry.pose.pose = path.poses.back().pose;
    odometry_pub_->publish(odometry);
  }
}

void LocalLoopClosurePublisher::publishPoseGraph(
    const VIO::LcdOutput& output) {
  pose_graph_tools_msgs::msg::PoseGraph graph;
  graph.header.stamp = rclcpp::Time(output.timestamp_);
  graph.header.frame_id = map_frame_id_;
  for (const auto key : output.states_.keys()) {
    if (!output.states_.exists(key)) {
      continue;
    }
    pose_graph_tools_msgs::msg::PoseGraphNode node;
    node.key = key;
    node.robot_id = robot_id_;
    node.header.frame_id = map_frame_id_;
    const auto time_it = output.timestamp_map_.find(key);
    node.header.stamp = rclcpp::Time(
        time_it == output.timestamp_map_.end() ? output.timestamp_
                                                : time_it->second);
    poseToMsg(output.states_.at<gtsam::Pose3>(key), &node.pose);
    graph.nodes.push_back(std::move(node));
  }
  using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
  for (const auto& factor_ptr : output.nfg_) {
    const auto* factor = dynamic_cast<const PoseBetween*>(factor_ptr.get());
    if (!factor) {
      continue;
    }
    pose_graph_tools_msgs::msg::PoseGraphEdge edge;
    edge.header = graph.header;
    edge.key_from = factor->key1();
    edge.key_to = factor->key2();
    edge.robot_from = robot_id_;
    edge.robot_to = robot_id_;
    edge.type = edge.key_to == edge.key_from + 1
                    ? pose_graph_tools_msgs::msg::PoseGraphEdge::ODOM
                    : pose_graph_tools_msgs::msg::PoseGraphEdge::LOOPCLOSE;
    poseToMsg(factor->measured(), &edge.pose);
    const gtsam::Matrix covariance = factorCovariance(factor->noiseModel());
    edge.covariance.fill(0.0);
    if (covariance.rows() == 6 && covariance.cols() == 6) {
      for (size_t row = 0; row < 6; ++row) {
        for (size_t col = 0; col < 6; ++col) {
          edge.covariance[row * 6 + col] = covariance(row, col);
        }
      }
    }
    graph.edges.push_back(std::move(edge));
  }
  pose_graph_pub_->publish(graph);
}

void LocalLoopClosurePublisher::publishTf(const VIO::LcdOutput& output) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = rclcpp::Time(output.timestamp_);
  transform.header.frame_id = map_frame_id_;
  transform.child_frame_id = odom_frame_id_;
  utils::poseToMsgTF(output.Map_Pose_Odom_, &transform.transform);
  tf_broadcaster_->sendTransform(transform);
}

void LocalLoopClosurePublisher::poseToMsg(const gtsam::Pose3& pose,
                                           geometry_msgs::msg::Pose* msg) {
  msg->position.x = pose.x();
  msg->position.y = pose.y();
  msg->position.z = pose.z();
  const auto quaternion = pose.rotation().toQuaternion();
  msg->orientation.w = quaternion.w();
  msg->orientation.x = quaternion.x();
  msg->orientation.y = quaternion.y();
  msg->orientation.z = quaternion.z();
}

}  // namespace kimera_vio_ros::interfaces
