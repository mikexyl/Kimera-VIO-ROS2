/**
 * @file   ros_loop_closure_visualizer.cpp
 * @brief  ROS 2 interface for the loop closure module.
 */

#include "kimera_vio_ros/interfaces/ros_loop_closure_visualizer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>

#include <chrono>
#include <string>

#include "kimera_vio_ros/utils/geometry.hpp"

namespace kimera_vio_ros {
namespace interfaces {

using namespace std::chrono_literals;

namespace {
template <typename T>
T declareOrGetParameter(const rclcpp::Node::SharedPtr& node,
                        const std::string& name,
                        const T& default_value) {
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}
}  // namespace

RosLoopClosureVisualizer::LcdFrame::LcdFrame(
    const VIO::LcdOutput& lcd_output)
    : keypoints_2d(lcd_output.keypoints_2d_),
      keypoints_3d(lcd_output.keypoints_3d_),
      versors(lcd_output.versors_),
      bow_vec(lcd_output.bow_vec_),
      descriptors_mat(lcd_output.descriptors_mat_),
      T_base_cam(lcd_output.T_base_cam_) {
  timestamp_ns = lcd_output.timestamp_kf_;
  scores.resize(3);
  scores[0] = static_cast<float>(lcd_output.similarity_penalty);
  scores[1] = static_cast<float>(lcd_output.coverage_score);
  scores[2] = static_cast<float>(lcd_output.structure_score);

  if (!bow_vec.empty()) {
    CHECK(scores[0] != 0.0f);
    CHECK(scores[1] != 0.0f);
    CHECK(scores[2] != 0.0f);
  }
}

RosLoopClosureVisualizer::RosLoopClosureVisualizer(
    const rclcpp::Node::SharedPtr& node)
    : node_(node),
      robot_id_(0),
      bow_batch_size_(5),
      bow_skip_num_(1),
      publish_vlc_frames_(true) {
  base_link_frame_id_ = declareOrGetParameter(
      node_, "frame_id.base_link", std::string("base_link"));
  map_frame_id_ =
      declareOrGetParameter(node_, "frame_id.map", std::string("map"));
  odom_frame_id_ =
      declareOrGetParameter(node_, "frame_id.odom", std::string("odom"));

  const int robot_id_in = declareOrGetParameter(node_, "robot_id", 0);
  CHECK_GE(robot_id_in, 0);
  robot_id_ = static_cast<uint16_t>(robot_id_in);
  bow_batch_size_ = declareOrGetParameter(node_, "bow_batch_size", 5);
  bow_skip_num_ = declareOrGetParameter(node_, "bow_skip_num", 1);
  publish_vlc_frames_ =
      declareOrGetParameter(node_, "publish_vlc_frames", true);

  trajectory_pub_ =
      node_->create_publisher<nav_msgs::msg::Path>("optimized_trajectory", 1);
  posegraph_pub_ = node_->create_publisher<PoseGraphMsg>("pose_graph", 1);
  posegraph_incremental_pub_ =
      node_->create_publisher<PoseGraphMsg>("pose_graph_incremental", 1000);
  odometry_pub_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("optimized_odometry", 1);
  bow_query_pub_ =
      node_->create_publisher<BowQueriesMsg>("bow_query", 1000);
  vlc_frame_pub_ =
      node_->create_publisher<VLCFramesMsg>("vlc_frames", 100);
  vlc_frame_server_ = node_->create_service<VLCFrameQuerySrv>(
      "vlc_frame_query",
      std::bind(&RosLoopClosureVisualizer::VLCServiceCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  for (uint16_t robot_id = 0; robot_id <= robot_id_; ++robot_id) {
    BowQueriesMsg msg;
    msg.destination_robot_id = robot_id;
    bow_queries_[robot_id] = msg;
  }

  publish_timer_ = node_->create_wall_timer(
      1s, std::bind(&RosLoopClosureVisualizer::publishTimerCallback, this));

  new_frames_msg_.destination_robot_id = robot_id_;
}

void RosLoopClosureVisualizer::publishLcdOutput(
    const VIO::LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);
  CHECK_EQ(lcd_output->keypoints_3d_.size(), lcd_output->versors_.size());
  CHECK_NE(lcd_output->timestamp_, 0);

  std::lock_guard<std::mutex> lock(mutex_);
  frames_.push_back(LcdFrame(*lcd_output));

  processBowQuery();
  if (publish_vlc_frames_) {
    const size_t pose_id = frames_.size() - 1;
    VLCFrameMsg frame_msg;
    if (getFrameMsg(pose_id, &frame_msg)) {
      new_frames_msg_.frames.push_back(frame_msg);
    }
  }

  publishTf(lcd_output);
  if (trajectory_pub_->get_subscription_count() > 0) {
    publishOptimizedTrajectory(lcd_output);
  }
  if (posegraph_pub_->get_subscription_count() > 0 ||
      posegraph_incremental_pub_->get_subscription_count() > 0) {
    publishPoseGraph(lcd_output);
  }
}

void RosLoopClosureVisualizer::publishOptimizedTrajectory(
    const VIO::LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  const VIO::Timestamp& ts = lcd_output->timestamp_;
  const VIO::FrameIDTimestampMap& times = lcd_output->timestamp_map_;
  const gtsam::Values& trajectory = lcd_output->states_;

  nav_msgs::msg::Path path;
  path.poses.reserve(trajectory.size());
  for (size_t i = 0; i < trajectory.size(); i++) {
    const gtsam::Pose3 pose = trajectory.at<gtsam::Pose3>(i);

    geometry_msgs::msg::PoseStamped ps_msg;
    CHECK(times.count(i));
    ps_msg.header.stamp = rclcpp::Time(times.at(i));
    ps_msg.header.frame_id = map_frame_id_;
    poseToMsg(pose, &ps_msg.pose);

    path.poses.push_back(ps_msg);
  }

  path.header.stamp = rclcpp::Time(ts);
  path.header.frame_id = map_frame_id_;
  trajectory_pub_->publish(path);

  const gtsam::Pose3 latest_pose =
      trajectory.at<gtsam::Pose3>(trajectory.size() - 1);
  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.header.stamp = rclcpp::Time(ts);
  odometry_msg.header.frame_id = map_frame_id_;
  odometry_msg.child_frame_id = base_link_frame_id_;
  poseToMsg(latest_pose, &odometry_msg.pose.pose);
  odometry_pub_->publish(odometry_msg);
}

void RosLoopClosureVisualizer::updateRejectedEdges() {
  for (PoseGraphEdgeMsg& loop_closure_edge : loop_closure_edges_) {
    bool is_inlier = false;
    for (PoseGraphEdgeMsg& inlier_edge : inlier_edges_) {
      if (loop_closure_edge.key_from == inlier_edge.key_from &&
          loop_closure_edge.key_to == inlier_edge.key_to) {
        is_inlier = true;
        continue;
      }
    }
    if (!is_inlier) {
      loop_closure_edge.type = PoseGraphEdgeMsg::REJECTED_LOOPCLOSE;
    }
  }

  for (PoseGraphEdgeMsg& inlier_edge : inlier_edges_) {
    bool previously_stored = false;
    for (PoseGraphEdgeMsg& loop_closure_edge : loop_closure_edges_) {
      if (inlier_edge.key_from == loop_closure_edge.key_from &&
          inlier_edge.key_to == loop_closure_edge.key_to) {
        previously_stored = true;
        continue;
      }
    }
    if (!previously_stored) {
      loop_closure_edges_.push_back(inlier_edge);
    }
  }
}

using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;

void RosLoopClosureVisualizer::updateNodesAndEdges(
    const VIO::FrameIDTimestampMap& times,
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  inlier_edges_.clear();
  odometry_edges_.clear();
  for (size_t i = 0; i < nfg.size(); i++) {
    const auto factor = dynamic_cast<const PoseBetween*>(nfg[i].get());
    if (factor) {
      PoseGraphEdgeMsg edge;
      edge.header.frame_id = map_frame_id_;
      edge.key_from = factor->front();
      edge.key_to = factor->back();
      edge.robot_from = robot_id_;
      edge.robot_to = robot_id_;
      if (edge.key_to == edge.key_from + 1) {
        edge.type = PoseGraphEdgeMsg::ODOM;
      } else {
        edge.type = PoseGraphEdgeMsg::LOOPCLOSE;
      }
      poseToMsg(factor->measured(), &edge.pose);

      if (edge.type == PoseGraphEdgeMsg::ODOM) {
        odometry_edges_.push_back(edge);
      } else {
        inlier_edges_.push_back(edge);
      }
    }
  }

  updateRejectedEdges();

  pose_graph_nodes_.clear();
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    PoseGraphNodeMsg node;
    node.key = key_list[i];
    node.robot_id = robot_id_;

    const uint64_t frame_id = gtsam::Symbol(node.key).index();
    CHECK(times.count(frame_id));
    node.header.stamp = rclcpp::Time(times.at(frame_id));
    node.header.frame_id = map_frame_id_;

    const gtsam::Pose3& value = values.at<gtsam::Pose3>(i);
    poseToMsg(value, &node.pose);

    pose_graph_nodes_.push_back(node);
  }
}

RosLoopClosureVisualizer::PoseGraphMsg
RosLoopClosureVisualizer::getPosegraphMsg() const {
  PoseGraphMsg pose_graph;
  pose_graph.edges = odometry_edges_;
  pose_graph.edges.insert(pose_graph.edges.end(),
                          loop_closure_edges_.begin(),
                          loop_closure_edges_.end());
  pose_graph.nodes = pose_graph_nodes_;
  return pose_graph;
}

void RosLoopClosureVisualizer::publishPoseGraph(
    const VIO::LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  const VIO::Timestamp& ts = lcd_output->timestamp_;
  updateNodesAndEdges(
      lcd_output->timestamp_map_, lcd_output->nfg_, lcd_output->states_);
  PoseGraphMsg graph = getPosegraphMsg();
  graph.header.stamp = rclcpp::Time(ts);
  graph.header.frame_id = map_frame_id_;
  posegraph_pub_->publish(graph);

  if (odometry_edges_.size() > 0 && pose_graph_nodes_.size() >= 2) {
    PoseGraphMsg incremental_graph;
    PoseGraphEdgeMsg last_odom_edge = odometry_edges_.back();
    last_odom_edge.header.stamp = rclcpp::Time(ts);
    last_odom_edge.type = PoseGraphEdgeMsg::ODOM;
    last_odom_edge.robot_from = robot_id_;
    last_odom_edge.robot_to = robot_id_;
    incremental_graph.edges.push_back(last_odom_edge);
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 2));
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 1));
    if (lcd_output->lcd_status_ == VIO::LCDStatus::LOOP_DETECTED &&
        !lcd_output->relative_pose_.empty() &&
        !lcd_output->id_match_.empty() &&
        !lcd_output->id_recent_.empty()) {
      PoseGraphEdgeMsg last_lc_edge;
      poseToMsg(lcd_output->relative_pose_[0], &last_lc_edge.pose);
      last_lc_edge.key_from = lcd_output->id_match_[0];
      last_lc_edge.key_to = lcd_output->id_recent_[0];
      last_lc_edge.robot_from = robot_id_;
      last_lc_edge.robot_to = robot_id_;
      last_lc_edge.header.stamp = rclcpp::Time(ts);
      last_lc_edge.type = PoseGraphEdgeMsg::LOOPCLOSE;
      loop_closure_edges_.push_back(last_lc_edge);
      incremental_graph.edges.push_back(last_lc_edge);
    }
    incremental_graph.header.stamp = rclcpp::Time(ts);
    incremental_graph.header.frame_id = map_frame_id_;
    posegraph_incremental_pub_->publish(incremental_graph);
  }
}

void RosLoopClosureVisualizer::publishTf(
    const VIO::LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  geometry_msgs::msg::TransformStamped map_tf;
  map_tf.header.stamp = rclcpp::Time(lcd_output->timestamp_);
  map_tf.header.frame_id = map_frame_id_;
  map_tf.child_frame_id = odom_frame_id_;
  utils::poseToMsgTF(lcd_output->Map_Pose_Odom_, &map_tf.transform);
  tf_broadcaster_->sendTransform(map_tf);
}

void RosLoopClosureVisualizer::processBowQuery() {
  if (frames_.empty()) {
    return;
  }
  const size_t pose_id = frames_.size() - 1;
  if (pose_id % static_cast<size_t>(bow_skip_num_) != 0) {
    return;
  }

  pose_graph_tools_msgs::msg::BowVector bow_vec_msg;
  for (auto it = frames_.back().bow_vec.begin();
       it != frames_.back().bow_vec.end();
       ++it) {
    bow_vec_msg.word_ids.push_back(static_cast<uint32_t>(it->first));
    bow_vec_msg.word_values.push_back(static_cast<float>(it->second));
  }
  pose_graph_tools_msgs::msg::BowQuery bow_msg;
  bow_msg.robot_id = robot_id_;
  bow_msg.pose_id = static_cast<uint32_t>(pose_id);
  bow_msg.bow_vector = bow_vec_msg;
  bow_msg.bow_vector.scores = frames_.back().scores;
  if (!frames_.back().bow_vec.empty()) {
    CHECK(frames_.back().scores.size() == 3);
    CHECK(frames_.back().scores[0] != 0.0f);
    CHECK(frames_.back().scores[1] != 0.0f);
    CHECK(frames_.back().scores[2] != 0.0f);
  }
  bow_msg.header.stamp = rclcpp::Time(frames_.back().timestamp_ns);
  for (uint16_t robot_id = 0; robot_id <= robot_id_; ++robot_id) {
    bow_queries_[robot_id].queries.push_back(bow_msg);
  }
  CHECK_NE(rclcpp::Time(bow_msg.header.stamp).nanoseconds(), 0);
}

void RosLoopClosureVisualizer::publishTimerCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (bow_queries_[robot_id_].queries.size() >=
      static_cast<size_t>(bow_batch_size_)) {
    bow_queries_[robot_id_].header.stamp = node_->now();
    bow_query_pub_->publish(bow_queries_[robot_id_]);
    bow_queries_[robot_id_].queries.clear();
  }

  uint16_t selected_robot_id = 0;
  size_t selected_batch_size = 0;
  for (uint16_t robot_id = 0; robot_id < robot_id_; ++robot_id) {
    if (bow_queries_[robot_id].queries.size() >= selected_batch_size) {
      selected_robot_id = robot_id;
      selected_batch_size = bow_queries_[robot_id].queries.size();
    }
  }

  if (selected_batch_size >= static_cast<size_t>(bow_batch_size_)) {
    bow_queries_[selected_robot_id].header.stamp = node_->now();
    bow_query_pub_->publish(bow_queries_[selected_robot_id]);
    bow_queries_[selected_robot_id].queries.clear();
  }

  if (new_frames_msg_.frames.size() >= 50) {
    vlc_frame_pub_->publish(new_frames_msg_);
    new_frames_msg_.frames.clear();
  }
}

void RosLoopClosureVisualizer::VLCServiceCallback(
    const std::shared_ptr<VLCFrameQuerySrv::Request> request,
    std::shared_ptr<VLCFrameQuerySrv::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  CHECK(request->robot_id == robot_id_);
  response->frames.clear();

  for (const auto& pose_id : request->pose_ids) {
    VLCFrameMsg frame_msg;
    if (!getFrameMsg(pose_id, &frame_msg)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Requested frame %u does not exist!",
                   pose_id);
      continue;
    }
    CHECK(!frame_msg.keypoints.empty());
    response->frames.push_back(frame_msg);
  }
}

bool RosLoopClosureVisualizer::getFrameMsg(size_t pose_id,
                                           VLCFrameMsg* frame_msg) const {
  CHECK_NOTNULL(frame_msg);
  if (pose_id >= frames_.size()) {
    return false;
  }
  const auto& frame = frames_[pose_id];

  frame_msg->robot_id = robot_id_;
  frame_msg->pose_id = static_cast<uint32_t>(pose_id);

  if (!frame.keypoints_2d.empty()) {
    CHECK_EQ(frame.keypoints_3d.size(), frame.versors.size());
    CHECK_EQ(frame.keypoints_2d.size(), frame.keypoints_3d.size());

    for (size_t i = 0; i < frame.keypoints_2d.size(); ++i) {
      frame_msg->keypoints.push_back(frame.keypoints_2d[i].x);
      frame_msg->keypoints.push_back(frame.keypoints_2d[i].y);
    }

    pcl::PointCloud<pcl::PointXYZ> versors;
    for (size_t i = 0; i < frame.keypoints_3d.size(); ++i) {
      const gtsam::Vector3& v = frame.versors[i];
      versors.push_back(pcl::PointXYZ(v(0), v(1), v(2)));

      const gtsam::Vector3& p = frame.keypoints_3d[i];
      if (p.norm() < 1e-3) {
        frame_msg->depths.push_back(0);
      } else {
        frame_msg->depths.push_back(p[2]);
      }
    }
    pcl::toROSMsg(versors, frame_msg->versors);

    cv::Mat descriptors_fp16;
    cv::convertFp16(frame.descriptors_mat, descriptors_fp16);
    cv_bridge::CvImage cv_img;
    cv_img.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    cv_img.image = descriptors_fp16;
    cv_img.toImageMsg(frame_msg->descriptors_mat);
  }

  CHECK(!frame.T_base_cam.equals(gtsam::Pose3::Identity(), 1e-6));
  poseToMsg(frame.T_base_cam, &frame_msg->t_base_cam);

  return true;
}

void RosLoopClosureVisualizer::poseToMsg(const gtsam::Pose3& pose,
                                         geometry_msgs::msg::Pose* msg) {
  CHECK_NOTNULL(msg);
  msg->position.x = pose.x();
  msg->position.y = pose.y();
  msg->position.z = pose.z();
  const gtsam::Quaternion quat = pose.rotation().toQuaternion();
  msg->orientation.w = quat.w();
  msg->orientation.x = quat.x();
  msg->orientation.y = quat.y();
  msg->orientation.z = quat.z();
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
