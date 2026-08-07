#include "kimera_vio_ros/interfaces/multi_robot_loop_closure_bridge.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "kimera_vio_ros/utils/cv_bridge_compat.hpp"

namespace kimera_vio_ros::interfaces {
namespace {

template <typename T>
T declareOrGet(const rclcpp::Node::SharedPtr &node, const std::string &name,
               const T &default_value) {
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

rclcpp::QoS reliableQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
}

rclcpp::QoS reliableTransientQos(size_t depth) {
  return reliableQos(depth).transient_local();
}

gtsam::Matrix factorCovariance(const gtsam::SharedNoiseModel &model) {
  const gtsam::noiseModel::Base *base = model.get();
  if (const auto *robust =
          dynamic_cast<const gtsam::noiseModel::Robust *>(base)) {
    base = robust->noise().get();
  }
  if (const auto *gaussian =
          dynamic_cast<const gtsam::noiseModel::Gaussian *>(base)) {
    return gaussian->covariance();
  }
  return gtsam::Matrix::Identity(6, 6) * 1e-2;
}

void covarianceToMsg(const gtsam::Matrix &covariance,
                     std::array<double, 36> *out) {
  out->fill(0.0);
  if (covariance.rows() != 6 || covariance.cols() != 6) {
    for (size_t i = 0; i < 6; ++i) {
      (*out)[i * 6 + i] = 1e-2;
    }
    return;
  }
  for (size_t row = 0; row < 6; ++row) {
    for (size_t col = 0; col < 6; ++col) {
      (*out)[row * 6 + col] = covariance(row, col);
    }
  }
}

gtsam::Pose3 poseFromMsg(const geometry_msgs::msg::Pose &pose) {
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z),
      gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

bool samePose(const geometry_msgs::msg::Pose &lhs,
              const geometry_msgs::msg::Pose &rhs) {
  return poseFromMsg(lhs).equals(poseFromMsg(rhs), 1e-9);
}

bool sameFactor(const pose_graph_tools_msgs::msg::PoseGraphEdge &lhs,
                const pose_graph_tools_msgs::msg::PoseGraphEdge &rhs) {
  if (!samePose(lhs.pose, rhs.pose) || lhs.has_scale != rhs.has_scale ||
      lhs.scale != rhs.scale || lhs.scale_sigma != rhs.scale_sigma) {
    return false;
  }
  for (size_t i = 0; i < lhs.covariance.size(); ++i) {
    if (std::abs(lhs.covariance[i] - rhs.covariance[i]) > 1e-12) {
      return false;
    }
  }
  return true;
}

} // namespace

MultiRobotLoopClosureBridge::CachedFrame::CachedFrame(
    const VIO::LcdOutput &output)
    : timestamp_ns(output.timestamp_kf_), keypoints_2d(output.keypoints_2d_),
      keypoints_3d(output.keypoints_3d_), versors(output.versors_),
      landmark_ids(output.landmark_ids_), bow_vec(output.bow_vec_),
      descriptors_mat(output.descriptors_mat_.clone()),
      T_base_cam(output.T_base_cam_) {}

MultiRobotLoopClosureBridge::CachedFrame::CachedFrame(
    const VIO::LcdVerificationFrame &frame)
    : timestamp_ns(frame.timestamp), keypoints_2d(frame.keypoints_2d),
      keypoints_3d(frame.keypoints_3d), versors(frame.versors),
      landmark_ids(frame.landmark_ids),
      descriptors_mat(frame.descriptors_mat.clone()),
      T_base_cam(frame.T_base_cam) {}

MultiRobotLoopClosureBridge::MultiRobotLoopClosureBridge(
    const rclcpp::Node::SharedPtr &node)
    : node_(node) {
  enabled_ = declareOrGet(node_, "multi_robot_bridge.enabled", false);
  if (!enabled_) {
    RCLCPP_INFO(node_->get_logger(),
                "Multi-robot loop-closure bridge disabled");
    return;
  }

  const int robot_id = declareOrGet(node_, "robot_id", 0);
  const int descriptor_batch_size =
      declareOrGet(node_, "multi_robot_bridge.descriptor_batch_size", 5);
  const int descriptor_stride =
      declareOrGet(node_, "multi_robot_bridge.descriptor_stride", 1);
  const int verification_batch_size = declareOrGet(
      node_, "multi_robot_bridge.verification_frame_batch_size", 50);
  publish_verification_frames_ = declareOrGet(
      node_, "multi_robot_bridge.publish_verification_frames", true);
  flush_period_s_ =
      declareOrGet(node_, "multi_robot_bridge.flush_period_s", 1.0);
  map_frame_id_ = declareOrGet(node_, "frame_id.map", std::string("map"));

  if (robot_id < 0 || robot_id > std::numeric_limits<uint16_t>::max() ||
      descriptor_batch_size <= 0 || descriptor_stride <= 0 ||
      verification_batch_size <= 0 || flush_period_s_ <= 0.0) {
    throw std::invalid_argument("Invalid multi_robot_bridge parameter");
  }
  robot_id_ = static_cast<uint16_t>(robot_id);
  descriptor_batch_size_ = static_cast<size_t>(descriptor_batch_size);
  descriptor_stride_ = static_cast<size_t>(descriptor_stride);
  verification_frame_batch_size_ = static_cast<size_t>(verification_batch_size);

  pose_graph_pub_ = node_->create_publisher<PoseGraphMsg>("pose_graph/updates",
                                                          reliableQos(1000));
  descriptor_pub_ = node_->create_publisher<BowQueriesMsg>(
      "descriptors/global", reliableTransientQos(100));
  refinement_pub_ = node_->create_publisher<JistRefinementBundlesMsg>(
      "descriptors/jist_refinement", reliableTransientQos(1000));
  frame_pub_ = node_->create_publisher<VLCFramesMsg>("frames/verification",
                                                     reliableQos(100));
  pose_graph_service_ = node_->create_service<PoseGraphQuerySrv>(
      "pose_graph/get",
      std::bind(&MultiRobotLoopClosureBridge::poseGraphService, this,
                std::placeholders::_1, std::placeholders::_2));
  frame_service_ = node_->create_service<VLCFrameQuerySrv>(
      "frames/verification/get",
      std::bind(&MultiRobotLoopClosureBridge::frameService, this,
                std::placeholders::_1, std::placeholders::_2));

  pending_descriptors_.destination_robot_id = robot_id_;
  pending_frames_.destination_robot_id = robot_id_;
  flush_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(flush_period_s_)),
      [this]() { flush(); });

  RCLCPP_INFO(node_->get_logger(),
              "Multi-robot loop-closure bridge enabled for robot %u",
              robot_id_);
}

MultiRobotLoopClosureBridge::~MultiRobotLoopClosureBridge() {
  if (flush_timer_) {
    flush_timer_->cancel();
  }
  flush();
}

void MultiRobotLoopClosureBridge::publishLcdOutput(
    const VIO::LcdOutput::ConstPtr &output) {
  if (!enabled_ || !output) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const VIO::FrameId frame_id = output->keyframe_id_;
  frames_.insert_or_assign(frame_id, CachedFrame(*output));

  if (output->jist_refinement_bundle) {
    const auto &bundle = *output->jist_refinement_bundle;
    if (bundle.sequence_endpoint_id != frame_id ||
        bundle.frame_ids.size() != 5u ||
        bundle.frame_descriptors.rows != 5 ||
        bundle.frame_descriptors.cols != 512 ||
        bundle.frame_descriptors.type() != CV_32FC1) {
      throw std::runtime_error(
          "Malformed JIST refinement bundle in VIO output");
    }
    for (const auto &verification : bundle.verification_frames) {
      if (verification.frame_id == frame_id) {
        continue;
      }
      frames_.insert_or_assign(verification.frame_id,
                               CachedFrame(verification));
    }

    JistRefinementBundlesMsg message;
    message.header.stamp = rclcpp::Time(output->timestamp_kf_);
    message.destination_robot_id = robot_id_;
    auto &bundle_message = message.bundles.emplace_back();
    bundle_message.header = message.header;
    bundle_message.robot_id = robot_id_;
    bundle_message.sequence_pose_id =
        static_cast<uint32_t>(bundle.sequence_endpoint_id);
    bundle_message.descriptor_dim =
        static_cast<uint32_t>(bundle.frame_descriptors.cols);
    bundle_message.frame_ids.reserve(bundle.frame_ids.size());
    for (const auto selected_id : bundle.frame_ids) {
      bundle_message.frame_ids.push_back(static_cast<uint32_t>(selected_id));
    }
    const size_t descriptor_count = bundle.frame_descriptors.total();
    const float *descriptor_data = bundle.frame_descriptors.ptr<float>();
    bundle_message.frame_descriptors.assign(
        descriptor_data, descriptor_data + descriptor_count);
    refinement_pub_->publish(message);
  }
  const CachedFrame &frame = frames_.at(frame_id);
  queueDescriptor(frame_id, frame);
  if (publish_verification_frames_ && !frame.bow_vec.empty()) {
    queueVerificationFrame(frame_id);
  }

  PoseGraphMsg incremental;
  updatePoseGraph(*output, &incremental);
  if (!incremental.edges.empty() || !incremental.nodes.empty()) {
    incremental.header.stamp = rclcpp::Time(output->timestamp_);
    incremental.header.frame_id = map_frame_id_;
    pose_graph_pub_->publish(incremental);
  }
  flushLocked(false);
}

void MultiRobotLoopClosureBridge::updatePoseGraph(const VIO::LcdOutput &output,
                                                  PoseGraphMsg *incremental) {
  std::set<uint64_t> revised_nodes;
  const auto keys = output.states_.keys();
  for (const auto key : keys) {
    if (!output.states_.exists(key)) {
      continue;
    }
    PoseGraphNodeMsg node;
    node.key = key;
    node.robot_id = robot_id_;
    node.header.frame_id = map_frame_id_;
    const auto time_it = output.timestamp_map_.find(key);
    node.header.stamp =
        rclcpp::Time(time_it == output.timestamp_map_.end() ? output.timestamp_
                                                            : time_it->second);
    poseToMsg(output.states_.at<gtsam::Pose3>(key), &node.pose);
    const auto previous = nodes_.find(key);
    if (previous != nodes_.end() && !samePose(previous->second.pose, node.pose)) {
      revised_nodes.insert(key);
    }
    nodes_.insert_or_assign(key, node);
    if (sent_nodes_.insert(key).second) {
      incremental->nodes.push_back(node);
    }
  }

  using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
  for (const auto &factor_ptr : output.nfg_) {
    const auto *factor = dynamic_cast<const PoseBetween *>(factor_ptr.get());
    if (!factor) {
      continue;
    }
    PoseGraphEdgeMsg edge;
    edge.key_from = factor->key1();
    edge.key_to = factor->key2();
    edge.robot_from = robot_id_;
    edge.robot_to = robot_id_;
    edge.type = edge.key_to == edge.key_from + 1 ? PoseGraphEdgeMsg::ODOM
                                                 : PoseGraphEdgeMsg::LOOPCLOSE;
    edge.has_scale = false;
    edge.scale = 1.0;
    edge.scale_sigma = -1.0;
    edge.header.frame_id = map_frame_id_;
    edge.header.stamp = rclcpp::Time(output.timestamp_);
    poseToMsg(factor->measured(), &edge.pose);
    covarianceToMsg(factorCovariance(factor->noiseModel()), &edge.covariance);
    const EdgeId id{edge.key_from, edge.key_to, edge.type};
    const auto previous = edges_.find(id);
    const bool changed =
        previous == edges_.end() || !sameFactor(previous->second, edge) ||
        revised_nodes.count(edge.key_from) || revised_nodes.count(edge.key_to);
    edges_.insert_or_assign(id, edge);
    if (changed) {
      incremental->edges.push_back(edge);
      for (const auto key : {edge.key_from, edge.key_to}) {
        const auto node_it = nodes_.find(key);
        if (node_it != nodes_.end() && sent_nodes_.insert(key).second) {
          incremental->nodes.push_back(node_it->second);
        }
      }
    }
  }
}

void MultiRobotLoopClosureBridge::queueDescriptor(VIO::FrameId frame_id,
                                                  const CachedFrame &frame) {
  if (frame.bow_vec.empty() || frame_id % descriptor_stride_ != 0 ||
      !queued_descriptor_ids_.insert(frame_id).second) {
    return;
  }
  pose_graph_tools_msgs::msg::BowQuery query;
  query.header.stamp = rclcpp::Time(frame.timestamp_ns);
  query.robot_id = robot_id_;
  query.pose_id = static_cast<uint32_t>(frame_id);
  for (const auto &entry : frame.bow_vec) {
    query.bow_vector.word_values.push_back(
        static_cast<float>(entry.second));
  }
  pending_descriptors_.queries.push_back(std::move(query));
}

void MultiRobotLoopClosureBridge::queueVerificationFrame(
    VIO::FrameId frame_id) {
  if (queued_verification_frame_ids_.count(frame_id)) {
    return;
  }
  VLCFrameMsg frame;
  if (getFrameMsg(frame_id, &frame)) {
    queued_verification_frame_ids_.insert(frame_id);
    pending_frames_.frames.push_back(std::move(frame));
  }
}

bool MultiRobotLoopClosureBridge::getFrameMsg(VIO::FrameId frame_id,
                                              VLCFrameMsg *msg) const {
  const auto it = frames_.find(frame_id);
  if (it == frames_.end() || !msg) {
    return false;
  }
  const CachedFrame &frame = it->second;
  if (frame.keypoints_2d.empty() || frame.keypoints_3d.empty() ||
      frame.versors.empty() || frame.descriptors_mat.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Frame %lu has no verification payload",
                static_cast<unsigned long>(frame_id));
    return false;
  }
  if (frame.keypoints_2d.size() != frame.keypoints_3d.size() ||
      frame.keypoints_3d.size() != frame.versors.size() ||
      frame.versors.size() != frame.landmark_ids.size() ||
      frame.descriptors_mat.rows !=
          static_cast<int>(frame.keypoints_2d.size()) ||
      frame.descriptors_mat.type() != CV_32FC1) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Frame %lu has inconsistent verification arrays",
                 static_cast<unsigned long>(frame_id));
    return false;
  }
  msg->robot_id = robot_id_;
  msg->pose_id = static_cast<uint32_t>(frame_id);
  for (size_t index = 0; index < frame.keypoints_2d.size(); ++index) {
    msg->keypoints.push_back(frame.keypoints_2d[index].x);
    msg->keypoints.push_back(frame.keypoints_2d[index].y);
    msg->landmark_ids.push_back(frame.landmark_ids[index]);
    const auto &landmark = frame.keypoints_3d[index];
    msg->depths.push_back(landmark.norm() < 1e-3 ? 0.0f : landmark.z());
  }
  pcl::PointCloud<pcl::PointXYZ> versors;
  for (const auto &versor : frame.versors) {
    versors.emplace_back(versor.x(), versor.y(), versor.z());
  }
  pcl::toROSMsg(versors, msg->versors);

  cv::Mat descriptors_fp16;
  cv::convertFp16(frame.descriptors_mat, descriptors_fp16);
  cv_bridge::CvImage image;
  image.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  image.image = descriptors_fp16;
  image.toImageMsg(msg->descriptors_mat);
  poseToMsg(frame.T_base_cam, &msg->t_base_cam);
  return true;
}

void MultiRobotLoopClosureBridge::flush() {
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  flushLocked(true);
}

void MultiRobotLoopClosureBridge::flushLocked(bool force) {
  if (!pending_descriptors_.queries.empty() &&
      (force ||
       pending_descriptors_.queries.size() >= descriptor_batch_size_)) {
    pending_descriptors_.header.stamp = node_->now();
    descriptor_pub_->publish(pending_descriptors_);
    pending_descriptors_.queries.clear();
  }
  if (!pending_frames_.frames.empty() &&
      (force ||
       pending_frames_.frames.size() >= verification_frame_batch_size_)) {
    pending_frames_.header.stamp = node_->now();
    frame_pub_->publish(pending_frames_);
    pending_frames_.frames.clear();
  }
}

MultiRobotLoopClosureBridge::PoseGraphMsg
MultiRobotLoopClosureBridge::getPoseGraphMsg() const {
  PoseGraphMsg msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = map_frame_id_;
  for (const auto &[key, node] : nodes_) {
    static_cast<void>(key);
    msg.nodes.push_back(node);
  }
  for (const auto &[id, edge] : edges_) {
    static_cast<void>(id);
    msg.edges.push_back(edge);
  }
  return msg;
}

void MultiRobotLoopClosureBridge::poseGraphService(
    const std::shared_ptr<PoseGraphQuerySrv::Request> request,
    std::shared_ptr<PoseGraphQuerySrv::Response> response) {
  if (!request || !response || request->robot_id != robot_id_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  response->pose_graph = getPoseGraphMsg();
}

void MultiRobotLoopClosureBridge::frameService(
    const std::shared_ptr<VLCFrameQuerySrv::Request> request,
    std::shared_ptr<VLCFrameQuerySrv::Response> response) {
  if (!request || !response || request->robot_id != robot_id_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto frame_id : request->pose_ids) {
    VLCFrameMsg frame;
    if (getFrameMsg(frame_id, &frame)) {
      response->frames.push_back(std::move(frame));
    }
  }
}

void MultiRobotLoopClosureBridge::poseToMsg(const gtsam::Pose3 &pose,
                                            geometry_msgs::msg::Pose *msg) {
  msg->position.x = pose.x();
  msg->position.y = pose.y();
  msg->position.z = pose.z();
  const auto quaternion = pose.rotation().toQuaternion();
  msg->orientation.w = quaternion.w();
  msg->orientation.x = quaternion.x();
  msg->orientation.y = quaternion.y();
  msg->orientation.z = quaternion.z();
}

} // namespace kimera_vio_ros::interfaces
