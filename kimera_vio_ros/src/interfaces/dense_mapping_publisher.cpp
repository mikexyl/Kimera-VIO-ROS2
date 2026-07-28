#include "kimera_vio_ros/interfaces/dense_mapping_publisher.hpp"

#include "kimera_vio_ros/utils/cv_bridge_compat.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace kimera_vio_ros {
namespace interfaces {
namespace {

geometry_msgs::msg::Pose poseToMessage(const gtsam::Pose3& pose) {
  geometry_msgs::msg::Pose message;
  message.position.x = pose.x();
  message.position.y = pose.y();
  message.position.z = pose.z();
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  message.orientation.w = quaternion.w();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  return message;
}

bool finitePose(const gtsam::Pose3& pose) {
  return pose.rotation().matrix().allFinite() &&
         pose.translation().allFinite();
}

sensor_msgs::msg::CameraInfo packetCameraInfo(
    const VIO::MonoDepthRawPacket& packet,
    const std_msgs::msg::Header& header) {
  sensor_msgs::msg::CameraInfo message;
  message.header = header;
  message.width = static_cast<std::uint32_t>(packet.intrinsics.width);
  message.height = static_cast<std::uint32_t>(packet.intrinsics.height);
  message.distortion_model = "none";
  message.k = {packet.intrinsics.fx,
               0.0,
               packet.intrinsics.cx,
               0.0,
               packet.intrinsics.fy,
               packet.intrinsics.cy,
               0.0,
               0.0,
               1.0};
  message.p = {packet.intrinsics.fx,
               0.0,
               packet.intrinsics.cx,
               0.0,
               0.0,
               packet.intrinsics.fy,
               packet.intrinsics.cy,
               0.0,
               0.0,
               0.0,
               1.0,
               0.0};
  return message;
}

bool validPacket(const VIO::MonoDepthRawPacket& packet,
                 std::string* failure_reason) {
  const bool valid_intrinsics =
      std::isfinite(packet.intrinsics.fx) &&
      std::isfinite(packet.intrinsics.fy) &&
      std::isfinite(packet.intrinsics.cx) &&
      std::isfinite(packet.intrinsics.cy) && packet.intrinsics.fx > 0.0 &&
      packet.intrinsics.fy > 0.0 && packet.intrinsics.width > 0 &&
      packet.intrinsics.height > 0;
  const cv::Size expected_size(packet.intrinsics.width,
                               packet.intrinsics.height);
  if (!packet.source_image_is_undistorted || !valid_intrinsics ||
      !finitePose(packet.body_T_cam) || packet.source_image_bgr.empty() ||
      packet.source_image_bgr.type() != CV_8UC3 || packet.depth.empty() ||
      packet.depth.type() != CV_32FC1 ||
      !packet.confidence_filtering_enabled || !packet.confidence_valid ||
      !std::isfinite(packet.confidence_threshold) ||
      packet.confidence_threshold <= 0.0 || packet.valid_mask.empty() ||
      packet.valid_mask.type() != CV_8UC1 ||
      packet.source_image_bgr.size() != expected_size ||
      packet.depth.size() != expected_size ||
      packet.valid_mask.size() != expected_size) {
    *failure_reason =
        "DA3 frame has malformed images, confidence mask, calibration, or "
        "extrinsic";
    return false;
  }
  return true;
}

dense_mapping::msg::Da3Frame packetToMessage(
    const VIO::MonoDepthRawPacket& packet,
    const std::string& camera_frame_id) {
  dense_mapping::msg::Da3Frame message;
  const rclcpp::Time stamp(packet.timestamp);
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = camera_frame_id;
  message.stamp = stamp;
  message.keyframe_id = packet.keyframe_id;
  message.color = *cv_bridge::CvImage(
                       header,
                       sensor_msgs::image_encodings::BGR8,
                       packet.source_image_bgr)
                       .toImageMsg();
  message.depth = *cv_bridge::CvImage(
                       header,
                       sensor_msgs::image_encodings::TYPE_32FC1,
                       packet.depth)
                       .toImageMsg();
  message.support_mask = *cv_bridge::CvImage(
                              header,
                              sensor_msgs::image_encodings::MONO8,
                              packet.valid_mask)
                              .toImageMsg();
  message.camera_info = packetCameraInfo(packet, header);
  message.body_t_camera = poseToMessage(packet.body_T_cam);
  return message;
}

}  // namespace

DenseMappingPublisher::DenseMappingPublisher(
    rclcpp::Node::SharedPtr node,
    const VIO::CameraParams& camera_params)
    : node_(std::move(node)),
      camera_params_(camera_params),
      camera_frame_id_(camera_params.camera_id_) {
  if (!node_ || camera_frame_id_.empty() || camera_params_.image_size_.width <= 0 ||
      camera_params_.image_size_.height <= 0 ||
      !std::isfinite(camera_params_.intrinsics_[0]) ||
      !std::isfinite(camera_params_.intrinsics_[1]) ||
      camera_params_.intrinsics_[0] <= 0.0 ||
      camera_params_.intrinsics_[1] <= 0.0 ||
      !finitePose(camera_params_.body_Pose_cam_)) {
    throw std::invalid_argument(
        "dense mapping publisher requires a valid named camera calibration");
  }
  const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  da3_run_publisher_ =
      node_->create_publisher<dense_mapping::msg::Da3Run>("mapping/da3_runs",
                                                          qos);
  keyframe_publisher_ =
      node_->create_publisher<dense_mapping::msg::KeyframeState>(
          "mapping/keyframes", qos);
  local_window_publisher_ =
      node_->create_publisher<pose_graph_tools_msgs::msg::PoseGraph>(
          "mapping/local_window_poses", qos);
}

void DenseMappingPublisher::publish(
    const VIO::BackendOutput::Ptr& output,
    const nav_msgs::msg::Odometry& odometry) const {
  if (!output) {
    throw std::invalid_argument("cannot publish a null backend output");
  }
  local_window_publisher_->publish(makeLocalWindowState(*output, odometry));
  keyframe_publisher_->publish(makeKeyframeState(*output, odometry));

  if (!output->da3_packet_) {
    return;
  }
  const auto& packet = output->da3_packet_;
  const bool has_any_run_metadata = packet->da3_context_packet ||
                                    packet->da3_context_keyframe_id.has_value() ||
                                    packet->da3_context_cam_T_current_cam.has_value();
  if (!has_any_run_metadata) {
    return;
  }
  dense_mapping::msg::Da3Run message;
  std::string failure_reason;
  if (!makeDa3Run(packet, &message, &failure_reason)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Skipping malformed DA3 run for keyframe %zu: %s",
                 static_cast<std::size_t>(packet->keyframe_id),
                 failure_reason.c_str());
    return;
  }
  da3_run_publisher_->publish(message);
}

dense_mapping::msg::KeyframeState DenseMappingPublisher::makeKeyframeState(
    const VIO::BackendOutput& output,
    const nav_msgs::msg::Odometry& odometry) const {
  // Kimera's initialization output (keyframe 0) legitimately has no frontend
  // measurement bundle. It is still a sparse keyframe and must be published;
  // all later keyframes require their current, non-historical observations.
  if (!output.keyframe_measurements_ && output.cur_kf_id_ != 0u) {
    throw std::invalid_argument(
        "backend output is missing current keyframe measurements");
  }
  dense_mapping::msg::KeyframeState message;
  message.keyframe_id = output.cur_kf_id_;
  message.odometry = odometry;
  message.camera_info.header = odometry.header;
  message.camera_info.header.frame_id = camera_frame_id_;
  message.camera_info.width = camera_params_.image_size_.width;
  message.camera_info.height = camera_params_.image_size_.height;
  message.camera_info.distortion_model = "none";
  message.camera_info.k = {camera_params_.intrinsics_[0],
                           0.0,
                           camera_params_.intrinsics_[2],
                           0.0,
                           camera_params_.intrinsics_[1],
                           camera_params_.intrinsics_[3],
                           0.0,
                           0.0,
                           1.0};
  message.body_t_camera = poseToMessage(camera_params_.body_Pose_cam_);

  const VIO::StereoMeasurements empty_measurements;
  const auto& measurements = output.keyframe_measurements_
                                 ? output.keyframe_measurements_->second
                                 : empty_measurements;
  message.observations.reserve(measurements.size());
  std::size_t rejected_observations = 0u;
  for (const VIO::StereoMeasurement& measurement : measurements) {
    if (measurement.first < 0 || !std::isfinite(measurement.second.uL()) ||
        !std::isfinite(measurement.second.v()) ||
        !std::isfinite(measurement.px_sigma_) ||
        !std::isfinite(measurement.score_)) {
      ++rejected_observations;
      continue;
    }
    dense_mapping::msg::FeatureObservation observation;
    observation.landmark_id = measurement.first;
    observation.left_u = measurement.second.uL();
    observation.left_v = measurement.second.v();
    observation.has_right_pixel = std::isfinite(measurement.second.uR());
    if (observation.has_right_pixel) {
      observation.right_u = measurement.second.uR();
      observation.right_v = measurement.second.v();
    }
    observation.pixel_sigma = measurement.px_sigma_;
    observation.score = measurement.score_;
    message.observations.push_back(std::move(observation));
  }
  if (rejected_observations > 0u) {
    RCLCPP_WARN(node_->get_logger(),
                "Keyframe %zu omitted %zu malformed feature candidates from "
                "the dense-mapping observation update",
                static_cast<std::size_t>(output.cur_kf_id_),
                rejected_observations);
  }

  VIO::PointsWithIdMap landmark_updates = output.landmarks_out_local_window_;
  landmark_updates.insert(output.landmarks_in_local_window_.begin(),
                          output.landmarks_in_local_window_.end());
  message.landmark_updates.reserve(landmark_updates.size());
  for (const auto& [id, point] : landmark_updates) {
    if (id < 0 || !point.allFinite()) {
      throw std::invalid_argument("backend landmark update is malformed");
    }
    dense_mapping::msg::Landmark landmark;
    landmark.id = id;
    landmark.position.x = point.x();
    landmark.position.y = point.y();
    landmark.position.z = point.z();
    const auto count = output.lmk_num_observations_.find(id);
    landmark.observation_count =
        count == output.lmk_num_observations_.end() ? 0u : count->second;
    const auto residual = output.lmk_smart_factor_residuals_.find(id);
    landmark.residual =
        residual == output.lmk_smart_factor_residuals_.end()
            ? 0.0
            : residual->second;
    if (!std::isfinite(landmark.residual)) {
      throw std::invalid_argument("backend landmark residual is malformed");
    }
    message.landmark_updates.push_back(std::move(landmark));
  }
  return message;
}

pose_graph_tools_msgs::msg::PoseGraph
DenseMappingPublisher::makeLocalWindowState(
    const VIO::BackendOutput& output,
    const nav_msgs::msg::Odometry& odometry) const {
  if (odometry.header.frame_id.empty() ||
      !finitePose(output.W_State_Blkf_.pose_)) {
    throw std::invalid_argument(
        "local-window pose snapshot requires valid output odometry");
  }

  const gtsam::Key current_key =
      gtsam::Symbol(VIO::kPoseSymbolChar, output.cur_kf_id_);
  if (!output.state_.exists(current_key)) {
    throw std::invalid_argument(
        "backend local-window state is missing the current keyframe pose");
  }
  const gtsam::Pose3 smoother_T_current =
      output.state_.at<gtsam::Pose3>(current_key);
  if (!finitePose(smoother_T_current)) {
    throw std::invalid_argument(
        "backend current smoother pose is malformed");
  }

  // state_ is expressed in the fixed-lag smoother frame, while the regular
  // odometry output is expressed in the incrementally propagated world frame.
  // Anchor the complete snapshot with the exact current pose used by the ROS
  // odometry publisher so every emitted pose shares odometry.header.frame_id.
  const gtsam::Pose3 odometry_T_smoother =
      output.W_State_Blkf_.pose_ * smoother_T_current.inverse();

  pose_graph_tools_msgs::msg::PoseGraph message;
  message.header = odometry.header;
  message.nodes.reserve(output.state_.size());
  bool found_current = false;
  for (const gtsam::Key key : output.state_.keys()) {
    const gtsam::Symbol symbol(key);
    if (symbol.chr() != VIO::kPoseSymbolChar) {
      continue;
    }
    const gtsam::Pose3 smoother_T_body = output.state_.at<gtsam::Pose3>(key);
    const gtsam::Pose3 odometry_T_body =
        odometry_T_smoother * smoother_T_body;
    if (!finitePose(odometry_T_body)) {
      throw std::invalid_argument(
          "backend local-window state contains a malformed pose");
    }
    pose_graph_tools_msgs::msg::PoseGraphNode pose;
    pose.header = odometry.header;
    pose.robot_id = 0;
    pose.key = symbol.index();
    pose.pose = poseToMessage(odometry_T_body);
    found_current = found_current || pose.key == output.cur_kf_id_;
    message.nodes.push_back(std::move(pose));
  }
  if (message.nodes.empty() || !found_current) {
    throw std::invalid_argument(
        "backend local-window state contains no current pose");
  }
  return message;
}

bool DenseMappingPublisher::makeDa3Run(
    const VIO::MonoDepthRawPacket::ConstPtr& packet,
    dense_mapping::msg::Da3Run* message,
    std::string* failure_reason) const {
  if (!message || !failure_reason || !packet || !packet->da3_context_packet ||
      !packet->da3_context_keyframe_id.has_value() ||
      !packet->da3_context_cam_T_current_cam.has_value()) {
    if (failure_reason) {
      *failure_reason = "DA3 run metadata is incomplete";
    }
    return false;
  }
  const auto& context = packet->da3_context_packet;
  if (context->keyframe_id != *packet->da3_context_keyframe_id ||
      context->keyframe_id >= packet->keyframe_id ||
      context->confidence_threshold != packet->confidence_threshold ||
      !finitePose(*packet->da3_context_cam_T_current_cam) ||
      !validPacket(*context, failure_reason) ||
      !validPacket(*packet, failure_reason)) {
    if (failure_reason->empty()) {
      *failure_reason = "DA3 run IDs or relative pose are malformed";
    }
    return false;
  }
  message->header.stamp = rclcpp::Time(packet->timestamp);
  message->header.frame_id = camera_frame_id_;
  message->context = packetToMessage(*context, camera_frame_id_);
  message->current = packetToMessage(*packet, camera_frame_id_);
  message->context_camera_t_current_camera =
      poseToMessage(*packet->da3_context_cam_T_current_cam);
  return true;
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
