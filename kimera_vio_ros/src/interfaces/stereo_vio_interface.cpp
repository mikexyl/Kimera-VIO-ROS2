#include "kimera_vio_ros/interfaces/stereo_vio_interface.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "projective_mesher_msgs/msg/landmark_observation.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{
namespace
{

void poseToMsg(const gtsam::Pose3 & pose, geometry_msgs::msg::Pose * msg)
{
  const auto & t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  msg->position.x = t.x();
  msg->position.y = t.y();
  msg->position.z = t.z();
  msg->orientation.w = q.w();
  msg->orientation.x = q.x();
  msg->orientation.y = q.y();
  msg->orientation.z = q.z();
}

geometry_msgs::msg::Point pointToMsg(const gtsam::Point3 & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

std::optional<std::string> getRosImageEncoding(const cv::Mat & image)
{
  namespace enc = sensor_msgs::image_encodings;
  switch (image.type()) {
    case CV_8UC1:
      return enc::MONO8;
    case CV_8UC3:
      return enc::BGR8;
    case CV_8UC4:
      return enc::BGRA8;
    case CV_16UC1:
      return enc::MONO16;
    case CV_32FC1:
      return enc::TYPE_32FC1;
    default:
      return std::nullopt;
  }
}

void fillImageMessage(
  const cv::Mat & image,
  const std_msgs::msg::Header & header,
  sensor_msgs::msg::Image * image_msg)
{
  image_msg->header = header;
  image_msg->height = static_cast<uint32_t>(std::max(0, image.rows));
  image_msg->width = static_cast<uint32_t>(std::max(0, image.cols));
  if (image.empty()) {
    return;
  }

  const auto encoding = getRosImageEncoding(image);
  if (!encoding) {
    return;
  }

  cv_bridge::CvImage cv_image;
  cv_image.header = header;
  cv_image.encoding = *encoding;
  cv_image.image = image;
  cv_image.toImageMsg(*image_msg);
}

}  // namespace

StereoVioInterface::StereoVioInterface(
  rclcpp::Node::SharedPtr & node)
: BaseInterface(node),
  ImageInterface(node),
  ImuInterface(node),
  StereoInterface(node),
  BackendInterface(node)
{
  vio_pipeline_->registerBackendOutputCallback(
    std::bind(
      &BackendInterface::callbackBackendOutput,
      static_cast<BackendInterface *>(this),
      std::placeholders::_1));

  publish_mesher_input_ =
    node_->declare_parameter("publish_mesher_input", true);
  const int mesher_queue_size =
    node_->declare_parameter("mesher_input_queue_size", 30);
  mesher_input_queue_size_ =
    static_cast<size_t>(std::max(1, mesher_queue_size));

  if (publish_mesher_input_) {
    mesher_input_pub_ = node_->create_publisher<MesherInputMsg>(
      "mesher_input", rclcpp::QoS(10));
    vio_pipeline_->registerFrontendOutputCallback(
      std::bind(
        &StereoVioInterface::callbackMesherFrontendOutput,
        this,
        std::placeholders::_1));
    vio_pipeline_->registerBackendOutputCallback(
      std::bind(
        &StereoVioInterface::callbackMesherBackendOutput,
        this,
        std::placeholders::_1));
  }
  BaseInterface::start();
}

StereoVioInterface::~StereoVioInterface()
{
}

void StereoVioInterface::callbackMesherFrontendOutput(
  const VIO::FrontendOutput::Ptr & output)
{
  if (!publish_mesher_input_ || !output || !output->is_keyframe_) {
    return;
  }

  VIO::FrontendOutput::Ptr frontend_to_publish;
  VIO::BackendOutput::Ptr backend_to_publish;
  {
    std::lock_guard<std::mutex> lock(mesher_input_mutex_);
    mesher_frontend_outputs_[output->timestamp_] = output;
    collectSyncedMesherInput(
      output->timestamp_, &frontend_to_publish, &backend_to_publish);
    pruneMesherInputQueues();
  }

  if (frontend_to_publish && backend_to_publish) {
    publishMesherInput(frontend_to_publish, backend_to_publish);
  }
}

void StereoVioInterface::callbackMesherBackendOutput(
  const VIO::BackendOutput::Ptr & output)
{
  if (!publish_mesher_input_ || !output) {
    return;
  }

  VIO::FrontendOutput::Ptr frontend_to_publish;
  VIO::BackendOutput::Ptr backend_to_publish;
  {
    std::lock_guard<std::mutex> lock(mesher_input_mutex_);
    mesher_backend_outputs_[output->timestamp_] = output;
    collectSyncedMesherInput(
      output->timestamp_, &frontend_to_publish, &backend_to_publish);
    pruneMesherInputQueues();
  }

  if (frontend_to_publish && backend_to_publish) {
    publishMesherInput(frontend_to_publish, backend_to_publish);
  }
}

bool StereoVioInterface::collectSyncedMesherInput(
  const VIO::Timestamp & timestamp,
  VIO::FrontendOutput::Ptr * frontend_output,
  VIO::BackendOutput::Ptr * backend_output)
{
  auto frontend_iter = mesher_frontend_outputs_.find(timestamp);
  auto backend_iter = mesher_backend_outputs_.find(timestamp);
  if (frontend_iter == mesher_frontend_outputs_.end() ||
    backend_iter == mesher_backend_outputs_.end())
  {
    return false;
  }

  *frontend_output = frontend_iter->second;
  *backend_output = backend_iter->second;
  mesher_frontend_outputs_.erase(frontend_iter);
  mesher_backend_outputs_.erase(backend_iter);
  return true;
}

void StereoVioInterface::pruneMesherInputQueues()
{
  while (mesher_frontend_outputs_.size() > mesher_input_queue_size_) {
    mesher_frontend_outputs_.erase(mesher_frontend_outputs_.begin());
  }
  while (mesher_backend_outputs_.size() > mesher_input_queue_size_) {
    mesher_backend_outputs_.erase(mesher_backend_outputs_.begin());
  }
}

void StereoVioInterface::publishMesherInput(
  const VIO::FrontendOutput::Ptr & frontend_output,
  const VIO::BackendOutput::Ptr & backend_output) const
{
  if (!mesher_input_pub_ || !frontend_output || !backend_output) {
    return;
  }

  const auto & stereo_frame = frontend_output->stereo_frame_lkf_;
  const auto & left_frame = stereo_frame.getLeftFrame();

  MesherInputMsg msg;
  msg.header.stamp = rclcpp::Time(backend_output->timestamp_);
  msg.header.frame_id = world_frame_id_;
  msg.frame_id = backend_output->cur_kf_id_;
  fillImageMessage(left_frame.img_, msg.header, &msg.image);

  const gtsam::Pose3 left_camera_pose =
    backend_output->W_State_Blkf_.pose_ *
    vio_params_->camera_params_.at(0).body_Pose_cam_;
  poseToMsg(left_camera_pose, &msg.camera_pose);

  const auto & landmark_ids = left_frame.landmarks_;
  const auto & keypoints = left_frame.keypoints_;
  const auto & keypoint_status = stereo_frame.right_keypoints_status_;
  const auto & camera_points = stereo_frame.keypoints_3d_;
  const bool has_status = keypoint_status.size() == landmark_ids.size();
  const bool has_camera_points = camera_points.size() == landmark_ids.size();

  msg.observations.reserve(landmark_ids.size());

  for (size_t idx = 0; idx < landmark_ids.size(); ++idx) {
    if (idx >= keypoints.size()) {
      break;
    }

    const VIO::LandmarkId landmark_id = landmark_ids.at(idx);
    if (landmark_id < 0) {
      continue;
    }

    projective_mesher_msgs::msg::LandmarkObservation observation;
    observation.id = static_cast<int64_t>(landmark_id);

    observation.keypoint.x = keypoints.at(idx).x;
    observation.keypoint.y = keypoints.at(idx).y;
    observation.keypoint.z = 0.0;

    const auto status =
      has_status ? keypoint_status.at(idx) : VIO::VALID;
    const auto world_iter =
      backend_output->landmarks_with_id_map_.find(landmark_id);
    if (world_iter != backend_output->landmarks_with_id_map_.end()) {
      observation.world_point = pointToMsg(world_iter->second);
    } else if (status == VIO::VALID && has_camera_points) {
      const gtsam::Point3 world_point =
        left_camera_pose.transformFrom(gtsam::Point3(camera_points.at(idx)));
      observation.world_point = pointToMsg(world_point);
    } else {
      continue;
    }

    if (has_camera_points) {
      observation.has_camera_point = true;
      observation.camera_point = pointToMsg(camera_points.at(idx));
    }

    observation.status = static_cast<uint8_t>(status);
    msg.observations.push_back(std::move(observation));
  }

  mesher_input_pub_->publish(std::move(msg));
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
