#pragma once

#include <chrono>
using namespace std::chrono_literals;

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/pipeline/StereoImuPipeline.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "kimera_vio_ros/interfaces/ros2_data_provider.hpp"
#include "kimera_vio_ros/interfaces/ros2_visualizer.hpp"
#include "kimera_vio_ros/utils/geometry.hpp"

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;
using Odometry = nav_msgs::msg::Odometry;

namespace kimera_vio_ros {
namespace interfaces {

class StereoVioInterface {
 public:
  StereoVioInterface(rclcpp::Node::SharedPtr& node) : node_(node) {
    std::string params_folder_;
    params_folder_ = node_->declare_parameter("params_folder", "");
    CHECK(!params_folder_.empty());
    vio_params_ = std::make_shared<VIO::VioParams>(params_folder_);

    callback_group_pipeline_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
    base_link_frame_id_ =
        node_->declare_parameter("frame_id.base_link", "base_link");
    map_frame_id_ = node_->declare_parameter("frame_id.map", "map");
    world_frame_id_ = node_->declare_parameter("frame_id.world", "world");

    int queue_size_ = 10;

    auto info_qos = rclcpp::SystemDefaultsQoS();
    std::string left_info_topic = "left/camera_info";
    std::string right_info_topic = "right/camera_info";
    left_info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
        node_, left_info_topic, info_qos.get_rmw_qos_profile());
    right_info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
        node_, right_info_topic, info_qos.get_rmw_qos_profile());
    left_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
        node_, "left/image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    right_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
        node_, "right/image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    exact_info_sync_ =
        std::make_shared<ExactInfoSync>(ExactInfoPolicy(queue_size_),
                                        *left_info_sub_,
                                        *right_info_sub_,
                                        *left_image_sub_,
                                        *right_image_sub_);
    exact_info_sync_->registerCallback(&StereoVioInterface::stereo_info_cb,
                                       this);
  }
  ~StereoVioInterface() {
    if (vio_pipeline_) {
      vio_pipeline_->shutdown();
      if (handle_pipeline_.joinable()) {
        handle_pipeline_.join();
      }
      LOG(INFO) << "Destroying Stereo VIO Interface...";
    }
    if (data_provider_interface_) {
      data_provider_interface_->shutdown();
      if (handle_data_provider_.joinable()) {
        handle_data_provider_.join();
      }
      LOG(INFO) << "Destroying Ros2DataProviderInterface...";
    }
  }

  VIO::VioParams::Ptr vio_params_;
  VIO::StereoImuPipeline::Ptr vio_pipeline_{nullptr};
  Ros2DataProviderInterface::Ptr data_provider_interface_{nullptr};

  std::thread handle_pipeline_;
  std::thread handle_data_provider_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr pipeline_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_pipeline_;

  typedef message_filters::sync_policies::
      ExactTime<CameraInfo, CameraInfo, Image, Image>
          ExactInfoPolicy;
  typedef message_filters::Synchronizer<ExactInfoPolicy> ExactInfoSync;
  std::shared_ptr<ExactInfoSync> exact_info_sync_;
  std::shared_ptr<message_filters::Subscriber<CameraInfo>> left_info_sub_;
  std::shared_ptr<message_filters::Subscriber<CameraInfo>> right_info_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> left_image_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> right_image_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string world_frame_id_;

  void start() {
    LOG(INFO) << "Starting Stereo VIO Interface...";
    if (vio_params_->parallel_run_) {
      handle_pipeline_ = std::thread(&VIO::Pipeline::spin, vio_pipeline_.get());
    } else {
      pipeline_timer_ = node_->create_wall_timer(
          10ms,
          std::bind(&VIO::Pipeline::spin, vio_pipeline_.get()),
          callback_group_pipeline_);
    }
  }

  void stereo_info_cb(const CameraInfo::ConstSharedPtr& left_msg,
                      const CameraInfo::ConstSharedPtr& right_msg,
                      const Image::ConstSharedPtr& left_image_msg,
                      const Image::ConstSharedPtr& right_image_msg) {
    CHECK_GE(vio_params_->camera_params_.size(), 2u);

    // print out the frame id for debugging
    LOG(INFO) << "Received camera info for left: " << left_msg->header.frame_id
              << " and right: " << right_msg->header.frame_id;

    // Initialize CameraParams for pipeline.
    if (not msgCamInfoToCameraParams(left_msg,
                                     left_image_msg->header.frame_id,
                                     &vio_params_->camera_params_.at(0)) or
        not msgCamInfoToCameraParams(right_msg,
                                     right_image_msg->header.frame_id,
                                     &vio_params_->camera_params_.at(1))) {
      LOG(WARNING) << "Failed to convert CameraInfo messages to CameraParams.";
      return;
    }

    vio_params_->camera_params_.at(0).print();
    vio_params_->camera_params_.at(1).print();

    if (not vio_pipeline_) {
      auto visualizer = std::make_unique<Ros2Visualizer>(
          node_, base_link_frame_id_, map_frame_id_, world_frame_id_);
      vio_pipeline_ = std::make_shared<VIO::StereoImuPipeline>(
          *vio_params_, std::move(visualizer));
    }

    if (not data_provider_interface_) {
      // Signal the correct reception of camera info
      RCLCPP_INFO(node_->get_logger(), "Received camera parameters.");

      data_provider_interface_ =
          std::make_unique<Ros2DataProviderInterface>(node_,
                                                      vio_params_,
                                                      vio_pipeline_,
                                                      base_link_frame_id_,
                                                      map_frame_id_,
                                                      world_frame_id_);

      start();
    }

    // unsubscribe from the camera info topics
    left_info_sub_->unsubscribe();
    right_info_sub_->unsubscribe();
  }

  bool msgCamInfoToCameraParams(const CameraInfo::ConstSharedPtr& cam_info,
                                std::string frame_id,
                                VIO::CameraParams* cam_params) {
    CHECK_NOTNULL(cam_params);

    // Get intrinsics from incoming CameraInfo messages:
    cam_params->camera_id_ = frame_id;
    CHECK(!cam_params->camera_id_.empty());

    CHECK(cam_info->distortion_model == "plumb_bob" ||
          cam_info->distortion_model == "equidistant");

    if (cam_info->distortion_model == "plumb_bob") {
      // Kimera-VIO terms the plumb bob dist. model the as radtan.
      cam_params->distortion_model_ = VIO::DistortionModel::RADTAN;
      // Kimera-VIO can't take a 6th order radial distortion term.
      CHECK_EQ(cam_info->d.size(), 5);
    } else {
      LOG(FATAL) << "Other distortion models not supported yet: "
                 << cam_info->distortion_model;
    }

    const std::vector<double>& distortion_coeffs =
        std::vector<double>(cam_info->d.begin(), cam_info->d.end());

    CHECK_GE(distortion_coeffs.size(), 4);
    VIO::CameraParams::convertDistortionVectorToMatrix(
        distortion_coeffs, &cam_params->distortion_coeff_mat_);

    cam_params->image_size_ = cv::Size(cam_info->width, cam_info->height);

    cam_params->frame_rate_ = 0;  // TODO(marcus): is there a way to get this?

    std::array<double, 4> intrinsics = {
        cam_info->k[0], cam_info->k[4], cam_info->k[2], cam_info->k[5]};
    cam_params->intrinsics_ = intrinsics;
    VIO::CameraParams::convertIntrinsicsVectorToMatrix(cam_params->intrinsics_,
                                                       &cam_params->K_);

    // VIO::CameraParams::createGtsamCalibration(cam_params->distortion_coeff_,
    //                                           cam_params->intrinsics_,
    //                                           &cam_params->calibration_);

    // Get extrinsics from the TF tree:
    geometry_msgs::msg::TransformStamped cam_tf;

    // return false if the camera frame is not found
    if (not tf_buffer_->canTransform(base_link_frame_id_,
                                     cam_params->camera_id_,
                                     cam_info->header.stamp)) {
      LOG(WARNING) << "TF for camera frame " << cam_params->camera_id_
                   << " not found. Will not set body_Pose_cam_";
      return false;
    }

    try {
      // print base link and caemra id for debugg
      LOG(INFO) << "Looking up TF for camera: " << cam_params->camera_id_
                << " with base link: " << base_link_frame_id_;
      cam_tf = tf_buffer_->lookupTransform(
          base_link_frame_id_, cam_params->camera_id_, cam_info->header.stamp);
    } catch (tf2::TransformException& ex) {
      RCLCPP_FATAL(
          node_->get_logger(),
          "TF for left/right camera frames not available. Either publish to "
          "tree or provide CameraParameter yaml files.:\n%s",
          ex.what());
      rclcpp::shutdown();
    }

    utils::msgTFtoPose(cam_tf.transform, &cam_params->body_Pose_cam_);
    cam_params->body_Pose_cam_.print();

    return true;
  }
};

}  // namespace interfaces
}  // namespace kimera_vio_ros
