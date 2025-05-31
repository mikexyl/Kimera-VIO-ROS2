#pragma once

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

#include "kimera_vio_ros/utils/geometry.hpp"

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;
using Odometry = nav_msgs::msg::Odometry;

namespace kimera_vio_ros {
namespace interfaces {

class Ros2DataProviderInterface : public VIO::DataProviderInterface {
 public:
  Ros2DataProviderInterface(rclcpp::Node::SharedPtr& node,
                            VIO::VioParams::Ptr vio_params,
                            VIO::StereoImuPipeline::Ptr vio_pipeline,
                            std::string base_link_frame_id,
                            std::string map_frame_id,
                            std::string world_frame_id)
      : vio_params_(vio_params),
        base_link_frame_id_(base_link_frame_id),
        map_frame_id_(map_frame_id),
        world_frame_id_(world_frame_id),
        node_(node),
        vio_pipeline_(vio_pipeline) {
    this->registerImuSingleCallback(
        std::bind(&VIO::Pipeline::fillSingleImuQueue,
                  vio_pipeline.get(),
                  std::placeholders::_1));

    this->registerImuMultiCallback(std::bind(&VIO::Pipeline::fillMultiImuQueue,
                                             vio_pipeline.get(),
                                             std::placeholders::_1));

    callback_group_imu_ = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto imu_opt = rclcpp::SubscriptionOptions();
    imu_opt.callback_group = callback_group_imu_;

    std::string imu_topic = "imu/data";
    auto qos = rclcpp::SensorDataQoS();
    imu_sub_ = node->create_subscription<Imu>(
        imu_topic,
        qos,
        std::bind(
            &Ros2DataProviderInterface::imu_cb, this, std::placeholders::_1),
        imu_opt);

    this->registerLeftFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillLeftFrameQueue,
                  vio_pipeline.get(),
                  std::placeholders::_1));

    this->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  vio_pipeline.get(),
                  std::placeholders::_1));

    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    // tf_listener_ =
    //     std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_,
    //     false);

    int queue_size_ = 10;

    auto image_qos = rclcpp::SensorDataQoS();
    std::string left_image_topic = "left/image";
    std::string right_image_topic = "right/image";
    // TODO: Perhaps switch to image_transport to support more transports
    // std::string transport = "raw";
    // image_transport::TransportHints hints(node, transport);
    // left_sub_.subscribe(node, left_topic, hints.getTransport(),
    // qos.get_rmw_qos_profile()); right_sub_.subscribe(node, right_topic,
    // hints.getTransport(), qos.get_rmw_qos_profile()); exact_image_sync_ =
    // std::make_shared<ExactImageSync>(
    //   ExactImagePolicy(queue_size_), left_sub_, right_sub_);

    // TODO: Assign message filter subscribers to callback_group_stereo_
    // Pending: https://github.com/ros2/message_filters/issues/45
    // Create message_filters::Subscribers
    left_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
        node_, left_image_topic, image_qos.get_rmw_qos_profile());

    right_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
        node_, right_image_topic, image_qos.get_rmw_qos_profile());

    // Create synchronizer
    exact_image_sync_ = std::make_shared<ExactImageSync>(
        ExactImagePolicy(queue_size_), *left_image_sub_, *right_image_sub_);

    exact_image_sync_->registerCallback(
        &Ros2DataProviderInterface::stereo_image_cb, this);

    LOG(INFO) << "Subscribing to stereo image topics: " << left_image_topic
              << " and " << right_image_topic;
  }

  VIO::FrameId frame_count_{0};
  rclcpp::Time last_stereo_timestamp_{0};
  VIO::VioParams::Ptr vio_params_;

  typedef message_filters::sync_policies::ExactTime<Image, Image>
      ExactImagePolicy;
  typedef message_filters::Synchronizer<ExactImagePolicy> ExactImageSync;
  std::shared_ptr<ExactImageSync> exact_image_sync_;
  std::shared_ptr<message_filters::Subscriber<Image>> left_image_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> right_image_sub_;

  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string world_frame_id_;

  void stereo_image_cb(const Image::SharedPtr left_msg,
                       const Image::SharedPtr right_msg) {
    rclcpp::Time left_stamp(left_msg->header.stamp);
    rclcpp::Time right_stamp(right_msg->header.stamp);
    if (left_stamp.nanoseconds() > last_stereo_timestamp_.nanoseconds()) {
      static const VIO::CameraParams& left_cam_info =
          vio_params_->camera_params_.at(0);
      static const VIO::CameraParams& right_cam_info =
          vio_params_->camera_params_.at(1);

      const VIO::Timestamp& timestamp_left = left_stamp.nanoseconds();
      const VIO::Timestamp& timestamp_right = right_stamp.nanoseconds();

      CHECK(left_frame_callback_);
      CHECK(right_frame_callback_);

      left_frame_callback_(std::make_unique<VIO::Frame>(
          frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));
      right_frame_callback_(
          std::make_unique<VIO::Frame>(frame_count_,
                                       timestamp_right,
                                       right_cam_info,
                                       readRosImage(right_msg)));
      // LOG_EVERY_N(INFO, 30) << "Done: KimeraVioNode::stereo_image_cb";
      frame_count_++;
    }
    last_stereo_timestamp_ = left_stamp;
  }

  ~Ros2DataProviderInterface() = default;

  const cv::Mat readRosImage(const Image::ConstSharedPtr& img_msg) {
    cv_bridge::CvImageConstPtr cv_constptr;
    try {
      cv_constptr = cv_bridge::toCvShare(img_msg);
    } catch (cv_bridge::Exception& exception) {
      // RCLCPP_FATAL(this->get_logger(), "cv_bridge exception: %s",
      // exception.what()); rclcpp::shutdown();
    }

    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      // LOG(WARNING) << "Converting image...";
      cv::cvtColor(cv_constptr->image, cv_constptr->image, cv::COLOR_BGR2GRAY);
    } else {
      // CHECK_EQ(cv_constptr->encoding, sensor_msgs::image_encodings::MONO8)
      //     << "Expected image with MONO8 or BGR8 encoding.";
    }

    return cv_constptr->image;
  }

  void imu_cb(const Imu::SharedPtr imu_msg) {
    rclcpp::Time stamp(imu_msg->header.stamp);
    if (stamp.nanoseconds() > last_imu_timestamp_.nanoseconds()) {
      VIO::Timestamp timestamp = stamp.nanoseconds();
      VIO::ImuAccGyr imu_accgyr;

      imu_accgyr(0) = imu_msg->linear_acceleration.x;
      imu_accgyr(1) = imu_msg->linear_acceleration.y;
      imu_accgyr(2) = imu_msg->linear_acceleration.z;
      imu_accgyr(3) = imu_msg->angular_velocity.x;
      imu_accgyr(4) = imu_msg->angular_velocity.y;
      imu_accgyr(5) = imu_msg->angular_velocity.z;

      this->imu_single_callback_(VIO::ImuMeasurement(timestamp, imu_accgyr));
      //   LOG_EVERY_N(INFO, 200) << "Done: KimeraVioNode::imu_cb";
    }
    last_imu_timestamp_ = stamp;
  }

  // implement spin() from DataProviderInterface
  bool spin() override {
    return true;  // Return true to indicate that data is still available
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_imu_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Time last_imu_timestamp_;

  VIO::StereoImuPipeline::Ptr vio_pipeline_;
};
}  // namespace interfaces
}  // namespace kimera_vio_ros