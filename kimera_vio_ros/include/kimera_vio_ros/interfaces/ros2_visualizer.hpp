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

#include <future>
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

class Ros2Visualizer : public VIO::Visualizer3D {
 public:
  Ros2Visualizer(rclcpp::Node::SharedPtr& node,
                 std::string base_link_frame_id,
                 std::string map_frame_id,
                 std::string world_frame_id)
      : Visualizer3D(VIO::VisualizationType::kNone),
        node_(node),
        base_link_frame_id_(base_link_frame_id),
        map_frame_id_(map_frame_id),
        world_frame_id_(world_frame_id) {
    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odometry_pub_ = node_->create_publisher<Odometry>("odometry", odom_qos);
  }

  ~Ros2Visualizer() = default;

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    this->publishState(input.backend_output_);

    return std::make_unique<VIO::VisualizerOutput>();
  }

  void publishState(const VIO::BackendOutput::ConstPtr& output) const {
    CHECK(output);
    // Get latest estimates for odometry.
    const VIO::Timestamp& ts = output->timestamp_;
    const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
    const gtsam::Rot3& rotation = pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();
    const gtsam::Vector3& velocity = output->W_State_Blkf_.velocity_;
    const gtsam::Matrix6& pose_cov =
        gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
    const gtsam::Matrix3& vel_cov =
        gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

    // First publish odometry estimate
    Odometry odometry_msg;

    // Create header.
    odometry_msg.header.stamp = rclcpp::Time(ts);
    odometry_msg.header.frame_id = world_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;

    // Position
    odometry_msg.pose.pose.position.x = pose.x();
    odometry_msg.pose.pose.position.y = pose.y();
    odometry_msg.pose.pose.position.z = pose.z();

    // Orientation
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();

    // Remap covariance from GTSAM convention
    // to odometry convention and fill in covariance
    static const std::vector<int> remapping{3, 4, 5, 0, 1, 2};

    // Position covariance first, angular covariance after
    DCHECK_EQ(pose_cov.rows(), remapping.size());
    DCHECK_EQ(pose_cov.rows() * pose_cov.cols(),
              odometry_msg.pose.covariance.size());
    for (int i = 0; i < pose_cov.rows(); i++) {
      for (int j = 0; j < pose_cov.cols(); j++) {
        odometry_msg.pose
            .covariance[remapping[i] * pose_cov.cols() + remapping[j]] =
            pose_cov(i, j);
      }
    }

    // Linear velocities, trivial values for angular
    const gtsam::Matrix3& inversed_rotation = rotation.transpose();
    const VIO::Vector3 velocity_body = inversed_rotation * velocity;
    odometry_msg.twist.twist.linear.x = velocity_body(0);
    odometry_msg.twist.twist.linear.y = velocity_body(1);
    odometry_msg.twist.twist.linear.z = velocity_body(2);

    // Velocity covariance: first linear
    // and then angular (trivial values for angular)
    const gtsam::Matrix3 vel_cov_body =
        inversed_rotation.matrix() * vel_cov * rotation.matrix();
    DCHECK_EQ(vel_cov_body.rows(), 3);
    DCHECK_EQ(vel_cov_body.cols(), 3);
    DCHECK_EQ(odometry_msg.twist.covariance.size(), 36);
    for (int i = 0; i < vel_cov_body.rows(); i++) {
      for (int j = 0; j < vel_cov_body.cols(); j++) {
        odometry_msg.twist
            .covariance[i * static_cast<int>(
                                sqrt(odometry_msg.twist.covariance.size())) +
                        j] = vel_cov_body(i, j);
      }
    }
    // Publish message
    odometry_pub_->publish(odometry_msg);
  }

 private:
  rclcpp::Node::SharedPtr node_;

  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string world_frame_id_;

  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
};