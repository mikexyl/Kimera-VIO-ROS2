#include "kimera_vio_ros/interfaces/backend_interface.hpp"
#include "kimera_vio_ros/utils/geometry.hpp"

#include <chrono>

namespace kimera_vio_ros {
namespace interfaces {

BackendInterface::BackendInterface(rclcpp::Node::SharedPtr &node)
    : BaseInterface(node), backend_output_queue_("Backend output") {
  callback_group_backend_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  odometry_pub_ = node_->create_publisher<Odometry>("odometry", qos);
  camera_odometry_pubs_.reserve(vio_params_->camera_params_.size());
  camera_odometry_frame_ids_.reserve(vio_params_->camera_params_.size());
  for (size_t camera_idx = 0; camera_idx < vio_params_->camera_params_.size();
       ++camera_idx) {
    const auto &camera_params = vio_params_->camera_params_.at(camera_idx);
    const std::string frame_id =
        camera_params.camera_id_.empty()
            ? "camera_" + std::to_string(camera_idx)
            : camera_params.camera_id_;
    const std::string topic_name = frame_id + "_odometry";
    camera_odometry_pubs_.push_back(node_->create_publisher<Odometry>(topic_name, qos));
    camera_odometry_frame_ids_.push_back(frame_id);
    RCLCPP_INFO(
        node_->get_logger(),
        "Publishing camera odometry on '%s' for frame '%s'.",
        topic_name.c_str(), frame_id.c_str());
  }
  pointcloud_pub_ =
      node_->create_publisher<PointCloud2>("time_horizon_pointcloud", qos);
  backend_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&BackendInterface::drainBackendQueue, this),
      callback_group_backend_);
}

BackendInterface::~BackendInterface() { backend_output_queue_.shutdown(); }

void BackendInterface::drainBackendQueue() {
  VIO::BackendOutput::Ptr output;
  while (backend_output_queue_.pop(output)) {
    publishBackendOutput(output);
  }
}

void BackendInterface::publishBackendOutput(
    const VIO::BackendOutput::Ptr &output) {
  CHECK(output);
  RCLCPP_INFO_ONCE(
      node_->get_logger(),
      "Publishing first backend output to ROS odometry/TF topics.");
  publishTf(output);
  publishState(output);
  publishCameraStates(output);
  // if (imu_bias_pub_.getNumSubscribers() > 0) {
  //   publishImuBias(output);
  // }
  if (pointcloud_pub_->get_subscription_count() > 0) {
    publishTimeHorizonPointCloud(output);
  }
}

Odometry BackendInterface::buildOdometryMessage(
    const VIO::Timestamp &ts, const gtsam::Pose3 &pose,
    const gtsam::Matrix6 &pose_cov, const VIO::Vector3 &linear_velocity_child,
    const gtsam::Matrix3 &vel_cov_child,
    const std::string &child_frame_id) const {
  Odometry odometry_msg;

  // Create header.
  odometry_msg.header.stamp = rclcpp::Time(ts);
  odometry_msg.header.frame_id = world_frame_id_;
  odometry_msg.child_frame_id = child_frame_id;

  // Position
  odometry_msg.pose.pose.position.x = pose.x();
  odometry_msg.pose.pose.position.y = pose.y();
  odometry_msg.pose.pose.position.z = pose.z();

  // Orientation
  const gtsam::Quaternion &quaternion = pose.rotation().toQuaternion();
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

  odometry_msg.twist.twist.linear.x = linear_velocity_child(0);
  odometry_msg.twist.twist.linear.y = linear_velocity_child(1);
  odometry_msg.twist.twist.linear.z = linear_velocity_child(2);

  // Velocity covariance: first linear
  // and then angular (trivial values for angular)
  DCHECK_EQ(vel_cov_child.rows(), 3);
  DCHECK_EQ(vel_cov_child.cols(), 3);
  DCHECK_EQ(odometry_msg.twist.covariance.size(), 36);
  for (int i = 0; i < vel_cov_child.rows(); i++) {
    for (int j = 0; j < vel_cov_child.cols(); j++) {
      odometry_msg.twist
          .covariance[i * static_cast<int>(
                              sqrt(odometry_msg.twist.covariance.size())) +
                      j] = vel_cov_child(i, j);
    }
  }
  return odometry_msg;
}

void BackendInterface::publishState(
    const VIO::BackendOutput::Ptr &output) const {
  CHECK(output);
  const gtsam::Pose3 &pose = output->W_State_Blkf_.pose_;
  const gtsam::Rot3 &rotation = pose.rotation();
  const gtsam::Vector3 &velocity = output->W_State_Blkf_.velocity_;
  const gtsam::Matrix6 &pose_cov =
      gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3 &vel_cov =
      gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

  const gtsam::Matrix3 body_R_world = rotation.transpose();
  const VIO::Vector3 velocity_body = body_R_world * velocity;
  const gtsam::Matrix3 vel_cov_body =
      body_R_world * vel_cov * rotation.matrix();

  odometry_pub_->publish(buildOdometryMessage(
      output->timestamp_, pose, pose_cov, velocity_body, vel_cov_body,
      base_link_frame_id_));
}

void BackendInterface::publishCameraStates(
    const VIO::BackendOutput::Ptr &output) const {
  CHECK(output);
  if (camera_odometry_pubs_.empty()) {
    return;
  }

  const gtsam::Pose3 &body_pose = output->W_State_Blkf_.pose_;
  const gtsam::Rot3 &body_rotation = body_pose.rotation();
  const gtsam::Vector3 &velocity_world = output->W_State_Blkf_.velocity_;
  const gtsam::Matrix6 &pose_cov =
      gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3 &vel_cov_world =
      gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

  const gtsam::Matrix3 body_R_world = body_rotation.transpose();
  const VIO::Vector3 velocity_body = body_R_world * velocity_world;
  const gtsam::Matrix3 vel_cov_body =
      body_R_world * vel_cov_world * body_rotation.matrix();

  for (size_t camera_idx = 0; camera_idx < camera_odometry_pubs_.size();
       ++camera_idx) {
    const auto &publisher = camera_odometry_pubs_.at(camera_idx);
    const auto &camera_params = vio_params_->camera_params_.at(camera_idx);
    const gtsam::Pose3 camera_pose =
        body_pose * camera_params.body_Pose_cam_;
    const gtsam::Matrix3 camera_R_body =
        camera_params.body_Pose_cam_.rotation().transpose();
    const VIO::Vector3 velocity_camera = camera_R_body * velocity_body;
    const gtsam::Matrix3 vel_cov_camera =
        camera_R_body * vel_cov_body * camera_R_body.transpose();

    publisher->publish(buildOdometryMessage(
        output->timestamp_, camera_pose, pose_cov, velocity_camera,
        vel_cov_camera, camera_odometry_frame_ids_.at(camera_idx)));
  }
}

void BackendInterface::publishTf(const VIO::BackendOutput::Ptr &output) {
  CHECK(output);

  const VIO::Timestamp &timestamp = output->timestamp_;
  const gtsam::Pose3 &pose = output->W_State_Blkf_.pose_;
  // Publish base_link TF.
  TransformStamped odom_tf;
  odom_tf.header.stamp = rclcpp::Time(timestamp);
  odom_tf.header.frame_id = world_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;

  utils::poseToMsgTF(pose, &odom_tf.transform);
  tf_broadcaster_->sendTransform(odom_tf);
}

void BackendInterface::publishTimeHorizonPointCloud(
    const VIO::BackendOutput::Ptr &output) const {
  CHECK(output);
  const VIO::Timestamp &timestamp = output->timestamp_;
  const VIO::PointsWithIdMap &points_with_id =
      output->landmarks_with_id_map_;
  const VIO::LmkIdToLmkTypeMap &lmk_id_to_lmk_type_map =
      output->lmk_id_to_lmk_type_map_;

  if (points_with_id.size() == 0) {
    // No points to visualize.
    return;
  }

  PointCloud2::UniquePtr pc_msg(new PointCloud2);
  pc_msg->header.stamp = rclcpp::Time(timestamp);
  pc_msg->header.frame_id = world_frame_id_;
  pc_msg->width = points_with_id.size();
  pc_msg->height = 1;
  pc_msg->is_dense = true;
  pc_msg->point_step = 3 * sizeof(float) + 3 * sizeof(uint8_t);
  pc_msg->row_step = pc_msg->point_step * pc_msg->width;
  pc_msg->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(pc_msg->width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc_msg, "b");

  bool color_the_cloud = false;
  if (lmk_id_to_lmk_type_map.size() != 0) {
    color_the_cloud = true;
    CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
  }

  // Populate cloud structure with 3D points.
  size_t i = 0;
  for (const std::pair<VIO::LandmarkId, gtsam::Point3> id_point :
       points_with_id) {
    const gtsam::Point3 point_3d = id_point.second;
    *iter_x = static_cast<float>(point_3d.x());
    *iter_y = static_cast<float>(point_3d.y());
    *iter_z = static_cast<float>(point_3d.z());

    if (color_the_cloud) {
      DCHECK(lmk_id_to_lmk_type_map.find(id_point.first) !=
             lmk_id_to_lmk_type_map.end());
      switch (lmk_id_to_lmk_type_map.at(id_point.first)) {
      case VIO::LandmarkType::SMART: {
        *iter_r = 0;
        *iter_g = 255;
        *iter_b = 0;
        break;
      }
      case VIO::LandmarkType::PROJECTION: {
        *iter_r = 0;
        *iter_g = 0;
        *iter_b = 255;
        break;
      }
      default: {
        *iter_r = 255;
        *iter_g = 0;
        *iter_b = 0;
        break;
      }
      }
    }
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
    i++;
  }
  pointcloud_pub_->publish(std::move(pc_msg));
}

} // namespace interfaces
} // namespace kimera_vio_ros
