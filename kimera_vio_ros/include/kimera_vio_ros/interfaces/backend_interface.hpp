#ifndef KIMERA_ROS__INTERFACES__BACKEND_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__BACKEND_INTERFACE_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kimera_vio_ros/interfaces/base_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <vector>

using Odometry = nav_msgs::msg::Odometry;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

namespace kimera_vio_ros
{
namespace interfaces
{

class BackendInterface : virtual public BaseInterface
{
public:
  BackendInterface(
    rclcpp::Node::SharedPtr & node);
  virtual ~BackendInterface();

public:
  inline void callbackBackendOutput(const VIO::BackendOutput::Ptr & output)
  {
    backend_output_queue_.push(output);
  }

protected:
  void publishBackendOutput(const VIO::BackendOutput::Ptr & output);

protected:
  VIO::ThreadsafeQueue<VIO::BackendOutput::Ptr> backend_output_queue_;

private:
  void drainBackendQueue();
  Odometry buildOdometryMessage(
    const VIO::Timestamp & ts,
    const gtsam::Pose3 & pose,
    const gtsam::Matrix6 & pose_cov,
    const VIO::Vector3 & linear_velocity_child,
    const gtsam::Matrix3 & vel_cov_child,
    const std::string & child_frame_id) const;
  void publishState(const VIO::BackendOutput::Ptr & output) const;
  void publishCameraStates(const VIO::BackendOutput::Ptr & output) const;
  void publishTf(const VIO::BackendOutput::Ptr & output);
  void publishTimeHorizonPointCloud(const VIO::BackendOutput::Ptr & output) const;
  // void publishImuBias(const VIO::BackendOutput::Ptr& output) const;

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_backend_;
  rclcpp::TimerBase::SharedPtr backend_timer_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> camera_odometry_pubs_;
  std::vector<std::string> camera_odometry_frame_ids_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;

};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__BACKEND_INTERFACE_HPP_
