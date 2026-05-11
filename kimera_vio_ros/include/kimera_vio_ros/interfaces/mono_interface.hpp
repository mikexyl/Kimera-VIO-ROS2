#ifndef KIMERA_ROS__INTERFACES__MONO_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__MONO_INTERFACE_HPP_

#include "kimera_vio_ros/interfaces/image_interface.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

namespace kimera_vio_ros {
namespace interfaces {

class MonoInterface : virtual public ImageInterface {
public:
  MonoInterface(rclcpp::Node::SharedPtr &node);
  virtual ~MonoInterface();

private:
  void mono_info_cb(const CameraInfo::ConstSharedPtr &msg);
  void mono_image_cb(const Image::SharedPtr msg);

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_mono_;

  std::shared_ptr<message_filters::Subscriber<Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<CameraInfo>> info_sub_;
  bool camera_info_received_ = false;

  VIO::FrameId frame_count_;
  rclcpp::Time last_mono_timestamp_;
};

} // namespace interfaces
} // namespace kimera_vio_ros

#endif // KIMERA_ROS__INTERFACES__MONO_INTERFACE_HPP_
