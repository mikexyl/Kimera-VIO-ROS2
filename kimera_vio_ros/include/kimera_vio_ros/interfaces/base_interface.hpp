#ifndef KIMERA_ROS__INTERFACES__BASE_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__BASE_INTERFACE_HPP_

#include <future>
#include <memory>

#include "glog/logging.h"
#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera_vio_ros/interfaces/local_loop_closure_publisher.hpp"
#include "kimera_vio_ros/interfaces/multi_robot_loop_closure_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace kimera_vio_ros
{
namespace interfaces
{

class BaseInterface : public VIO::DataProviderInterface
{
public:
  BaseInterface(
    rclcpp::Node::SharedPtr & node);
  virtual ~BaseInterface();
  void start();

protected:
  rclcpp::Node::SharedPtr node_;
  VIO::VioParams::Ptr vio_params_;
  VIO::Pipeline::Ptr vio_pipeline_;
  std::unique_ptr<LocalLoopClosurePublisher> local_lcd_publisher_;
  std::unique_ptr<MultiRobotLoopClosureBridge> multi_robot_bridge_;

  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  std::string world_frame_id_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_pipeline_;
  rclcpp::TimerBase::SharedPtr pipeline_timer_;
  std::future<bool> handle_pipeline_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__BASE_INTERFACE_HPP_
