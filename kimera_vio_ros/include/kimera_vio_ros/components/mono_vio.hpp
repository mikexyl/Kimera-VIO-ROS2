#ifndef KIMERA_ROS__COMPONENTS__MONO_VIO_HPP_
#define KIMERA_ROS__COMPONENTS__MONO_VIO_HPP_

#include "kimera_vio_ros/interfaces/mono_vio_interface.hpp"

namespace kimera_vio_ros
{
namespace components
{

class MonoVio : public rclcpp::Node
{
public:
  MonoVio(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  MonoVio(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~MonoVio();

private:
  void init();

  std::unique_ptr<interfaces::BaseInterface> vio_node_;
};

}  // namespace components
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__COMPONENTS__MONO_VIO_HPP_
