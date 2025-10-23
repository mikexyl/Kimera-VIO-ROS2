#ifndef KIMERA_ROS__INTERFACES__MONO_VIO_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__MONO_VIO_INTERFACE_HPP_

#include "kimera_vio_ros/interfaces/backend_interface.hpp"
#include "kimera_vio_ros/interfaces/imu_interface.hpp"
#include "kimera_vio_ros/interfaces/mono_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

class MonoVioInterface
  : public ImuInterface,
  public MonoInterface,
  public BackendInterface
{
public:
  MonoVioInterface(
    rclcpp::Node::SharedPtr & node);
  ~MonoVioInterface();

};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__MONO_VIO_INTERFACE_HPP_
