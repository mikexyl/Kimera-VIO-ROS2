#include "kimera_vio_ros/interfaces/mono_vio_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

MonoVioInterface::MonoVioInterface(
  rclcpp::Node::SharedPtr & node)
: BaseInterface(node),
  ImageInterface(node),
  ImuInterface(node),
  MonoInterface(node),
  BackendInterface(node)
{
  BaseInterface::start();
}

MonoVioInterface::~MonoVioInterface()
{
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
