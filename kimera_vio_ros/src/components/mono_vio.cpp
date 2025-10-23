#include "kimera_vio_ros/components/mono_vio.hpp"

namespace kimera_vio_ros
{
namespace components
{

MonoVio::MonoVio(
  const rclcpp::NodeOptions & node_options)
: Node("mono_vio", node_options)
{
  init();
}

MonoVio::MonoVio(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & node_options)
: Node(node_name, ns, node_options)
{
  init();
}

MonoVio::~MonoVio()
{
}

void MonoVio::init()
{
  rclcpp::Node::SharedPtr node = std::shared_ptr<rclcpp::Node>(this);
  vio_node_ = std::make_unique<interfaces::MonoVioInterface>(node);
}

}  // namespace components
}  // namespace kimera_vio_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kimera_vio_ros::components::MonoVio)
