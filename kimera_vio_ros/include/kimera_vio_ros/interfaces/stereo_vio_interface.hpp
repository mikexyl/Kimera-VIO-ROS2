#ifndef KIMERA_ROS__INTERFACES__STEREO_VIO_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__STEREO_VIO_INTERFACE_HPP_

#include <map>
#include <mutex>

#include "kimera_vio_ros/interfaces/backend_interface.hpp"
#include "kimera_vio_ros/interfaces/imu_interface.hpp"
#include "kimera_vio_ros/interfaces/stereo_interface.hpp"
#include "projective_mesher_msgs/msg/mesher_input.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

class StereoVioInterface
  : public ImuInterface,
  public StereoInterface,
  public BackendInterface
{
public:
  StereoVioInterface(
    rclcpp::Node::SharedPtr & node);
  ~StereoVioInterface();

private:
  using MesherInputMsg = projective_mesher_msgs::msg::MesherInput;

  void callbackMesherFrontendOutput(const VIO::FrontendOutput::Ptr & output);
  void callbackMesherBackendOutput(const VIO::BackendOutput::Ptr & output);
  void publishMesherInput(
    const VIO::FrontendOutput::Ptr & frontend_output,
    const VIO::BackendOutput::Ptr & backend_output) const;
  bool collectSyncedMesherInput(
    const VIO::Timestamp & timestamp,
    VIO::FrontendOutput::Ptr * frontend_output,
    VIO::BackendOutput::Ptr * backend_output);
  void pruneMesherInputQueues();

  rclcpp::Publisher<MesherInputMsg>::SharedPtr mesher_input_pub_;
  std::mutex mesher_input_mutex_;
  std::map<VIO::Timestamp, VIO::FrontendOutput::Ptr> mesher_frontend_outputs_;
  std::map<VIO::Timestamp, VIO::BackendOutput::Ptr> mesher_backend_outputs_;
  size_t mesher_input_queue_size_{30};
  bool publish_mesher_input_{true};
};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__STEREO_VIO_INTERFACE_HPP_
