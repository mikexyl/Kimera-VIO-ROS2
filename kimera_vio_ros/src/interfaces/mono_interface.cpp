#include "kimera_vio_ros/interfaces/mono_interface.hpp"
#include <glog/logging.h>
#include <kimera-vio/pipeline/MonoImuPipeline.h>

namespace kimera_vio_ros {
namespace interfaces {

MonoInterface::MonoInterface(rclcpp::Node::SharedPtr &node)
    : BaseInterface(node), ImageInterface(node), frame_count_(VIO::FrameId(0)),
      last_mono_timestamp_(0) {
  this->registerLeftFrameCallback(std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                                            vio_pipeline_.get(),
                                            std::placeholders::_1));

  auto mono_pipeline =
      std::dynamic_pointer_cast<VIO::MonoImuPipeline>(vio_pipeline_);
  CHECK(mono_pipeline != nullptr)
      << "vio_pipeline_ was not correctly initialized as a MonoImuPipeline";

  callback_group_mono_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto mono_opt = rclcpp::SubscriptionOptions();
  mono_opt.callback_group = callback_group_mono_;

  int queue_size_ = 10;

  auto info_qos = rclcpp::SystemDefaultsQoS();
  std::string info_topic = "camera_info";
  info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
      node_, info_topic, info_qos.get_rmw_qos_profile());
  info_sub_->registerCallback(&MonoInterface::mono_info_cb, this);

  auto image_qos = rclcpp::SensorDataQoS();
  std::string image_topic = "image";
  
  // TODO: Assign message filter subscribers to callback_group_mono_
  // Pending: https://github.com/ros2/message_filters/issues/45
  image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
      node_, image_topic, image_qos.get_rmw_qos_profile());
  image_sub_->registerCallback(&MonoInterface::mono_image_cb, this);

  bool use_camera_info_;
  use_camera_info_ = node_->declare_parameter("use_camera_info", true);

  CHECK(not use_camera_info_) << "disabled";

  RCLCPP_INFO(
      node_->get_logger(),
      "Using YAML parameter files instead of online camera parameters.");
  camera_info_received_ = true;
  info_sub_->unsubscribe();
}

MonoInterface::~MonoInterface() {}

void MonoInterface::mono_info_cb(
    const CameraInfo::ConstSharedPtr &msg) {
  CHECK_GE(vio_params_->camera_params_.size(), 1u);

  // Initialize CameraParams for pipeline.
  msgCamInfoToCameraParams(msg, &vio_params_->camera_params_.at(0));

  vio_params_->camera_params_.at(0).print();

  // Signal the correct reception of camera info
  camera_info_received_ = true;
  if (camera_info_received_) {
    RCLCPP_INFO(node_->get_logger(),
                "Switching subscriptions from CameraInfo to Image after "
                "receiving camera parameters.");
    // Unregister info callback as it is no longer needed.
    info_sub_->unsubscribe();
    // Reregister image callback as it is now ready.
    image_sub_->subscribe();
  }
}

void MonoInterface::mono_image_cb(const Image::SharedPtr msg) {
  rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() > last_mono_timestamp_.nanoseconds()) {
    static const VIO::CameraParams &cam_info =
        vio_params_->camera_params_.at(0);

    const VIO::Timestamp &timestamp = stamp.nanoseconds();

    auto img = readRosImage(msg);

    left_frame_callback_(std::make_unique<VIO::Frame>(
        frame_count_, timestamp, cam_info, img));
    // LOG_EVERY_N(INFO, 30) << "Done: KimeraVioNode::mono_image_cb";
    frame_count_++;
  }
  last_mono_timestamp_ = stamp;
}

} // namespace interfaces
} // namespace kimera_vio_ros
