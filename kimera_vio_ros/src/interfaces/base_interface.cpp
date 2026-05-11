#include <chrono>
#include <optional>
#include <string>

#include "kimera_vio_ros/interfaces/RerunVisualizer.h"
#include "kimera_vio_ros/interfaces/base_interface.hpp"
#include <kimera-vio/pipeline/MonoImuPipeline.h>
#include <kimera-vio/pipeline/StereoImuPipeline.h>

using namespace std::chrono_literals;

namespace kimera_vio_ros {
namespace interfaces {

BaseInterface::BaseInterface(rclcpp::Node::SharedPtr &node)
    : VIO::DataProviderInterface(), node_(node), vio_params_(nullptr),
      vio_pipeline_(nullptr), tf_buffer_(nullptr), tf_broadcaster_(nullptr),
      tf_listener_(nullptr) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  callback_group_pipeline_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  base_link_frame_id_ =
      node_->declare_parameter("frame_id.base_link", "base_link");
  odom_frame_id_ = node_->declare_parameter("frame_id.odom", "odom");
  map_frame_id_ = node_->declare_parameter("frame_id.map", "map");
  world_frame_id_ = node_->declare_parameter("frame_id.world", "world");

  std::string params_folder_;
  params_folder_ = node_->declare_parameter("params_folder", "");
  CHECK(!params_folder_.empty());
  vio_params_ = std::make_shared<VIO::VioParams>(params_folder_);

  // Determine if this is a mono or stereo setup based on number of cameras
  bool is_mono = vio_params_->frontend_type_ == VIO::FrontendType::kMonoImu;

  const auto rerun_recording_id_param =
      node_->declare_parameter<std::string>("rerun_recording_id", "");
  std::optional<std::string> rerun_recording_id = std::nullopt;
  if (!rerun_recording_id_param.empty()) {
    rerun_recording_id = rerun_recording_id_param;
  }
  const auto rerun_result_dir =
      node_->declare_parameter<std::string>("rerun_result_dir", "");
  VIO::Visualizer3D::UniquePtr rerun_visualizer;
  if (FLAGS_visualize) {
    rerun_visualizer = std::make_unique<VIO::RerunVisualizer>(
        VIO::RerunVisualizer::Params{
            .base_link_frame_id = base_link_frame_id_,
            .odom_frame_id = odom_frame_id_,
            .map_frame_id = map_frame_id_,
            .recording_id = rerun_recording_id,
            .result_dir = rerun_result_dir});
  }

  vio_pipeline_.reset();
  if (is_mono) {
    RCLCPP_INFO(node_->get_logger(), "Initializing Mono VIO Pipeline");
    vio_pipeline_ = std::make_shared<VIO::MonoImuPipeline>(
        *vio_params_, std::move(rerun_visualizer), nullptr);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Initializing Stereo VIO Pipeline");
    vio_params_->camera_params_[1].print();
    vio_pipeline_ = std::make_shared<VIO::StereoImuPipeline>(
        *vio_params_, std::move(rerun_visualizer), nullptr);
  }

  if (FLAGS_use_lcd != 0) {
    ros_lcd_visualizer_ =
        std::make_unique<RosLoopClosureVisualizer>(node_);
    vio_pipeline_->registerLcdOutputCallback(
        [this](const VIO::LcdOutput::Ptr& msg) {
          CHECK_NOTNULL(ros_lcd_visualizer_.get())->publishLcdOutput(msg);
        });
  }
}

BaseInterface::~BaseInterface() {
  vio_pipeline_->shutdown();
  if (vio_params_->parallel_run_) {
    handle_pipeline_.get();
  }
}

void BaseInterface::start() {
  if (vio_params_->parallel_run_) {
    handle_pipeline_ = std::async(std::launch::async, &VIO::Pipeline::spin,
                                  vio_pipeline_.get());
  } else {
    pipeline_timer_ = node_->create_wall_timer(
        10ms, std::bind(&VIO::Pipeline::spin, vio_pipeline_.get()),
        callback_group_pipeline_);
  }
}

} // namespace interfaces
} // namespace kimera_vio_ros
