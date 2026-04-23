#include <chrono>

#include "kimera_vio_ros/interfaces/RerunVisualizer.h"
#include "kimera_vio_ros/interfaces/base_interface.hpp"

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
  map_frame_id_ = node_->declare_parameter("frame_id.map", "map");
  world_frame_id_ = node_->declare_parameter("frame_id.world", "world");

  std::string params_folder_;
  params_folder_ = node_->declare_parameter("params_folder", "");
  CHECK(!params_folder_.empty());
  vio_params_ = std::make_shared<VIO::VioParams>(params_folder_);
  if (vio_params_->backend_params_ &&
      vio_params_->backend_params_->autoInitialize_ == 0 &&
      vio_params_->backend_params_->initial_ground_truth_state_.equals(
          VIO::VioNavState())) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Backend requested ground-truth initialization but no initial "
        "ground-truth state was loaded; falling back to IMU auto-initialization.");
    vio_params_->backend_params_->autoInitialize_ = 1;
  }

  vio_params_->camera_params_[0].print();
  vio_params_->camera_params_[1].print();

  auto rerun_visualizer = std::make_unique<VIO::RerunVisualizer>(
      VIO::RerunVisualizer::Params{
          .result_dir = "/tmp/deslam",
          .node = node_,
      });

  rerun_visualizer_module_ = std::make_unique<VIO::VisualizerModule>(
      nullptr, vio_params_->parallel_run_, std::move(rerun_visualizer));

  vio_pipeline_.reset();
  vio_pipeline_ = std::make_shared<VIO::Pipeline>(*vio_params_);
  vio_pipeline_->registerBackendOutputCallback(
      std::bind(&VIO::VisualizerModule::fillBackendQueue,
                std::ref(*rerun_visualizer_module_), std::placeholders::_1));
  vio_pipeline_->registerFrontendOutputCallback(
      std::bind(&VIO::VisualizerModule::fillFrontendQueue,
                std::ref(*rerun_visualizer_module_), std::placeholders::_1));
  vio_pipeline_->registerMesherOutputCallback(
      std::bind(&VIO::VisualizerModule::fillMesherQueue,
                std::ref(*rerun_visualizer_module_), std::placeholders::_1));
}

BaseInterface::~BaseInterface() {
  if (rerun_visualizer_module_) {
    rerun_visualizer_module_->shutdown();
  }
  if (vio_pipeline_) {
    vio_pipeline_->shutdown();
  }
  if (vio_params_->parallel_run_) {
    if (handle_rerun_visualizer_.valid()) {
      handle_rerun_visualizer_.get();
    }
    if (handle_pipeline_.valid()) {
      handle_pipeline_.get();
    }
  }
}

void BaseInterface::start() {
  if (vio_params_->parallel_run_) {
    handle_rerun_visualizer_ =
        std::async(std::launch::async, &VIO::VisualizerModule::spin,
                   rerun_visualizer_module_.get());
    handle_pipeline_ = std::async(std::launch::async, &VIO::Pipeline::spin,
                                  vio_pipeline_.get());
  } else {
    pipeline_timer_ = node_->create_wall_timer(
        10ms,
        [this]() {
          vio_pipeline_->spin();
          rerun_visualizer_module_->spin();
        },
        callback_group_pipeline_);
  }
}

} // namespace interfaces
} // namespace kimera_vio_ros
