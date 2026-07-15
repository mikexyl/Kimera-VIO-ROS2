#include <chrono>
#include <filesystem>
#include <optional>
#include <string>

#include "kimera_vio_ros/interfaces/RerunVisualizer.h"
#include "kimera_vio_ros/interfaces/base_interface.hpp"
#include <kimera-vio/common/DenseMapTypes.h>
#include <kimera-vio/common/MonoDepthTypes.h>
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

  // Model artifacts are deployment inputs. Non-empty ROS parameters override
  // the selected YAML profile; active pipelines validate their final paths.
  auto override_path = [this](const std::string &parameter,
                              std::string *target) {
    const auto value = node_->declare_parameter<std::string>(parameter, "");
    if (!value.empty()) {
      CHECK(std::filesystem::is_regular_file(value))
          << "Model parameter '" << parameter
          << "' does not point to a readable file: " << value;
      *target = value;
    }
  };
  override_path("models.xfeat",
                &vio_params_->frontend_params_.feature_detector_params_.xfeat_path_);
  override_path(
      "models.xfeat_interp_bilinear",
      &vio_params_->frontend_params_.feature_detector_params_.interp_bilinear_path_);
  override_path(
      "models.xfeat_interp_bicubic",
      &vio_params_->frontend_params_.feature_detector_params_.interp_bicubic_path_);
  override_path(
      "models.xfeat_interp_nearest",
      &vio_params_->frontend_params_.feature_detector_params_.interp_nearest_path_);
  override_path("models.lightglue_frontend",
                &vio_params_->frontend_params_.tracker_params_.lighterglue_model_path_);
  override_path("models.lightglue_lcd", &vio_params_->lcd_params_.lcd_lg_model_path_);
  const auto jist_model_path =
      node_->declare_parameter<std::string>("models.jist", "");
  if (!jist_model_path.empty()) {
    CHECK(vio_params_->lcd_params_.vpr_model_type_ ==
          VIO::VprModelType::kJist)
        << "models.jist was supplied for a non-JIST VPR profile";
    CHECK(std::filesystem::is_regular_file(jist_model_path))
        << "Model parameter 'models.jist' does not point to a readable file: "
        << jist_model_path;
    vio_params_->lcd_params_.vpr_model_path_ = jist_model_path;
  }
  // Determine if this is a mono or stereo setup based on number of cameras
  bool is_mono = vio_params_->frontend_type_ == VIO::FrontendType::kMonoImu;

  const auto rerun_recording_id_param =
      node_->declare_parameter<std::string>("rerun_recording_id", "");
  const auto rerun_application_id = node_->declare_parameter<std::string>(
      "rerun_application_id", "kimera_vio");
  std::optional<std::string> rerun_recording_id = std::nullopt;
  if (!rerun_recording_id_param.empty()) {
    rerun_recording_id = rerun_recording_id_param;
  }
  const auto rerun_result_dir =
      node_->declare_parameter<std::string>("rerun_result_dir", "");
  const auto rerun_host = node_->declare_parameter<std::string>(
      "rerun_host", "rerun+http://127.0.0.1:9876/proxy");
  const auto use_rerun_visualizer =
      node_->declare_parameter<bool>("use_rerun_visualizer", FLAGS_visualize);
  VIO::MonoDepthParams &mono_depth_params = vio_params_->mono_depth_params_;
  mono_depth_params.enabled =
      node_->declare_parameter<bool>("mono_depth.enabled", false);
  mono_depth_params.engine_path =
      node_->declare_parameter<std::string>("mono_depth.engine_path", "");
  mono_depth_params.mode = VIO::monoDepthModeFromString(
      node_->declare_parameter<std::string>("mono_depth.mode", "single_view"));
  mono_depth_params.da3_keyframe_selection_method =
      VIO::da3KeyframeSelectionMethodFromString(
          node_->declare_parameter<std::string>(
              "mono_depth.da3_keyframe_selection_method", "distance"));
  mono_depth_params.da3_keyframe_skip =
      node_->declare_parameter<int>("mono_depth.da3_keyframe_skip", 0);
  mono_depth_params.da3_keyframe_covisibility_threshold =
      node_->declare_parameter<double>(
          "mono_depth.da3_keyframe_covisibility_threshold", 0.5);
  mono_depth_params.min_keyframe_distance_m = node_->declare_parameter<double>(
      "mono_depth.min_keyframe_distance_m", 1.0);
  mono_depth_params.point_stride =
      node_->declare_parameter<int>("mono_depth.point_stride", 4);
  mono_depth_params.max_points_per_keyframe =
      node_->declare_parameter<int>("mono_depth.max_points_per_keyframe", 5000);
  mono_depth_params.visualization_point_stride = node_->declare_parameter<int>(
      "mono_depth.visualization_point_stride", mono_depth_params.point_stride);
  mono_depth_params.visualization_max_points_per_keyframe =
      node_->declare_parameter<int>(
          "mono_depth.visualization_max_points_per_keyframe",
          mono_depth_params.max_points_per_keyframe);
  mono_depth_params.min_depth_m =
      node_->declare_parameter<double>("mono_depth.min_depth_m", 0.1);
  mono_depth_params.max_depth_m =
      node_->declare_parameter<double>("mono_depth.max_depth_m", 30.0);
  mono_depth_params.depth_weighting_enabled = node_->declare_parameter<bool>(
      "mono_depth.depth_weighting_enabled", true);
  mono_depth_params.depth_weight_normal_radius =
      node_->declare_parameter<int>("mono_depth.depth_weight_normal_radius", 2);
  mono_depth_params.depth_weight_min =
      node_->declare_parameter<double>("mono_depth.depth_weight_min", 0.05);
  mono_depth_params.depth_weight_grazing_power =
      node_->declare_parameter<double>("mono_depth.depth_weight_grazing_power",
                                       1.0);
  mono_depth_params.depth_weight_range_ref = node_->declare_parameter<double>(
      "mono_depth.depth_weight_range_ref", 0.0);
  mono_depth_params.depth_weight_range_power = node_->declare_parameter<double>(
      "mono_depth.depth_weight_range_power", 2.0);
  mono_depth_params.depth_weight_range_min = node_->declare_parameter<double>(
      "mono_depth.depth_weight_range_min", 0.05);
  mono_depth_params.visualize_weights =
      node_->declare_parameter<bool>("mono_depth.visualize_weights", false);
  mono_depth_params.min_confidence =
      node_->declare_parameter<double>("mono_depth.min_confidence", 1.1);
  mono_depth_params.visualize_confidence =
      node_->declare_parameter<bool>("mono_depth.visualize_confidence", false);
  mono_depth_params.point_radius = static_cast<float>(
      node_->declare_parameter<double>("mono_depth.point_radius", 0.005));
  mono_depth_params.verbose =
      node_->declare_parameter<bool>("mono_depth.verbose", false);
  mono_depth_params.scale_alignment_method =
      VIO::monoDepthScaleAlignmentMethodFromString(
          node_->declare_parameter<std::string>(
              "mono_depth.scale_alignment_method", "none"));
  mono_depth_params.visualize_landmark_scale_alignment =
      node_->declare_parameter<bool>(
          "mono_depth.visualize_landmark_scale_alignment", false);
  mono_depth_params.da3_essential_factors_enabled =
      node_->declare_parameter<bool>("mono_depth.da3_essential_factors_enabled",
                                     false);
  mono_depth_params.da3_baseline_ratio_factors_enabled =
      node_->declare_parameter<bool>(
          "mono_depth.da3_baseline_ratio_factors_enabled", false);
  mono_depth_params.da3_baseline_ratio_log_sigma =
      node_->declare_parameter<double>(
          "mono_depth.da3_baseline_ratio_log_sigma", 0.25);
  mono_depth_params.icp_only_da3_overlap_fusion =
      node_->declare_parameter<bool>("mono_depth.icp_only_da3_overlap_fusion",
                                     false);
  mono_depth_params.landmark_scale_flatness_radius =
      node_->declare_parameter<int>("mono_depth.landmark_scale_flatness_radius",
                                    4);
  mono_depth_params.landmark_scale_max_relative_depth_variation =
      node_->declare_parameter<double>(
          "mono_depth.landmark_scale_max_relative_depth_variation", 0.15);
  VIO::validateMonoDepthScaleAlignmentConfiguration(
      mono_depth_params.mode, mono_depth_params.scale_alignment_method);
  VIO::BackendParams &backend_params = *vio_params_->backend_params_;
  backend_params.vgicp_icp_only_da3_overlap_fusion_ =
      mono_depth_params.icp_only_da3_overlap_fusion;
  CHECK(!backend_params.vgicp_icp_only_da3_overlap_fusion_ ||
        (backend_params.vgicp_factors_enabled_ &&
         backend_params.vgicp_icp_only_enabled_))
      << "mono_depth.icp_only_da3_overlap_fusion requires the isolated "
         "mono-depth diagnostic path to be enabled in BackendParams.yaml";
  VIO::DenseMapParams &dense_map_params = vio_params_->dense_map_params_;
  dense_map_params.enabled =
      node_->declare_parameter<bool>("dense_map.enabled", true);
  dense_map_params.backend =
      VIO::denseMapBackendFromString(node_->declare_parameter<std::string>(
          "dense_map.backend", "gaussian_voxel_map"));
  dense_map_params.voxel_resolution =
      node_->declare_parameter<double>("dense_map.voxel_resolution", 0.15);
  dense_map_params.point_radius = static_cast<float>(
      node_->declare_parameter<double>("dense_map.point_radius", 0.025));
  VIO::Visualizer3D::UniquePtr rerun_visualizer;
  if (use_rerun_visualizer) {
    rerun_visualizer = std::make_unique<VIO::RerunVisualizer>(
        VIO::RerunVisualizer::Params{.application_id = rerun_application_id,
                                     .base_link_frame_id = base_link_frame_id_,
                                     .odom_frame_id = odom_frame_id_,
                                     .map_frame_id = map_frame_id_,
                                     .recording_id = rerun_recording_id,
                                     .result_dir = rerun_result_dir,
                                     .rerun_host = rerun_host});
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
    CHECK(std::filesystem::is_regular_file(
        vio_params_->lcd_params_.vpr_model_path_))
        << "The configured VPR model does not point to a readable file: "
        << vio_params_->lcd_params_.vpr_model_path_;
    local_lcd_publisher_ = std::make_unique<LocalLoopClosurePublisher>(node_);
    multi_robot_bridge_ =
        std::make_unique<MultiRobotLoopClosureBridge>(node_);
    vio_pipeline_->registerLcdOutputCallback(
        [this](const VIO::LcdOutput::Ptr &msg) {
          CHECK_NOTNULL(local_lcd_publisher_.get())->publishLcdOutput(msg);
          CHECK_NOTNULL(multi_robot_bridge_.get())->publishLcdOutput(msg);
        });
  }
}

BaseInterface::~BaseInterface() {
  vio_pipeline_->shutdown();
  if (vio_params_->parallel_run_) {
    handle_pipeline_.get();
  }
  if (multi_robot_bridge_) {
    multi_robot_bridge_->flush();
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
