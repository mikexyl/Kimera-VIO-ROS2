#include <chrono>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

#include "kimera_vio_ros/interfaces/RerunVisualizer.h"
#include "kimera_vio_ros/interfaces/base_interface.hpp"
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
  const bool use_external_odom = node_->declare_parameter(
      "use_external_odom", FLAGS_use_external_odometry);
  CHECK_EQ(use_external_odom, FLAGS_use_external_odometry)
      << "The use_external_odom ROS parameter and "
         "--use_external_odometry flag must match";

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
      CHECK_EQ(std::filesystem::path(value).extension(), ".engine")
          << "Model parameter '" << parameter
          << "' requires a TensorRT .engine file: " << value;
      *target = value;
    }
  };
  override_path("models.xfeat",
                &vio_params_->frontend_params_.feature_detector_params_.xfeat_path_);
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
    CHECK_EQ(std::filesystem::path(jist_model_path).extension(), ".engine")
        << "Model parameter 'models.jist' requires a TensorRT .engine file: "
        << jist_model_path;
    vio_params_->lcd_params_.vpr_model_path_ = jist_model_path;
  }
  const auto mixvpr_model_path =
      node_->declare_parameter<std::string>("models.mixvpr", "");
  if (!mixvpr_model_path.empty()) {
    CHECK(vio_params_->lcd_params_.vpr_model_type_ ==
          VIO::VprModelType::kMixVPR)
        << "models.mixvpr was supplied for a non-MixVPR VPR profile";
    CHECK(std::filesystem::is_regular_file(mixvpr_model_path))
        << "Model parameter 'models.mixvpr' does not point to a readable file: "
        << mixvpr_model_path;
    CHECK_EQ(std::filesystem::path(mixvpr_model_path).extension(), ".engine")
        << "Model parameter 'models.mixvpr' requires a TensorRT .engine file: "
        << mixvpr_model_path;
    vio_params_->lcd_params_.vpr_model_path_ = mixvpr_model_path;
  }
  auto &dense_stereo_params =
      vio_params_->frontend_params_.stereo_matching_params_.dense_stereo_params_;
  const auto stereo_depth_method =
      node_->declare_parameter<std::string>("stereo_depth.method", "");
  if (!stereo_depth_method.empty()) {
    dense_stereo_params.stereo_depth_method_ =
        VIO::stereoDepthMethodFromString(stereo_depth_method);
  }
  const auto stereo_depth_model =
      node_->declare_parameter<std::string>("models.stereo_depth", "");
  if (!stereo_depth_model.empty()) {
    CHECK(dense_stereo_params.stereo_depth_method_ ==
              VIO::StereoDepthMethod::LIGHTSTEREO ||
          dense_stereo_params.stereo_depth_method_ ==
              VIO::StereoDepthMethod::FAST_FOUNDATION_STEREO)
        << "models.stereo_depth is only valid for a TensorRT stereo method";
    CHECK(std::filesystem::is_regular_file(stereo_depth_model))
        << "Model parameter 'models.stereo_depth' does not point to a "
           "readable file: "
        << stereo_depth_model;
    CHECK_EQ(std::filesystem::path(stereo_depth_model).extension(), ".engine")
        << "Model parameter 'models.stereo_depth' requires a TensorRT "
           ".engine file: "
        << stereo_depth_model;
    dense_stereo_params.engine_path_ = stereo_depth_model;
  }
  if (dense_stereo_params.stereo_depth_method_ ==
          VIO::StereoDepthMethod::LIGHTSTEREO ||
      dense_stereo_params.stereo_depth_method_ ==
          VIO::StereoDepthMethod::FAST_FOUNDATION_STEREO) {
    CHECK(std::filesystem::is_regular_file(dense_stereo_params.engine_path_))
        << VIO::stereoDepthMethodToString(
               dense_stereo_params.stereo_depth_method_)
        << " requires a readable TensorRT engine: "
        << dense_stereo_params.engine_path_;
    CHECK_EQ(std::filesystem::path(dense_stereo_params.engine_path_).extension(),
             ".engine");
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
  const auto rerun_visualization_profile_name =
      node_->declare_parameter<std::string>("rerun_visualization_profile",
                                           "full");
  VIO::RerunVisualizer::VisualizationProfile rerun_visualization_profile =
      VIO::RerunVisualizer::VisualizationProfile::kFull;
  if (rerun_visualization_profile_name == "full") {
    rerun_visualization_profile =
        VIO::RerunVisualizer::VisualizationProfile::kFull;
  } else if (rerun_visualization_profile_name == "minimal") {
    rerun_visualization_profile =
        VIO::RerunVisualizer::VisualizationProfile::kMinimal;
  } else {
    LOG(FATAL) << "rerun_visualization_profile must be 'full' or 'minimal', "
                  "got: "
               << rerun_visualization_profile_name;
  }
  const auto rerun_tracking_image_jpeg_quality =
      node_->declare_parameter<int>("rerun_tracking_image_jpeg_quality", 80);
  CHECK_GE(rerun_tracking_image_jpeg_quality, 1);
  CHECK_LE(rerun_tracking_image_jpeg_quality, 100);
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
  mono_depth_params.min_confidence =
      node_->declare_parameter<double>("mono_depth.min_confidence", 1.1);
  mono_depth_params.visualize_confidence =
      node_->declare_parameter<bool>("mono_depth.visualize_confidence", false);
  mono_depth_params.verbose =
      node_->declare_parameter<bool>("mono_depth.verbose", false);
  mono_depth_params.scale_alignment_method =
      VIO::monoDepthScaleAlignmentMethodFromString(
          node_->declare_parameter<std::string>(
              "mono_depth.scale_alignment_method", "none"));
  mono_depth_params.da3_essential_factors_enabled =
      node_->declare_parameter<bool>("mono_depth.da3_essential_factors_enabled",
                                     false);
  mono_depth_params.da3_baseline_ratio_factors_enabled =
      node_->declare_parameter<bool>(
          "mono_depth.da3_baseline_ratio_factors_enabled", false);
  mono_depth_params.da3_baseline_ratio_log_sigma =
      node_->declare_parameter<double>(
          "mono_depth.da3_baseline_ratio_log_sigma", 0.25);
  mono_depth_params.landmark_scale_flatness_radius =
      node_->declare_parameter<int>("mono_depth.landmark_scale_flatness_radius",
                                    4);
  mono_depth_params.landmark_scale_max_relative_depth_variation =
      node_->declare_parameter<double>(
          "mono_depth.landmark_scale_max_relative_depth_variation", 0.15);
  VIO::validateMonoDepthScaleAlignmentConfiguration(
      mono_depth_params.mode, mono_depth_params.scale_alignment_method);
  VIO::Visualizer3D::UniquePtr rerun_visualizer;
  if (use_rerun_visualizer) {
    rerun_visualizer = std::make_unique<VIO::RerunVisualizer>(
        VIO::RerunVisualizer::Params{.application_id = rerun_application_id,
                                     .base_link_frame_id = base_link_frame_id_,
                                     .odom_frame_id = odom_frame_id_,
                                     .map_frame_id = map_frame_id_,
                                     .recording_id = rerun_recording_id,
                                     .result_dir = rerun_result_dir,
                                     .rerun_host = rerun_host,
                                     .visualization_profile =
                                         rerun_visualization_profile,
                                     .tracking_image_jpeg_quality =
                                         static_cast<int>(
                                             rerun_tracking_image_jpeg_quality)});
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

  if (use_external_odom) {
    registerExternalOdomCallback(
        std::bind(&VIO::Pipeline::fillExternalOdomQueue, vio_pipeline_.get(),
                  std::placeholders::_1));
    external_odometry_subscriber_ =
        node_->create_subscription<nav_msgs::msg::Odometry>(
            "external_odom", rclcpp::QoS(1000),
            std::bind(&BaseInterface::externalOdometryCallback, this,
                      std::placeholders::_1));
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

void BaseInterface::externalOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry) {
  CHECK(odometry);
  CHECK(external_odom_callback_);

  const auto &message_pose = odometry->pose.pose;
  const gtsam::Pose3 world_pose_body(
      gtsam::Rot3::Quaternion(message_pose.orientation.w,
                              message_pose.orientation.x,
                              message_pose.orientation.y,
                              message_pose.orientation.z),
      gtsam::Point3(message_pose.position.x, message_pose.position.y,
                    message_pose.position.z));

  // ROS odometry expresses linear velocity in the child/body frame. Kimera's
  // NavState expects it in the world frame.
  const auto &linear_velocity = odometry->twist.twist.linear;
  const gtsam::Vector3 body_velocity(linear_velocity.x, linear_velocity.y,
                                    linear_velocity.z);
  const gtsam::Vector3 world_velocity =
      world_pose_body.rotation() * body_velocity;

  external_odom_callback_(VIO::ExternalOdomMeasurement(
      rclcpp::Time(odometry->header.stamp).nanoseconds(),
      gtsam::NavState(world_pose_body, world_velocity)));
}

} // namespace interfaces
} // namespace kimera_vio_ros
