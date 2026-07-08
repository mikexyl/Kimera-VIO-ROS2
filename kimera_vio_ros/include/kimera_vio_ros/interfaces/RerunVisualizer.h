#pragma once

#include <aria_viz/visualizer_rerun.h>
#include <glog/logging.h>
#include <gtsam/slam/dataset.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/fmt/fmt.h>
#include <xfeat-cpp/mono_depth/mono_depth.h>

#ifdef HAVE_TENSORRT
#include <xfeat-cpp/mono_depth/depth_anything_v3_trt.h>
#endif

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <future>
#include <limits>
#include <map>
#include <optional>
#include <utility>

namespace VIO {

// Alias for the custom log handler signature
using GlogHandler =
    std::function<void(google::LogSeverity severity, const char *filename,
                       int line, const char *message)>;

// Custom sink that forwards glog messages to the user-provided handler
class CustomLogSink : public google::LogSink {
public:
  explicit CustomLogSink(GlogHandler handler) : handler_(std::move(handler)) {}

  void send(google::LogSeverity severity, const char *full_filename,
            const char *base_filename, int line, const struct tm *tm_time,
            const char *message, size_t message_len) override {
    // Build message string and forward to custom handler
    std::string msg(message, message_len);
    handler_(severity, base_filename, line, msg.c_str());
  }

private:
  GlogHandler handler_;
};

// -----------------------------------------------------------------------------
// Redirect std::cout to glog at INFO level by installing a custom streambuf
// -----------------------------------------------------------------------------
class GlogStreamBuf : public std::streambuf {
public:
  GlogStreamBuf() { setp(buffer_, buffer_ + sizeof(buffer_) - 1); }

protected:
  int_type overflow(int_type ch) override {
    if (ch != traits_type::eof()) {
      *pptr() = static_cast<char>(ch);
      pbump(1);
    }
    if (ch == '\n' || pptr() >= epptr()) {
      flushBuffer();
    }
    return ch;
  }

  int sync() override {
    flushBuffer();
    return 0;
  }

private:
  void flushBuffer() {
    std::ptrdiff_t len = pptr() - pbase();
    if (len <= 0)
      return;
    std::string msg(pbase(), len);
    LOG(INFO) << msg;
    pbump(-len);
  }

  char buffer_[1024];
};

class RerunVisualizer : public Visualizer3D, aria::viz::VisualizerRerun {
public:
  struct MonoDepthParams {
    bool enabled = false;
    std::string engine_path;
    int device_id = 0;
    int point_stride = 4;
    int max_points_per_keyframe = 5000;
    int max_map_points = 200000;
    double min_depth_m = 0.1;
    double max_depth_m = 30.0;
    float point_radius = 0.005f;
    bool verbose = false;
  };

  struct CachedMonoDepthFrame {
    cv::Mat image;
    CameraParams::Intrinsics intrinsics{};
    gtsam::Pose3 body_T_cam;
    KeypointsCV keypoints;
    LandmarkIds landmarks;
  };

  struct Params {
    std::string base_link_frame_id = "baselink";
    std::string odom_frame_id = "odom";
    std::string map_frame_id = "map";
    std::string gt_csv_file = "";
    std::optional<std::string> recording_id = std::nullopt;
    std::string result_dir = "";
    std::string rerun_host = "rerun+http://127.0.0.1:9876/proxy";
    MonoDepthParams mono_depth;
  };

  RerunVisualizer(const Params &params)
      : RerunVisualizer(params.base_link_frame_id, params.odom_frame_id,
                        params.map_frame_id, params.gt_csv_file,
                        params.recording_id, params.result_dir,
                        params.rerun_host, params.mono_depth) {}

  RerunVisualizer(std::string base_link_frame_id,
                  std::string odom_frame_id,
                  std::string map_frame_id,
                  std::string gt_csv_file,
                  std::optional<std::string> recording_id,
                  std::string result_dir,
                  std::string rerun_host,
                  MonoDepthParams mono_depth_params)
      : VIO::Visualizer3D(VIO::VisualizationType::kNone),
        aria::viz::VisualizerRerun(aria::viz::VisualizerRerun::Params(
            "kimera_vio", recording_id, rerun_host)),
        baselink_(base_link_frame_id), map_(map_frame_id), odom_(odom_frame_id),
        result_dir_(result_dir), mono_depth_params_(std::move(mono_depth_params)) {
    // draw the origin frame for visualization
    this->drawTf(map_, Pose3::Identity(), 0.3, true);

    if (not g_custom_sink) {
      AddGlogCustomSink([this](google::LogSeverity severity,
                               const char *filename, int line,
                               const char *message) {
        logGlogMessages(severity, filename, line, message);
      });
      RedirectStdCoutToGlog();
    }

    auto landmark_color = aria::viz::ColorMap::kGray;
    landmark_color[3] = 50;

    if (not gt_csv_file.empty()) {
      gt_trajectory_ = loadTrajectoryMapFromCSV(gt_csv_file);
      std::vector<gtsam::Pose3> gt_traj;
      for (const auto &[key, pose] : gt_trajectory_) {
        gt_traj.push_back(pose);
      }
      this->drawTf(map_ / "gt", T_map_gt_, 1.0, false);
      this->drawTrajectory(map_ / "gt" / "trajectory", gt_traj, landmark_color,
                           1.f, false);
    }

    if (not result_dir_.empty()) {
      // check if folder exists
      if (not std::filesystem::exists(result_dir_)) {
        LOG(FATAL) << "Result directory does not exist: " << result_dir_;
      }
      LOG(INFO) << "RerunVisualizer result directory: " << result_dir_;
    } else {
      LOG(INFO) << "RerunVisualizer result disabled";
    }

    if (mono_depth_params_.enabled) {
      LOG(INFO) << "Mono depth mapping enabled with engine: "
                << mono_depth_params_.engine_path;
    }

    // Initialize timer for trajectory saving
    last_save_time_ = std::chrono::steady_clock::now();
  }

  virtual ~RerunVisualizer() = default;

  // Hold the active custom sink so it persists for the program lifetime.
  static std::unique_ptr<CustomLogSink> g_custom_sink;

  // Call this after google::InitGoogleLogging(), to attach your custom handler
  // in addition to glog's default sinks (stderr and/or log files).
  inline void AddGlogCustomSink(GlogHandler handler) {
    // Remove previous custom sink if installed
    if (g_custom_sink) {
      google::RemoveLogSink(g_custom_sink.get());
      g_custom_sink.reset();
    }

    // Create and install a new sink; default sinks remain active
    g_custom_sink = std::make_unique<CustomLogSink>(std::move(handler));
    google::AddLogSink(g_custom_sink.get());
  }

  // Preserve original buffer so we can restore cout
  static std::streambuf *g_original_cout_buf_;
  static GlogStreamBuf g_glog_streambuf_;

  // Call after InitGoogleLogging() to capture std::cout output
  inline void RedirectStdCoutToGlog() {
    if (!g_original_cout_buf_) {
      g_original_cout_buf_ = std::cout.rdbuf(&g_glog_streambuf_);
    }
  }

  // Restore original std::cout behavior
  inline void RestoreStdCout() {
    if (g_original_cout_buf_) {
      std::cout.rdbuf(g_original_cout_buf_);
      g_original_cout_buf_ = nullptr;
    }
  }

  // Optional: remove the custom sink
  inline void RemoveGlogCustomSink() {
    if (g_custom_sink) {
      google::RemoveLogSink(g_custom_sink.get());
      g_custom_sink.reset();
    }
  }

  void logGlogMessages(google::LogSeverity severity, const char *filename,
                       int line, const char *message) {
    // glog severity to Rerun log level
    rerun::TextLogLevel level;
    switch (severity) {
    case google::GLOG_INFO:
      level = rerun::TextLogLevel::Info;
      break;
    case google::GLOG_WARNING:
      level = rerun::TextLogLevel::Warning;
      break;
    case google::GLOG_ERROR:
      level = rerun::TextLogLevel::Error;
      break;
    case google::GLOG_FATAL:
      level = rerun::TextLogLevel::Critical;
      break;
    default:
      level = rerun::TextLogLevel::Debug; // Default to Debug for other
                                          // severities
    }

    // Forward glog messages to Rerun
    this->rec()->log(
        "glog", rerun::TextLog(fmt::format("{}", message)).with_level(level));
  }

  VIO::VisualizerOutput::UniquePtr
  spinOnce(const VIO::VisualizerInput &input) override {
    std::lock_guard<std::mutex> lock(rerun_mutex_);
    this->setTimeNSec(input.timestamp_);
    this->drawTf(map_ / odom_ / baselink_,
                 input.backend_output_->W_State_Blkf_.pose_, 1.0, false);

    LOG(INFO) << "Backend output timestamp: " << input.timestamp_;

    odom_traj_.push_back(input.backend_output_->W_State_Blkf_.pose_);
    odom_states_.insert(input.backend_output_->cur_kf_id_,
                        input.backend_output_->W_State_Blkf_.pose_);
    timestamp_map_.insert(
        {input.backend_output_->cur_kf_id_, input.timestamp_});
    this->drawTrajectory(map_ / odom_ / "trajectory", odom_traj_,
                         aria::viz::ColorMap::kGreen, 1.f, false);

    // auto cur_cov = input.backend_output_->state_covariance_lkf_;
    // this->drawUncertainty(map_ / odom_ / baselink_ / "covariance",
    //                       Pose3::Identity(), cur_cov.block<3, 3>(0, 0),
    //                       aria::viz::ColorMap::kGreen, 0.1);

    drawCameraEntity(input);

    if (not input.frontend_output_->getTrackingImage()->empty()) {
      cv::Mat tracking_image_clone =
          input.frontend_output_->getTrackingImage()->clone();
      cv::Mat small_image;
      cv::resize(tracking_image_clone, small_image, cv::Size(), 0.5, 0.5);
      this->drawImage(map_ / odom_ / baselink_ / "tracking" / "image",
                      small_image, false);
    }

    updateMonoDepthMap(input);

    Landmarks lmks_vec;
    // convert landmark id->landmark map to vector
    for (const auto &[id, landmark] :
         input.backend_output_->landmarks_in_local_window_) {
      lmks_vec.push_back(landmark);
    }

    // find the earliest pose
    FrameId earliest_frame = std::numeric_limits<FrameId>::max();
    for (auto key : input.backend_output_->state_.keys()) {
      Symbol symbol(key);
      if (symbol.chr() == kPoseSymbolChar) {
        if (symbol.index() < earliest_frame) {
          earliest_frame = symbol.index();
        }
      }
    }

    Pose3 T_smoother_pose = input.backend_output_->state_.at<Pose3>(
        gtsam::Symbol(kPoseSymbolChar, earliest_frame));
    Pose3 T_odom_pose = odom_states_.at<Pose3>(earliest_frame);
    Pose3 W_T_smoother = T_odom_pose * T_smoother_pose.inverse();

    smoother_states_.clear();
    smoother_states_.insert_or_assign(input.backend_output_->state_);
    visualizeLandmarks(map_ / odom_ / "smoother", lmks_vec,
                       aria::viz::ColorMap::kRed);
    drawTf(map_ / odom_ / "smoother", W_T_smoother);
    drawPoints(map_ / odom_ / "smoother" / "states",
               input.backend_output_->state_, {aria::viz::ColorMap::kRed},
               {0.5});
    pose_states_.clear();
    for (const auto &key : smoother_states_.keys()) {
      Symbol symbol(key);
      if (symbol.chr() == kPoseSymbolChar) {
        pose_states_.insert_or_assign(symbol.index(),
                                      smoother_states_.at<Pose3>(key));
      }
    }

    // Check if it's time to save trajectories (every 10 seconds)
    this->checkAndSaveTrajectories(odom_states_);

    return std::make_unique<VIO::VisualizerOutput>();
  }

  void drawCameraEntity(const VIO::VisualizerInput &input) {
    const Frame *frame = input.frontend_output_->getTrackingFrame();
    if (frame == nullptr || frame->img_.empty()) {
      return;
    }

    const gtsam::Pose3 &body_T_cam = frame->cam_param_.body_Pose_cam_;
    const gtsam::Pose3 odom_T_body = input.backend_output_->W_State_Blkf_.pose_;
    const gtsam::Pose3 odom_T_cam = odom_T_body.compose(body_T_cam);
    const std::filesystem::path camera_path =
        map_ / odom_ / baselink_ / "camera";

    this->drawTf(camera_path, body_T_cam, 0.35f, false);
    camera_traj_.push_back(odom_T_cam);
    this->drawTrajectory(map_ / odom_ / "camera_trajectory", camera_traj_,
                         aria::viz::ColorMap::kLightBlue, 0.5f, false);

    const cv::Mat &K = frame->cam_param_.K_;
    std::array<float, 9> K_vec = {
        static_cast<float>(K.at<double>(0, 0)), // fx
        0.f,
        0.f,
        0.f,
        static_cast<float>(K.at<double>(1, 1)), // fy
        0.f,
        static_cast<float>(K.at<double>(0, 2)), // cx
        static_cast<float>(K.at<double>(1, 2)), // cy
        1.f};
    rerun::components::PinholeProjection image_from_camera(K_vec);

    this->rec()->log_with_static(
        camera_path.c_str(), false,
        rerun::Pinhole(image_from_camera)
            .with_resolution(frame->img_.cols, frame->img_.rows)
            .with_image_plane_distance(0.3f)
            .with_camera_xyz(rerun::components::ViewCoordinates::RDF));
  }

  void drawGtTraj(gtsam::Values est_traj_values,
                  const FrameIDTimestampMap &timestamp_map) {
    std::lock_guard<std::mutex> lock(rerun_mutex_);
    if (not gt_trajectory_.empty()) {
      if (est_traj_values.size() - prev_alignment_size_ > 50) {
        prev_alignment_size_ = est_traj_values.size();
        // extract the poses from the gtsam::Values
        std::map<FrameId, gtsam::Pose3> est_poses;
        std::map<FrameId, gtsam::Pose3> gt_poses;
        for (const auto &key : est_traj_values.keys()) {
          if (timestamp_map.find(key) != timestamp_map.end()) {
            Timestamp timestamp = timestamp_map.at(key);
            // find the closest timestamp in gt_trajectory_
            auto it = gt_trajectory_.lower_bound(timestamp);
            if (it != gt_trajectory_.end()) {
              est_poses[key] = est_traj_values.at<gtsam::Pose3>(key);
              gt_poses[key] = it->second;
            }
          }
        }

        // align the poses to the map frame
        gtsam::Point3Pairs poses_to_align;
        for (const auto &[key, pose] : est_poses) {
          if (gt_poses.find(key) != gt_poses.end()) {
            poses_to_align.emplace_back(pose.translation(),
                                        gt_poses.at(key).translation());
          }
        }
        auto T_est_gt = gtsam::Pose3::Align(poses_to_align);
        if (T_est_gt) {
          T_map_gt_ = T_est_gt.value();
          rec()->log(
              "gt_align",
              rerun::TextLog(
                  fmt::format("Aligned {} pairs to GT with t_map_gt: {},{},{}",
                              poses_to_align.size(), T_map_gt_.x(),
                              T_map_gt_.y(), T_map_gt_.z()))
                  .with_level(rerun::TextLogLevel::Info));
        } else {
          rec()->log(
              "gt_align",
              rerun::TextLog("Failed to align estimated trajectory to GT.")
                  .with_level(rerun::TextLogLevel::Error));
        }

        this->drawTf(map_ / "gt", T_map_gt_, 1.0, false);

        std::vector<gtsam::Pose3> gt_traj;
        for (const auto &[key, pose] : gt_trajectory_) {
          gt_traj.push_back(pose);
        }
      }
    }
  }

  void visualizeGraphInSmoother(const VIO::VisualizerInput &input) {
    this->drawPoints(map_ / odom_ / "smoother" / "values",
                     input.backend_output_->state_, {aria::viz::ColorMap::kRed},
                     {2.}, {}, false);
    if (not input.backend_output_->debug_info_.graphBeforeOpt.empty()) {
      this->drawFactors(map_ / odom_ / "smoother" / "graph",
                        input.backend_output_->debug_info_.graphBeforeOpt,
                        input.backend_output_->state_,
                        aria::viz::ColorMap::kRed, 1., false, true);
    }
  }

  void visualizeLandmarks(std::filesystem::path base_frame,
                          const Landmarks &landmarks, Eigen::Vector4f color) {
    std::vector<Point3> lmk_points(landmarks.begin(), landmarks.end());

    this->drawPoints(base_frame / "landmarks", lmk_points, {color}, {0.001}, {},
                     false);
  }

  bool ensureMonoDepthEstimator() {
    if (!mono_depth_params_.enabled || mono_depth_failed_) {
      return false;
    }
#ifdef HAVE_TENSORRT
    if (mono_depth_) {
      return true;
    }
    if (mono_depth_params_.engine_path.empty()) {
      LOG(ERROR) << "Mono depth mapping is enabled but mono_depth.engine_path "
                    "is empty.";
      mono_depth_failed_ = true;
      return false;
    }
    if (!std::filesystem::exists(mono_depth_params_.engine_path)) {
      LOG(ERROR) << "Mono depth TensorRT engine does not exist: "
                 << mono_depth_params_.engine_path;
      mono_depth_failed_ = true;
      return false;
    }

    try {
      xfeat::DepthAnythingV3TRT::Params params;
      params.engine_path = mono_depth_params_.engine_path;
      params.device_id = mono_depth_params_.device_id;
      params.verbose = mono_depth_params_.verbose;
      mono_depth_ = std::make_unique<xfeat::DepthAnythingV3TRT>(params);
    } catch (const std::exception &e) {
      LOG(ERROR) << "Failed to initialize DA3 mono depth: " << e.what();
      mono_depth_failed_ = true;
      return false;
    }
    return true;
#else
    LOG(ERROR) << "Mono depth mapping requires xfeat-cpp TensorRT support, but "
                  "HAVE_TENSORRT is not enabled.";
    mono_depth_failed_ = true;
    return false;
#endif
  }

  void updateMonoDepthMap(const VIO::VisualizerInput &input) {
    const auto frontend_type = input.frontend_output_->frontend_type_;
    if (!mono_depth_params_.enabled ||
        (frontend_type != FrontendType::kMonoImu &&
         frontend_type != FrontendType::kStereoImu)) {
      return;
    }

    if (input.frontend_output_->is_keyframe_) {
      cacheMonoDepthFrame(input);
    }

    const std::optional<FrameId> target_frame_id =
        findOldestSmootherPoseFrameId(input.backend_output_->state_);
    if (!target_frame_id.has_value()) {
      return;
    }

    if (!last_oldest_mono_depth_frame_id_.has_value()) {
      last_oldest_mono_depth_frame_id_ = *target_frame_id;
      return;
    }
    if (*last_oldest_mono_depth_frame_id_ == *target_frame_id) {
      return;
    }
    last_oldest_mono_depth_frame_id_ = *target_frame_id;

    if (last_mono_depth_frame_id_.has_value() &&
        *last_mono_depth_frame_id_ == *target_frame_id) {
      return;
    }

    const auto timestamp_it = timestamp_map_.find(*target_frame_id);
    if (timestamp_it == timestamp_map_.end()) {
      return;
    }

    const auto frame_it = mono_depth_frame_cache_.find(*target_frame_id);
    if (frame_it == mono_depth_frame_cache_.end()) {
      return;
    }

    const gtsam::Symbol target_pose_key(kPoseSymbolChar, *target_frame_id);
    if (input.backend_output_->state_.find(target_pose_key) ==
        input.backend_output_->state_.end()) {
      return;
    }
    if (odom_states_.find(*target_frame_id) == odom_states_.end()) {
      return;
    }

    if (!ensureMonoDepthEstimator()) {
      return;
    }

    const CachedMonoDepthFrame &frame = frame_it->second;
    if (frame.image.empty()) {
      return;
    }
    // Keep the image, intrinsics, and extrinsics from the same camera params.
    const gtsam::Pose3 &body_T_cam = frame.body_T_cam;

    cv::Mat bgr_image;
    if (frame.image.type() == CV_8UC3) {
      bgr_image = frame.image;
    } else if (frame.image.type() == CV_8UC1) {
      cv::cvtColor(frame.image, bgr_image, cv::COLOR_GRAY2BGR);
    } else if (frame.image.type() == CV_8UC4) {
      cv::cvtColor(frame.image, bgr_image, cv::COLOR_BGRA2BGR);
    } else {
      LOG_EVERY_N(WARNING, 30)
          << "Skipping mono depth map update for unsupported image type: "
          << frame.image.type();
      return;
    }

    const auto &intrinsics = frame.intrinsics;
    const double fx = intrinsics[0];
    const double fy = intrinsics[1];
    const double cx = intrinsics[2];
    const double cy = intrinsics[3];
    if (fx <= 0.0 || fy <= 0.0) {
      LOG_EVERY_N(WARNING, 30)
          << "Skipping mono depth map update because camera intrinsics are "
             "invalid.";
      return;
    }

    xfeat::MonoDepthResult depth_result;
    try {
#ifdef HAVE_TENSORRT
      xfeat::CameraIntrinsics mono_intrinsics;
      mono_intrinsics.fx = fx;
      mono_intrinsics.fy = fy;
      mono_intrinsics.cx = cx;
      mono_intrinsics.cy = cy;
      mono_intrinsics.width = bgr_image.cols;
      mono_intrinsics.height = bgr_image.rows;
      depth_result = mono_depth_->infer(bgr_image, mono_intrinsics);
#endif
    } catch (const std::exception &e) {
      LOG(ERROR) << "DA3 mono depth inference failed: " << e.what();
      return;
    }

    const cv::Mat &depth = depth_result.depth;
    if (depth.empty() || depth.type() != CV_32FC1) {
      LOG_EVERY_N(WARNING, 30)
          << "Skipping mono depth map update because DA3 returned an empty or "
             "non-float depth map.";
      return;
    }

    const int stride = std::max(1, mono_depth_params_.point_stride);
    const int max_points =
        std::max(0, mono_depth_params_.max_points_per_keyframe);
    if (max_points == 0) {
      return;
    }

    const gtsam::Pose3 smoother_T_body =
        input.backend_output_->state_.at<Pose3>(target_pose_key);
    const gtsam::Pose3 odom_T_body =
        odom_states_.at<Pose3>(*target_frame_id);
    const gtsam::Pose3 odom_T_smoother =
        odom_T_body.compose(smoother_T_body.inverse());
    const gtsam::Pose3 smoother_T_cam = smoother_T_body.compose(body_T_cam);
    const gtsam::Pose3 odom_T_cam = odom_T_smoother.compose(smoother_T_cam);
    const MonoDepthScaleEstimate scale_estimate = estimateMonoDepthScale(
        frame, input.backend_output_->landmarks_in_local_window_, depth,
        depth_result.sky_mask, smoother_T_cam.inverse());
    if (scale_estimate.updated) {
      mono_depth_scale_ = scale_estimate.scale;
      mono_depth_scale_valid_ = true;
    }
    const double depth_scale =
        mono_depth_scale_valid_ ? mono_depth_scale_ : 1.0;
    this->setTimeNSec(timestamp_it->second);
    this->drawScalar((map_ / odom_ / "mono_depth" / "scale").string(),
                     depth_scale);
    this->drawScalar((map_ / odom_ / "mono_depth" / "scale_pairs").string(),
                     static_cast<double>(scale_estimate.inlier_pairs));
    this->drawScalar((map_ / odom_ / "mono_depth" / "scale_log_rmse").string(),
                     scale_estimate.log_rmse);

    std::vector<Point3> candidate_points;
    std::vector<Eigen::Vector4f> candidate_colors;

    const int rows = std::min(depth.rows, bgr_image.rows);
    const int cols = std::min(depth.cols, bgr_image.cols);
    const bool has_sky_mask =
        !depth_result.sky_mask.empty() && depth_result.sky_mask.rows >= rows &&
        depth_result.sky_mask.cols >= cols;
    const size_t candidate_reserve =
        static_cast<size_t>(((rows + stride - 1) / stride) *
                            ((cols + stride - 1) / stride));
    candidate_points.reserve(candidate_reserve);
    candidate_colors.reserve(candidate_reserve);
    for (int v = 0; v < rows; v += stride) {
      const float *depth_row = depth.ptr<float>(v);
      const cv::Vec3b *color_row = bgr_image.ptr<cv::Vec3b>(v);
      const uint8_t *sky_row =
          has_sky_mask ? depth_result.sky_mask.ptr<uint8_t>(v) : nullptr;
      for (int u = 0; u < cols; u += stride) {
        if (sky_row != nullptr && sky_row[u] != 0u) {
          continue;
        }
        const float raw_z = depth_row[u];
        if (!std::isfinite(raw_z) || raw_z <= 0.0f) {
          continue;
        }
        const double z = static_cast<double>(raw_z) * depth_scale;
        if (!std::isfinite(z) || z < mono_depth_params_.min_depth_m ||
            z > mono_depth_params_.max_depth_m) {
          continue;
        }

        const double x = (static_cast<double>(u) - cx) * z / fx;
        const double y = (static_cast<double>(v) - cy) * z / fy;
        const Point3 point_odom = odom_T_cam.transformFrom(Point3(x, y, z));
        const cv::Vec3b &bgr = color_row[u];
        candidate_points.push_back(point_odom);
        candidate_colors.emplace_back(static_cast<float>(bgr[2]),
                                      static_cast<float>(bgr[1]),
                                      static_cast<float>(bgr[0]),
                                      180.0f);
      }
    }

    const size_t candidate_count = candidate_points.size();
    std::vector<Point3> new_points;
    std::vector<Eigen::Vector4f> new_colors;
    if (static_cast<int>(candidate_count) <= max_points) {
      new_points = std::move(candidate_points);
      new_colors = std::move(candidate_colors);
    } else {
      new_points.reserve(static_cast<size_t>(max_points));
      new_colors.reserve(static_cast<size_t>(max_points));
      for (int i = 0; i < max_points; ++i) {
        const size_t idx =
            std::min(candidate_count - 1,
                     (static_cast<size_t>(i) * candidate_count) /
                         static_cast<size_t>(max_points));
        new_points.push_back(candidate_points[idx]);
        new_colors.push_back(candidate_colors[idx]);
      }
    }

    if (new_points.empty()) {
      this->setTimeNSec(input.timestamp_);
      return;
    }

    mono_depth_map_points_.insert(
        mono_depth_map_points_.end(), new_points.begin(), new_points.end());
    mono_depth_map_colors_.insert(
        mono_depth_map_colors_.end(), new_colors.begin(), new_colors.end());

    const int max_map_points = std::max(0, mono_depth_params_.max_map_points);
    if (max_map_points > 0 &&
        static_cast<int>(mono_depth_map_points_.size()) > max_map_points) {
      const size_t excess =
          mono_depth_map_points_.size() - static_cast<size_t>(max_map_points);
      mono_depth_map_points_.erase(mono_depth_map_points_.begin(),
                                   mono_depth_map_points_.begin() + excess);
      mono_depth_map_colors_.erase(mono_depth_map_colors_.begin(),
                                   mono_depth_map_colors_.begin() + excess);
    }

    this->drawPoints(map_ / odom_ / "mono_depth" / "map",
                     mono_depth_map_points_,
                     mono_depth_map_colors_,
                     {mono_depth_params_.point_radius},
                     {},
                     false);
    this->setTimeNSec(input.timestamp_);
    last_mono_depth_frame_id_ = *target_frame_id;
    LOG_EVERY_N(INFO, 10)
        << "Mono depth map points: " << mono_depth_map_points_.size()
        << " (" << new_points.size() << "/" << candidate_count
        << " keyframe points for delayed frame " << *target_frame_id
        << "), scale: " << depth_scale << " from "
        << scale_estimate.inlier_pairs << "/"
        << scale_estimate.candidate_pairs
        << " landmark depth pairs, log rmse: " << scale_estimate.log_rmse;
  }

  void checkAndSaveTrajectories(const gtsam::Values &states = gtsam::Values()) {
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_save_time_ >= save_interval_) {
      if (!states.empty() && !timestamp_map_.empty()) {
        this->saveTUMTrajFile(states, timestamp_map_, "DE-VIO");
      }
      last_save_time_ = current_time;
    }
  }

  void forceSaveTrajectories() {
    if (!odom_states_.empty() && !timestamp_map_.empty()) {
      this->saveTUMTrajFile(odom_states_, timestamp_map_, "DE-VIO");
    }
    last_save_time_ = std::chrono::steady_clock::now();
  }

  void saveTUMTrajFile(const gtsam::Values &states,
                       const FrameIDTimestampMap &timestamp_map,
                       std::string name) {
    if (result_dir_.empty()) {
      return;
    }
    if (save_traj_futures_.find(name) != save_traj_futures_.end() &&
        save_traj_futures_[name].valid() &&
        save_traj_futures_[name].wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
      return; // Previous save is still in progress
    } else {
      save_traj_futures_.emplace(name, std::future<void>());
    }

    std::string filename = fmt::format("{}/{}.txt", result_dir_, name);

    auto write_traj = [&](const std::string &filename,
                          const gtsam::Values &states,
                          const FrameIDTimestampMap &timestamp_map) {
      // open file
      std::ofstream ofs(filename);
      if (!ofs.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
      }

      for (auto const &[key, value] : states) {
        auto it = timestamp_map.find(key);
        Timestamp timestamp;
        if (it != timestamp_map.end()) {
          timestamp = it->second;
        } else {
          continue;
        }
        double x, y, z, qx, qy, qz, qw;
        Pose3 pose = value.cast<Pose3>();
        x = pose.x();
        y = pose.y();
        z = pose.z();
        auto quat = pose.rotation().toQuaternion();
        qx = quat.x();
        qy = quat.y();
        qz = quat.z();
        qw = quat.w();

        double seconds =
            static_cast<double>(timestamp) / 1e9; // Convert ns to s

        ofs << std::fixed << std::setprecision(6) << seconds << " " << x << " "
            << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw
            << std::endl;
      }
      // close file
      ofs.close();
    };

    save_traj_futures_[name] = std::async(std::launch::async, write_traj,
                                          filename, states, timestamp_map);
  }

  std::vector<Eigen::Vector4f>
  getColorsFromFactorsType(const NonlinearFactorGraph &factors) {
    std::vector<Eigen::Vector4f> colors(factors.size(),
                                        aria::viz::ColorMap::kBlack);
    for (size_t i = 0; i < factors.size(); ++i) {
      const auto &factor = factors[i];
      if (factor == nullptr) {
        continue;
      }
      auto keys = factor->keys();
      if (keys.size() > 2)
        continue;
      uint64_t diff =
          (keys[0] > keys[1]) ? (keys[0] - keys[1]) : (keys[1] - keys[0]);
      if (diff == 1 or keys.size() == 1) {               // odometry edge
        colors[i] = Eigen::Vector4f(255, 255, 0.0, 255); // yellow
        continue;
      } else if (diff > 1 and keys.size() == 2) { // loop closure edge
        auto between_factor =
            std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
                factor);
        if (not between_factor) {
          factor->print();
          LOG(FATAL) << "Factor is not a BetweenFactor";
        }
        auto noise = between_factor->noiseModel();
        CHECK(noise);
        auto gauss =
            std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise);
        if (not gauss) {
          auto robust =
              std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise);
          gauss = std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
              robust->noise());
        }
        CHECK(gauss);
        auto info = gauss->information();
        double trans_precision = info.block<3, 3>(3, 3).norm();
        // if trans precision too small, then rot only factor
        if (trans_precision < 1e-6) {
          colors[i] = aria::viz::ColorMap::kRed;
        } else {
          colors[i] = aria::viz::ColorMap::kGreen;
        }
      }
    }
    return colors;
  }

  std::map<Timestamp, gtsam::Pose3>
  loadTrajectoryMapFromCSV(const std::string &filename) {
    std::map<int64_t, gtsam::Pose3> trajectory;
    std::ifstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty())
        continue;

      std::istringstream iss(line);
      std::vector<double> values;
      std::string token;

      while (std::getline(iss, token, ',')) {
        try {
          values.push_back(std::stod(token));
        } catch (const std::invalid_argument &) {
          std::cerr << "Invalid number in line: " << line << std::endl;
          values.clear();
          break;
        }
      }

      if (values.size() != 8) {
        std::cerr << "Skipping malformed line: " << line << std::endl;
        continue;
      }

      // Convert timestamp to int64_t (assume it's in seconds, multiply to get
      // nanoseconds)
      int64_t timestamp_ns = static_cast<int64_t>(values[0] * 1e9);

      double x = values[1];
      double y = values[2];
      double z = values[3];
      double qx = values[4];
      double qy = values[5];
      double qz = values[6];
      double qw = values[7];

      gtsam::Rot3 R = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
      gtsam::Point3 t(x, y, z);
      trajectory[timestamp_ns] = gtsam::Pose3(R, t);
    }

    return trajectory;
  }

private:
  struct MonoDepthScaleEstimate {
    double scale = 1.0;
    double log_rmse = 0.0;
    size_t candidate_pairs = 0u;
    size_t inlier_pairs = 0u;
    bool updated = false;
  };

  static bool sampleDepthBilinear(const cv::Mat &depth,
                                  const cv::Point2f &px,
                                  float *sampled_depth) {
    if (sampled_depth == nullptr || depth.empty() ||
        depth.type() != CV_32FC1 || !std::isfinite(px.x) ||
        !std::isfinite(px.y) || px.x < 0.0f || px.y < 0.0f ||
        px.x > static_cast<float>(depth.cols - 1) ||
        px.y > static_cast<float>(depth.rows - 1)) {
      return false;
    }

    const int x0 = static_cast<int>(std::floor(px.x));
    const int y0 = static_cast<int>(std::floor(px.y));
    const int x1 = std::min(x0 + 1, depth.cols - 1);
    const int y1 = std::min(y0 + 1, depth.rows - 1);
    const float wx = px.x - static_cast<float>(x0);
    const float wy = px.y - static_cast<float>(y0);

    const float z00 = depth.at<float>(y0, x0);
    const float z01 = depth.at<float>(y0, x1);
    const float z10 = depth.at<float>(y1, x0);
    const float z11 = depth.at<float>(y1, x1);
    if (!std::isfinite(z00) || !std::isfinite(z01) || !std::isfinite(z10) ||
        !std::isfinite(z11)) {
      return false;
    }

    *sampled_depth = (1.0f - wx) * (1.0f - wy) * z00 +
                     wx * (1.0f - wy) * z01 +
                     (1.0f - wx) * wy * z10 + wx * wy * z11;
    return true;
  }

  static bool isSkyPixel(const cv::Mat &sky_mask, const cv::Point2f &px) {
    if (sky_mask.empty() || sky_mask.type() != CV_8UC1 ||
        !std::isfinite(px.x) || !std::isfinite(px.y) || px.x < 0.0f ||
        px.y < 0.0f || px.x > static_cast<float>(sky_mask.cols - 1) ||
        px.y > static_cast<float>(sky_mask.rows - 1)) {
      return false;
    }

    const int u = std::min(std::max(static_cast<int>(std::lround(px.x)), 0),
                           sky_mask.cols - 1);
    const int v = std::min(std::max(static_cast<int>(std::lround(px.y)), 0),
                           sky_mask.rows - 1);
    return sky_mask.at<uint8_t>(v, u) != 0u;
  }

  static double medianValue(std::vector<double> values) {
    CHECK(!values.empty());
    const size_t middle = values.size() / 2u;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double median = values[middle];
    if (values.size() % 2u == 0u) {
      std::nth_element(values.begin(), values.begin() + middle - 1u,
                       values.end());
      median = 0.5 * (median + values[middle - 1u]);
    }
    return median;
  }

  void cacheMonoDepthFrame(const VIO::VisualizerInput &input) {
    const Frame *frame = input.frontend_output_->getTrackingFrame();
    if (frame == nullptr || frame->img_.empty()) {
      return;
    }

    CachedMonoDepthFrame cached_frame;
    cached_frame.image = frame->img_.clone();
    cached_frame.intrinsics = frame->cam_param_.intrinsics_;
    cached_frame.body_T_cam = frame->cam_param_.body_Pose_cam_;
    cached_frame.keypoints = frame->keypoints_;
    cached_frame.landmarks = frame->landmarks_;
    mono_depth_frame_cache_[input.backend_output_->cur_kf_id_] =
        std::move(cached_frame);

    while (mono_depth_frame_cache_.size() > kMonoDepthFrameCacheSize) {
      mono_depth_frame_cache_.erase(mono_depth_frame_cache_.begin());
    }
  }

  static std::optional<FrameId>
  findOldestSmootherPoseFrameId(const gtsam::Values &state) {
    std::optional<FrameId> oldest_frame_id = std::nullopt;
    for (auto key : state.keys()) {
      const Symbol symbol(key);
      if (symbol.chr() != kPoseSymbolChar) {
        continue;
      }
      const FrameId frame_id = symbol.index();
      if (!oldest_frame_id.has_value() || frame_id < *oldest_frame_id) {
        oldest_frame_id = frame_id;
      }
    }
    return oldest_frame_id;
  }

  MonoDepthScaleEstimate estimateMonoDepthScale(
      const CachedMonoDepthFrame &frame,
      const PointsWithIdMap &landmarks,
      const cv::Mat &depth,
      const cv::Mat &sky_mask,
      const gtsam::Pose3 &cam_T_landmark_frame) const {
    static constexpr size_t kMinPairs = 8u;
    static constexpr double kMinScale = 0.05;
    static constexpr double kMaxScale = 20.0;
    static constexpr double kRatioInlierFactor = 2.0;
    const double log_ratio_inlier_threshold =
        std::log(kRatioInlierFactor);

    MonoDepthScaleEstimate estimate;
    estimate.scale = mono_depth_scale_valid_ ? mono_depth_scale_ : 1.0;
    if (frame.keypoints.empty() || frame.landmarks.empty() || landmarks.empty() ||
        depth.empty() || depth.type() != CV_32FC1) {
      return estimate;
    }

    std::vector<double> log_ratios;
    const size_t feature_count =
        std::min(frame.keypoints.size(), frame.landmarks.size());
    log_ratios.reserve(feature_count);

    for (size_t i = 0u; i < feature_count; ++i) {
      const LandmarkId lmk_id = frame.landmarks[i];
      if (lmk_id == -1) {
        continue;
      }
      const auto lmk_it = landmarks.find(lmk_id);
      if (lmk_it == landmarks.end()) {
        continue;
      }

      const Point3 landmark_cam =
          cam_T_landmark_frame.transformFrom(lmk_it->second);
      const double landmark_depth = landmark_cam.z();
      if (!std::isfinite(landmark_depth) ||
          landmark_depth < mono_depth_params_.min_depth_m ||
          landmark_depth > mono_depth_params_.max_depth_m) {
        continue;
      }

      const cv::Point2f &px = frame.keypoints[i];
      if (isSkyPixel(sky_mask, px)) {
        continue;
      }

      float da3_depth = 0.0f;
      if (!sampleDepthBilinear(depth, px, &da3_depth) ||
          !std::isfinite(da3_depth) || da3_depth <= 0.0f) {
        continue;
      }

      const double da3_depth_d = static_cast<double>(da3_depth);
      log_ratios.push_back(std::log(landmark_depth) - std::log(da3_depth_d));
    }

    estimate.candidate_pairs = log_ratios.size();
    if (log_ratios.size() < kMinPairs) {
      return estimate;
    }

    const double median_log_ratio = medianValue(log_ratios);
    if (!std::isfinite(median_log_ratio)) {
      return estimate;
    }

    double sum_log_ratio = 0.0;
    for (const double log_ratio : log_ratios) {
      if (std::abs(log_ratio - median_log_ratio) >
          log_ratio_inlier_threshold) {
        continue;
      }
      sum_log_ratio += log_ratio;
      ++estimate.inlier_pairs;
    }

    if (estimate.inlier_pairs < kMinPairs) {
      return estimate;
    }

    // Minimize sum_i (log(scale * da3_i) - log(landmark_i))^2.
    const double log_scale =
        sum_log_ratio / static_cast<double>(estimate.inlier_pairs);
    const double scale = std::exp(log_scale);
    if (!std::isfinite(scale) || scale < kMinScale || scale > kMaxScale) {
      return estimate;
    }

    double sum_squared_log_error = 0.0;
    for (const double log_ratio : log_ratios) {
      if (std::abs(log_ratio - median_log_ratio) >
          log_ratio_inlier_threshold) {
        continue;
      }
      const double residual = log_scale - log_ratio;
      sum_squared_log_error += residual * residual;
    }

    estimate.scale = scale;
    estimate.log_rmse =
        std::sqrt(sum_squared_log_error /
                  static_cast<double>(estimate.inlier_pairs));
    estimate.updated = true;
    return estimate;
  }

  std::filesystem::path baselink_;
  std::filesystem::path map_;
  std::filesystem::path odom_;

  std::vector<Pose3> odom_traj_{};
  gtsam::Values odom_states_;

  std::future<void> draw_gt_traj_future_;
  std::map<std::string, std::future<void>> save_traj_futures_;
  std::future<void> lcd_output_future_;

  gtsam::Values smoother_states_;
  gtsam::Values pose_states_;

  std::map<Timestamp, Pose3> gt_trajectory_;
  Pose3 T_map_gt_ = Pose3::Identity();
  size_t prev_alignment_size_ = 0;

  std::string result_dir_{};

  PointsWithIdMap landmarks_in_odom_;
  std::vector<Pose3> camera_traj_;
  MonoDepthParams mono_depth_params_;
#ifdef HAVE_TENSORRT
  std::unique_ptr<xfeat::DepthAnythingV3TRT> mono_depth_;
#endif
  bool mono_depth_failed_ = false;
  std::optional<FrameId> last_mono_depth_frame_id_ = std::nullopt;
  std::optional<FrameId> last_oldest_mono_depth_frame_id_ = std::nullopt;
  double mono_depth_scale_ = 1.0;
  bool mono_depth_scale_valid_ = false;
  std::map<FrameId, CachedMonoDepthFrame> mono_depth_frame_cache_;
  std::vector<Point3> mono_depth_map_points_;
  std::vector<Eigen::Vector4f> mono_depth_map_colors_;

  FrameIDTimestampMap timestamp_map_;

  std::optional<std::pair<FrameId, FrameId>> last_odom_pair_{std::nullopt};
  ISAM2 isam2_;

  std::mutex rerun_mutex_;

  // Timer for saving trajectory files
  std::chrono::steady_clock::time_point last_save_time_;
  static constexpr std::chrono::seconds save_interval_{10};
  static constexpr size_t kMonoDepthFrameCacheSize = 256u;
};

} // namespace VIO
