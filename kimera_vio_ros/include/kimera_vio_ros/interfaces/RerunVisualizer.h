#pragma once

#include <aria_viz/visualizer_rerun.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <gtsam/slam/dataset.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <pcl_msgs/msg/vertices.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <optional>
#include <set>

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
    (void)full_filename;
    (void)tm_time;
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
  static constexpr char kPoseSymbolChar = 'x';

  struct Params {
    std::string base_link_frame_id = "baselink";
    std::string odom_frame_id = "odom";
    std::string map_frame_id = "map";
    std::string gt_csv_file = "";
    std::optional<std::string> recording_id = std::nullopt;
    std::string result_dir = "";
    rclcpp::Node::SharedPtr node = nullptr;
  };

  RerunVisualizer(const Params &params)
      : RerunVisualizer(params.base_link_frame_id, params.odom_frame_id,
                        params.map_frame_id, params.gt_csv_file,
                        params.recording_id, params.result_dir, params.node) {}

  RerunVisualizer(std::string base_link_frame_id = "baselink",
                  std::string odom_frame_id = "odom",
                  std::string map_frame_id = "map",
                  std::string gt_csv_file = "",
                  std::optional<std::string> recording_id = std::nullopt,
                  std::string result_dir = "",
                  rclcpp::Node::SharedPtr node = nullptr)
      // Keep the Kimera visualizer module mesh-aware so mesher outputs are
      // queued for the Rerun path as well.
        : VIO::Visualizer3D(VIO::VisualizationType::kMesh2dTo3dSparse,
                          VIO::BackendType::kStereoImu),
        aria::viz::VisualizerRerun(aria::viz::VisualizerRerun::Params(
            "kimera_vio", recording_id, "rerun+http://127.0.0.1:9876/proxy")),
        baselink_(base_link_frame_id), map_(map_frame_id), odom_(odom_frame_id),
        node_(std::move(node)),
        result_dir_(result_dir) {
    if (node_) {
      auto mesh_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      mesh_pub_ =
          node_->create_publisher<pcl_msgs::msg::PolygonMesh>("mesh", mesh_qos);
      mesh_texture_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "debug_mesh_img", mesh_qos);
    }

    // draw the origin frame for visualization
    this->drawTf(map_, Pose3(), 0.3, true);

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
    (void)filename;
    (void)line;
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

    cv::Mat tracking_image_clone = input.frontend_output_->feature_tracks_.clone();

    // only draw every 3 frames
    // if (input.backend_output_->cur_kf_id_ % 3 == 0) {
    //   this->drawCamera(input.backend_output_->cur_kf_id_,
    //                    input.backend_output_->W_State_Blkf_.pose_,
    //                    tracking_image_clone,
    //                    K,
    //                    false);
    // }

    if (not input.frontend_output_->feature_tracks_.empty()) {
      cv::Mat small_image;
      cv::resize(tracking_image_clone, small_image, cv::Size(), 0.5, 0.5);
      this->drawImage(map_ / odom_ / baselink_ / "tracking" / "image",
                      small_image, false);
    }

    drawKimeraMesh(input);
    publishKimeraMeshTopics(input);

    // Check if it's time to save trajectories (every 10 seconds)
    this->checkAndSaveTrajectories(odom_states_);

    return std::make_unique<VIO::VisualizerOutput>();
  }

  void drawCamera(int id, const Pose3 &cam_pose, const cv::Mat &image,
                  const cv::Mat &K, bool is_static = true) {
    // draw the tf for this camera
    std::string cam_name = fmt::format("f{}", id);
    this->drawTf(map_ / odom_ / "camera" / cam_name, cam_pose, 0, is_static);

    if (not image.empty()) {
      cv::Mat rgba32;
      if (image.type() == CV_8UC3) {
        cv::cvtColor(image, rgba32, cv::COLOR_BGR2RGBA);
      } else if (image.type() == CV_8UC1) {
        cv::cvtColor(image, rgba32, cv::COLOR_GRAY2RGBA);
      } else if (image.type() == CV_8UC4) {
        rgba32 = image;
      } else {
        throw std::runtime_error("Unsupported image type");
      }
      std::array<float, 9> K_vec = {
          static_cast<float>(K.at<double>(0, 0)), // fx
          0.f,
          static_cast<float>(K.at<double>(0, 2)), // cx
          0.f,
          static_cast<float>(K.at<double>(1, 1)), // fy
          static_cast<float>(K.at<double>(1, 2)), // cy
          0.f,
          0.f,
          1.f};
      rerun::components::PinholeProjection pp(K_vec);

      this->rec()->log_with_static(
          (map_ / odom_ / "camera" / cam_name).c_str(), is_static,
          rerun::Image::from_rgba32(rgba32,
                                    {static_cast<uint32_t>(image.cols),
                                     static_cast<uint32_t>(image.rows)}));
      // draw the camera
      this->rec()->log_with_static(
          (map_ / odom_ / "camera" / cam_name).c_str(), is_static,
          rerun::Pinhole::from_focal_length_and_resolution(
              K_vec[0],
              {static_cast<float>(image.cols), static_cast<float>(image.rows)})
              .with_image_plane_distance(0.1f)
              .with_camera_xyz(rerun::components::ViewCoordinates::FRD));
      // .with_image_from_camera(pp));
    }
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
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
                factor);
        if (not between_factor) {
          factor->print();
          LOG(FATAL) << "Factor is not a BetweenFactor";
        }
        auto noise = between_factor->noiseModel();
        CHECK(noise);
        auto gauss =
            boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise);
        if (not gauss) {
          auto robust =
              boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise);
          gauss = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
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

  struct MeshLogData {
    std::vector<gtsam::Point3> vertex_positions;
    std::vector<aria::viz::MeshTriangle> triangle_indices;
    std::vector<aria::viz::MeshTexcoord> vertex_texcoords;
    std::vector<gtsam::Point3> vertex_normals;
    cv::Mat texture_image;
  };

  static std::optional<std::string> getRosImageEncoding(const cv::Mat& image) {
    switch (image.type()) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_8UC4:
        return "bgra8";
      default:
        return std::nullopt;
    }
  }

  sensor_msgs::msg::PointCloud2 buildMeshPointCloudMessage(
      const MeshLogData& mesh_log_data, const rclcpp::Time& stamp) const {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = map_.string();

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
        8, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "normal_x", 1,
        sensor_msgs::msg::PointField::FLOAT32, "normal_y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "normal_z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "u", 1,
        sensor_msgs::msg::PointField::FLOAT32, "v", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(mesh_log_data.vertex_positions.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_normal_x(cloud_msg, "normal_x");
    sensor_msgs::PointCloud2Iterator<float> iter_normal_y(cloud_msg, "normal_y");
    sensor_msgs::PointCloud2Iterator<float> iter_normal_z(cloud_msg, "normal_z");
    sensor_msgs::PointCloud2Iterator<float> iter_u(cloud_msg, "u");
    sensor_msgs::PointCloud2Iterator<float> iter_v(cloud_msg, "v");

    for (size_t idx = 0; idx < mesh_log_data.vertex_positions.size();
         ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_normal_x, ++iter_normal_y,
                ++iter_normal_z, ++iter_u, ++iter_v) {
      const auto& position = mesh_log_data.vertex_positions[idx];
      const auto& texcoord = mesh_log_data.vertex_texcoords[idx];
      const auto& normal = mesh_log_data.vertex_normals[idx];

      *iter_x = static_cast<float>(position.x());
      *iter_y = static_cast<float>(position.y());
      *iter_z = static_cast<float>(position.z());
      *iter_normal_x = static_cast<float>(normal.x());
      *iter_normal_y = static_cast<float>(normal.y());
      *iter_normal_z = static_cast<float>(normal.z());
      *iter_u = texcoord[0];
      *iter_v = texcoord[1];
    }

    return cloud_msg;
  }

  pcl_msgs::msg::PolygonMesh buildPolygonMeshMessage(
      const MeshLogData& mesh_log_data, const rclcpp::Time& stamp) const {
    pcl_msgs::msg::PolygonMesh mesh_msg;
    mesh_msg.header.stamp = stamp;
    mesh_msg.header.frame_id = map_.string();
    mesh_msg.cloud = buildMeshPointCloudMessage(mesh_log_data, stamp);
    mesh_msg.polygons.reserve(mesh_log_data.triangle_indices.size());
    for (const auto& triangle : mesh_log_data.triangle_indices) {
      pcl_msgs::msg::Vertices polygon;
      polygon.vertices = {triangle[0], triangle[1], triangle[2]};
      mesh_msg.polygons.push_back(std::move(polygon));
    }
    return mesh_msg;
  }

  sensor_msgs::msg::Image::SharedPtr buildTextureImageMessage(
      const cv::Mat& texture_image, const rclcpp::Time& stamp) const {
    const auto encoding = getRosImageEncoding(texture_image);
    if (!encoding) {
      return nullptr;
    }

    cv_bridge::CvImage texture_cv_image;
    texture_cv_image.header.stamp = stamp;
    texture_cv_image.header.frame_id = map_.string();
    texture_cv_image.encoding = *encoding;
    texture_cv_image.image = texture_image;
    return texture_cv_image.toImageMsg();
  }

  static cv::Mat getStereoTextureImage(const VIO::FrontendOutput& frontend_output) {
    const auto& stereo_frame = frontend_output.stereo_frame_lkf_;
    if (!stereo_frame.getLeftFrame().img_.empty()) {
      return stereo_frame.getLeftFrame().img_;
    }
    if (!stereo_frame.left_img_rectified_.empty()) {
      return stereo_frame.left_img_rectified_;
    }
    return cv::Mat();
  }

  static std::vector<gtsam::Point3> extractMeshVertexPositions(
      const VIO::Mesh3D& mesh_3d) {
    cv::Mat vertices_mesh;
    mesh_3d.convertVerticesMeshToMat(&vertices_mesh);

    std::vector<gtsam::Point3> vertex_positions;
    vertex_positions.reserve(vertices_mesh.rows);
    for (int row = 0; row < vertices_mesh.rows; ++row) {
      const auto& vertex = vertices_mesh.at<cv::Point3f>(row, 0);
      vertex_positions.emplace_back(vertex.x, vertex.y, vertex.z);
    }
    return vertex_positions;
  }

  static std::vector<aria::viz::MeshTriangle> extractMeshTriangleIndices(
      const VIO::Mesh3D& mesh_3d, cv::Mat* polygons_mesh_out = nullptr) {
    cv::Mat polygons_mesh;
    mesh_3d.convertPolygonsMeshToMat(&polygons_mesh);
    if (polygons_mesh_out) {
      *polygons_mesh_out = polygons_mesh;
    }

    const size_t polygon_dimension = mesh_3d.getMeshPolygonDimension();
    if (polygon_dimension != 3u) {
      throw std::runtime_error("Only triangular meshes are supported in RerunVisualizer");
    }
    if (polygons_mesh.rows % static_cast<int>(polygon_dimension + 1u) != 0) {
      throw std::runtime_error("Malformed mesh polygon buffer");
    }

    std::vector<aria::viz::MeshTriangle> triangles;
    triangles.reserve(mesh_3d.getNumberOfPolygons());
    for (size_t polygon_idx = 0; polygon_idx < mesh_3d.getNumberOfPolygons();
         ++polygon_idx) {
      const int offset =
          static_cast<int>(polygon_idx * (polygon_dimension + 1u));
      const int polygon_size = polygons_mesh.at<int>(offset, 0);
      if (polygon_size != 3) {
        throw std::runtime_error("Encountered a non-triangle polygon");
      }

      triangles.push_back({
          static_cast<uint32_t>(polygons_mesh.at<int>(offset + 1, 0)),
          static_cast<uint32_t>(polygons_mesh.at<int>(offset + 2, 0)),
          static_cast<uint32_t>(polygons_mesh.at<int>(offset + 3, 0)),
      });
    }
    return triangles;
  }

  static std::optional<MeshLogData> buildTexturedMeshLogData(
      const VIO::VisualizerInput& input) {
    if (!input.mesher_output_) {
      return std::nullopt;
    }

    const cv::Mat texture_image = getStereoTextureImage(*input.frontend_output_);
    if (texture_image.empty()) {
      return std::nullopt;
    }

    const auto& mesh_2d = input.mesher_output_->mesh_2d_;
    const auto& mesh_3d = input.mesher_output_->mesh_3d_;
    const size_t polygon_dimension = mesh_2d.getMeshPolygonDimension();
    if (polygon_dimension != 3u || mesh_2d.getNumberOfPolygons() == 0) {
      return std::nullopt;
    }

    MeshLogData mesh_log_data;
    mesh_log_data.texture_image = texture_image;
    mesh_log_data.vertex_positions.reserve(mesh_2d.getNumberOfPolygons() *
                                           polygon_dimension);
    mesh_log_data.vertex_texcoords.reserve(mesh_2d.getNumberOfPolygons() *
                                           polygon_dimension);
    mesh_log_data.vertex_normals.reserve(mesh_2d.getNumberOfPolygons() *
                                         polygon_dimension);
    mesh_log_data.triangle_indices.reserve(mesh_2d.getNumberOfPolygons());

    const float denom_x = std::max(1, texture_image.cols);
    const float denom_y = std::max(1, texture_image.rows);

    VIO::Mesh2D::Polygon polygon_2d;
    for (size_t polygon_idx = 0; polygon_idx < mesh_2d.getNumberOfPolygons();
         ++polygon_idx) {
      if (!mesh_2d.getPolygon(polygon_idx, &polygon_2d) ||
          polygon_2d.size() != polygon_dimension) {
        continue;
      }

      VIO::Mesh3D::VertexType vertices_3d[3];
      if (!mesh_3d.getVertex(polygon_2d[0].getLmkId(), &vertices_3d[0]) ||
          !mesh_3d.getVertex(polygon_2d[1].getLmkId(), &vertices_3d[1]) ||
          !mesh_3d.getVertex(polygon_2d[2].getLmkId(), &vertices_3d[2])) {
        continue;
      }

      const uint32_t base_index =
          static_cast<uint32_t>(mesh_log_data.vertex_positions.size());

      for (size_t vertex_idx = 0; vertex_idx < polygon_dimension; ++vertex_idx) {
        const auto& position = vertices_3d[vertex_idx].getVertexPosition();
        const auto& normal = vertices_3d[vertex_idx].getVertexNormal();
        const auto& pixel = polygon_2d[vertex_idx].getVertexPosition();

        mesh_log_data.vertex_positions.emplace_back(position.x, position.y,
                                                    position.z);
        mesh_log_data.vertex_normals.emplace_back(normal.x, normal.y, normal.z);
        mesh_log_data.vertex_texcoords.push_back({
            std::clamp(pixel.x / denom_x, 0.0f, 1.0f),
            std::clamp(pixel.y / denom_y, 0.0f, 1.0f),
        });
      }

      // Match the legacy RViz publisher winding order.
      mesh_log_data.triangle_indices.push_back(
          {base_index + 2u, base_index + 1u, base_index});
    }

    if (mesh_log_data.triangle_indices.empty()) {
      return std::nullopt;
    }
    return mesh_log_data;
  }

  static std::optional<MeshLogData> buildGeometryMeshLogData(
      const VIO::VisualizerInput& input) {
    if (!input.mesher_output_) {
      return std::nullopt;
    }

    const auto& mesh_3d = input.mesher_output_->mesh_3d_;
    if (mesh_3d.getNumberOfPolygons() == 0 ||
        mesh_3d.getNumberOfUniqueVertices() == 0) {
      return std::nullopt;
    }

    MeshLogData mesh_log_data;
    mesh_log_data.vertex_positions = extractMeshVertexPositions(mesh_3d);
    mesh_log_data.triangle_indices = extractMeshTriangleIndices(mesh_3d);
    return mesh_log_data;
  }

  static std::vector<std::pair<Point3, Point3>> buildMeshWireframe(
      const std::vector<Point3>& vertex_positions,
      const std::vector<aria::viz::MeshTriangle>& triangle_indices) {
    std::vector<std::pair<Point3, Point3>> wireframe_segments;
    wireframe_segments.reserve(triangle_indices.size() * 3u);

    std::set<std::pair<uint32_t, uint32_t>> seen_edges;
    for (const auto& triangle : triangle_indices) {
      const std::array<std::pair<uint32_t, uint32_t>, 3> edges = {{
          {triangle[0], triangle[1]},
          {triangle[1], triangle[2]},
          {triangle[2], triangle[0]},
      }};

      for (const auto& edge : edges) {
        const auto [vertex0_idx, vertex1_idx] =
            std::minmax(edge.first, edge.second);
        if (vertex0_idx >= vertex_positions.size() ||
            vertex1_idx >= vertex_positions.size()) {
          continue;
        }
        if (!seen_edges.emplace(vertex0_idx, vertex1_idx).second) {
          continue;
        }

        wireframe_segments.emplace_back(vertex_positions[vertex0_idx],
                                        vertex_positions[vertex1_idx]);
      }
    }

    return wireframe_segments;
  }

  void drawKimeraWireframe(const std::string& entity_path,
                           const std::vector<Point3>& vertex_positions,
                           const std::vector<aria::viz::MeshTriangle>& triangle_indices) {
    const auto wireframe_segments =
        buildMeshWireframe(vertex_positions, triangle_indices);
    if (wireframe_segments.empty()) {
      return;
    }

    auto wireframe_color = aria::viz::ColorMap::kBlack;
    wireframe_color[3] = 220.f;
    this->drawLines(entity_path,
                    wireframe_segments,
                    {wireframe_color},
                    1.5f);
  }

  void drawKimeraMesh(const VIO::VisualizerInput& input) {
    const auto mesh_entity = map_ / "mesh";
    const auto wireframe_entity = mesh_entity / "wireframe";
    const auto textured_mesh = buildTexturedMeshLogData(input);
    if (textured_mesh) {
      if (!logged_textured_mesh_once_) {
        logged_textured_mesh_once_ = true;
        LOG(INFO) << "Logging textured Kimera mesh to Rerun with "
                  << textured_mesh->vertex_positions.size()
                  << " vertices and "
                  << textured_mesh->triangle_indices.size() << " triangles.";
      }
      this->drawTexturedMesh(mesh_entity,
                             textured_mesh->vertex_positions,
                             textured_mesh->triangle_indices,
                             textured_mesh->vertex_texcoords,
                             textured_mesh->texture_image,
                             {},
                             textured_mesh->vertex_normals,
                             false);
      drawKimeraWireframe(wireframe_entity,
                          textured_mesh->vertex_positions,
                          textured_mesh->triangle_indices);
      return;
    }

    if (logged_textured_mesh_once_) {
      return;
    }

    const auto geometry_mesh = buildGeometryMeshLogData(input);
    if (!geometry_mesh) {
      return;
    }

    if (!logged_geometry_mesh_once_) {
      logged_geometry_mesh_once_ = true;
      LOG(INFO) << "Logging geometry-only Kimera mesh to Rerun with "
                << geometry_mesh->vertex_positions.size() << " vertices and "
                << geometry_mesh->triangle_indices.size() << " triangles.";
    }
    this->drawMesh(mesh_entity,
                   geometry_mesh->vertex_positions,
                   geometry_mesh->triangle_indices,
                   {},
                   {},
                   false);
    drawKimeraWireframe(wireframe_entity,
                        geometry_mesh->vertex_positions,
                        geometry_mesh->triangle_indices);
  }

  void publishKimeraMeshTopics(const VIO::VisualizerInput& input) {
    if (!mesh_pub_ || !mesh_texture_pub_) {
      return;
    }

    const auto textured_mesh = buildTexturedMeshLogData(input);
    if (!textured_mesh) {
      return;
    }

    const auto stamp = rclcpp::Time(input.timestamp_);
    mesh_pub_->publish(buildPolygonMeshMessage(*textured_mesh, stamp));

    auto texture_msg =
        buildTextureImageMessage(textured_mesh->texture_image, stamp);
    if (texture_msg) {
      mesh_texture_pub_->publish(*texture_msg);
    }
  }

private:
  std::filesystem::path baselink_;
  std::filesystem::path map_;
  std::filesystem::path odom_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mesh_texture_pub_;

  std::vector<Pose3> odom_traj_{};
  gtsam::Values odom_states_;

  std::future<void> draw_gt_traj_future_;
  std::map<std::string, std::future<void>> save_traj_futures_;
  std::future<void> lcd_output_future_;

  std::map<Timestamp, Pose3> gt_trajectory_;
  Pose3 T_map_gt_ = Pose3();
  size_t prev_alignment_size_ = 0;

  std::string result_dir_{};

  FrameIDTimestampMap timestamp_map_;

  std::optional<std::pair<FrameId, FrameId>> last_odom_pair_{std::nullopt};
  bool logged_textured_mesh_once_{false};
  bool logged_geometry_mesh_once_{false};

  std::mutex rerun_mutex_;

  // Timer for saving trajectory files
  std::chrono::steady_clock::time_point last_save_time_;
  static constexpr std::chrono::seconds save_interval_{10};
};

} // namespace VIO
