#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <aria_viz/visualizer_rerun.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "pcl_msgs/msg/polygon_mesh.hpp"
#include "pcl_msgs/msg/vertices.hpp"
#include "projective_mesher_msgs/msg/landmark_observation.hpp"
#include "projective_mesher_msgs/msg/mesher_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace projective_mesher_ros {
namespace {

using LandmarkObservation = projective_mesher_msgs::msg::LandmarkObservation;
using MesherInput = projective_mesher_msgs::msg::MesherInput;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PolygonMesh = pcl_msgs::msg::PolygonMesh;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Observation {
  int64_t id{-1};
  cv::Point2f keypoint;
  Vec3 world_point;
};

struct MeshVertex {
  Vec3 position;
  Vec3 normal;
  std::array<float, 2> uv;
};

struct MeshData {
  std::vector<MeshVertex> vertices;
  std::vector<std::array<uint32_t, 3>> triangles;
};

Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
  return {
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x,
  };
}

double norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

std::optional<Vec3> normalized(const Vec3& value) {
  const double value_norm = norm(value);
  if (value_norm <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }
  return Vec3{
      value.x / value_norm,
      value.y / value_norm,
      value.z / value_norm,
  };
}

double distance(const Vec3& lhs, const Vec3& rhs) {
  return norm(lhs - rhs);
}

bool isFinite(const Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool isValidStatus(const uint8_t status) {
  return status == LandmarkObservation::VALID;
}

uint64_t stampToNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ull + stamp.nanosec;
}

Vec3 pointFromMsg(const geometry_msgs::msg::Point& point_msg) {
  return {point_msg.x, point_msg.y, point_msg.z};
}

gtsam::Point3 toGtsamPoint(const Vec3& point) {
  return gtsam::Point3(point.x, point.y, point.z);
}

gtsam::Pose3 poseFromMsg(const geometry_msgs::msg::Pose& pose_msg) {
  const auto& q = pose_msg.orientation;
  const auto& t = pose_msg.position;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                      gtsam::Point3(t.x, t.y, t.z));
}

bool pointInsideImage(const cv::Point2f& point, const cv::Rect2f& image_rect) {
  return point.x >= image_rect.x && point.y >= image_rect.y &&
         image_rect.contains(point);
}

std::optional<cv::Mat> imageMsgToCvMat(const sensor_msgs::msg::Image& image_msg) {
  namespace enc = sensor_msgs::image_encodings;
  if (image_msg.data.empty() || image_msg.width == 0 || image_msg.height == 0) {
    return std::nullopt;
  }

  int type = -1;
  bool convert_rgb_to_bgr = false;
  bool convert_rgba_to_bgra = false;
  if (image_msg.encoding == enc::MONO8) {
    type = CV_8UC1;
  } else if (image_msg.encoding == enc::BGR8) {
    type = CV_8UC3;
  } else if (image_msg.encoding == enc::RGB8) {
    type = CV_8UC3;
    convert_rgb_to_bgr = true;
  } else if (image_msg.encoding == enc::BGRA8) {
    type = CV_8UC4;
  } else if (image_msg.encoding == enc::RGBA8) {
    type = CV_8UC4;
    convert_rgba_to_bgra = true;
  } else {
    return std::nullopt;
  }

  cv::Mat image(static_cast<int>(image_msg.height),
                static_cast<int>(image_msg.width),
                type,
                const_cast<uint8_t*>(image_msg.data.data()),
                image_msg.step);
  if (convert_rgb_to_bgr) {
    cv::Mat bgr_image;
    cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
    return bgr_image;
  }
  if (convert_rgba_to_bgra) {
    cv::Mat bgra_image;
    cv::cvtColor(image, bgra_image, cv::COLOR_RGBA2BGRA);
    return bgra_image;
  }
  return image;
}

}  // namespace

class ProjectiveMesherNode : public rclcpp::Node {
 public:
  ProjectiveMesherNode() : Node("projective_mesher_node") {
    mesh_frame_id_ = declare_parameter<std::string>("mesh_frame_id", "");
    publish_texture_ = declare_parameter<bool>("publish_texture", true);
    point_match_tolerance_px_ =
        declare_parameter<double>("point_match_tolerance_px", 0.25);
    min_ratio_largest_smallest_side_ =
        declare_parameter<double>("min_ratio_btw_largest_smallest_side", 0.2);
    max_triangle_side_ = declare_parameter<double>("max_triangle_side", 2.0);
    min_observations_ = static_cast<size_t>(
        std::max<int64_t>(3, declare_parameter<int>("min_observations", 3)));
    enable_rerun_ = declare_parameter<bool>("enable_rerun", false);
    rerun_host_ = declare_parameter<std::string>(
        "rerun_host", "rerun+http://127.0.0.1:9876/proxy");
    rerun_app_id_ =
        declare_parameter<std::string>("rerun_app_id", "projective_mesher");
    rerun_recording_id_ =
        declare_parameter<std::string>("rerun_recording_id", "");
    rerun_entity_path_ =
        declare_parameter<std::string>("rerun_entity_path", "map/projective_mesher/mesh");
    rerun_camera_entity_path_ = declare_parameter<std::string>(
        "rerun_camera_entity_path", "map/projective_mesher/camera");
    log_rerun_camera_pose_ =
        declare_parameter<bool>("log_rerun_camera_pose", true);

    auto mesh_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    mesh_pub_ = create_publisher<PolygonMesh>("mesh", mesh_qos);
    texture_pub_ =
        create_publisher<sensor_msgs::msg::Image>("debug_mesh_img", mesh_qos);
    input_sub_ = create_subscription<MesherInput>(
        "mesher_input", rclcpp::QoS(10),
        std::bind(&ProjectiveMesherNode::inputCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Projective mesher listening on 'mesher_input'.");
    if (enable_rerun_) {
      initializeRerun();
    }
  }

 private:
  void inputCallback(const MesherInput::SharedPtr msg) {
    const auto observations = collectValidObservations(*msg);
    if (observations.size() < min_observations_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping mesher input with %zu valid observations; need at least %zu.",
          observations.size(), min_observations_);
      return;
    }

    const MeshData mesh_data = buildMesh(*msg, observations);
    if (mesh_data.triangles.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Delaunay meshing produced no valid triangles.");
      return;
    }

    const std::string frame_id = outputFrame(*msg);
    mesh_pub_->publish(buildMeshMessage(mesh_data, *msg, frame_id));

    if (publish_texture_) {
      auto texture_msg = msg->image;
      texture_msg.header.stamp = msg->header.stamp;
      texture_msg.header.frame_id = frame_id;
      texture_pub_->publish(texture_msg);
    }

    logMeshToRerun(mesh_data, *msg);
  }

  std::vector<Observation> collectValidObservations(const MesherInput& msg) const {
    std::vector<Observation> observations;
    if (msg.image.width == 0 || msg.image.height == 0) {
      RCLCPP_ERROR(get_logger(),
                   "MesherInput image width and height must be non-zero.");
      return observations;
    }

    const cv::Rect2f image_rect(
        0.0f, 0.0f, static_cast<float>(msg.image.width),
        static_cast<float>(msg.image.height));
    observations.reserve(msg.observations.size());

    std::unordered_set<int64_t> seen_landmark_ids;
    for (const auto& obs_msg : msg.observations) {
      if (obs_msg.id < 0 || !isValidStatus(obs_msg.status)) {
        continue;
      }
      if (!seen_landmark_ids.insert(obs_msg.id).second) {
        continue;
      }

      const cv::Point2f keypoint(
          static_cast<float>(obs_msg.keypoint.x),
          static_cast<float>(obs_msg.keypoint.y));
      if (!pointInsideImage(keypoint, image_rect)) {
        continue;
      }

      const Vec3 world_point = pointFromMsg(obs_msg.world_point);
      if (!isFinite(world_point)) {
        continue;
      }

      observations.push_back({obs_msg.id, keypoint, world_point});
    }

    return observations;
  }

  MeshData buildMesh(const MesherInput& msg,
                     const std::vector<Observation>& observations) const {
    MeshData mesh_data;
    const cv::Rect2f image_rect(
        0.0f, 0.0f, static_cast<float>(msg.image.width),
        static_cast<float>(msg.image.height));

    cv::Subdiv2D subdiv(image_rect);
    try {
      for (const auto& observation : observations) {
        subdiv.insert(observation.keypoint);
      }
    } catch (const cv::Exception& exception) {
      RCLCPP_ERROR(get_logger(), "OpenCV Delaunay insertion failed: %s",
                   exception.what());
      return mesh_data;
    }

    std::vector<cv::Vec6f> triangles_2d;
    subdiv.getTriangleList(triangles_2d);
    mesh_data.vertices.reserve(triangles_2d.size() * 3u);
    mesh_data.triangles.reserve(triangles_2d.size());

    for (const auto& triangle_2d : triangles_2d) {
      const cv::Point2f p0(triangle_2d[0], triangle_2d[1]);
      const cv::Point2f p1(triangle_2d[2], triangle_2d[3]);
      const cv::Point2f p2(triangle_2d[4], triangle_2d[5]);
      if (!pointInsideImage(p0, image_rect) ||
          !pointInsideImage(p1, image_rect) ||
          !pointInsideImage(p2, image_rect)) {
        continue;
      }

      const auto idx0 = nearestObservationIndex(p0, observations);
      const auto idx1 = nearestObservationIndex(p1, observations);
      const auto idx2 = nearestObservationIndex(p2, observations);
      if (!idx0 || !idx1 || !idx2 || *idx0 == *idx1 || *idx1 == *idx2 ||
          *idx0 == *idx2) {
        continue;
      }

      const Observation& obs0 = observations[*idx0];
      const Observation& obs1 = observations[*idx1];
      const Observation& obs2 = observations[*idx2];
      if (!passesTriangleFilters(obs0.world_point, obs1.world_point,
                                 obs2.world_point)) {
        continue;
      }

      const auto normal =
          normalized(cross(obs1.world_point - obs0.world_point,
                           obs2.world_point - obs0.world_point));
      if (!normal) {
        continue;
      }

      const uint32_t base_idx = static_cast<uint32_t>(mesh_data.vertices.size());
      mesh_data.vertices.push_back(toMeshVertex(obs0, *normal, msg));
      mesh_data.vertices.push_back(toMeshVertex(obs1, *normal, msg));
      mesh_data.vertices.push_back(toMeshVertex(obs2, *normal, msg));
      mesh_data.triangles.push_back({base_idx, base_idx + 1u, base_idx + 2u});
    }

    return mesh_data;
  }

  std::optional<size_t> nearestObservationIndex(
      const cv::Point2f& point,
      const std::vector<Observation>& observations) const {
    const double max_dist_sq =
        point_match_tolerance_px_ * point_match_tolerance_px_;
    double best_dist_sq = max_dist_sq;
    std::optional<size_t> best_idx;

    for (size_t idx = 0; idx < observations.size(); ++idx) {
      const cv::Point2f delta = observations[idx].keypoint - point;
      const double dist_sq =
          static_cast<double>(delta.x) * delta.x +
          static_cast<double>(delta.y) * delta.y;
      if (dist_sq <= best_dist_sq) {
        best_dist_sq = dist_sq;
        best_idx = idx;
      }
    }
    return best_idx;
  }

  bool passesTriangleFilters(const Vec3& p0, const Vec3& p1,
                             const Vec3& p2) const {
    const double d01 = distance(p0, p1);
    const double d12 = distance(p1, p2);
    const double d20 = distance(p2, p0);
    const double min_side = std::min({d01, d12, d20});
    const double max_side = std::max({d01, d12, d20});
    if (max_side <= std::numeric_limits<double>::epsilon()) {
      return false;
    }
    if (min_side / max_side < min_ratio_largest_smallest_side_) {
      return false;
    }
    if (max_triangle_side_ > 0.0 && max_side > max_triangle_side_) {
      return false;
    }
    return true;
  }

  MeshVertex toMeshVertex(const Observation& observation, const Vec3& normal,
                          const MesherInput& msg) const {
    const float denom_x = static_cast<float>(std::max<uint32_t>(1, msg.image.width));
    const float denom_y = static_cast<float>(std::max<uint32_t>(1, msg.image.height));
    return {
        observation.world_point,
        normal,
        {
            std::clamp(observation.keypoint.x / denom_x, 0.0f, 1.0f),
            std::clamp(observation.keypoint.y / denom_y, 0.0f, 1.0f),
        },
    };
  }

  PolygonMesh buildMeshMessage(const MeshData& mesh_data,
                               const MesherInput& input,
                               const std::string& frame_id) const {
    PolygonMesh mesh_msg;
    mesh_msg.header.stamp = input.header.stamp;
    mesh_msg.header.frame_id = frame_id;
    mesh_msg.cloud = buildPointCloudMessage(mesh_data, input, frame_id);
    mesh_msg.polygons.reserve(mesh_data.triangles.size());
    for (const auto& triangle : mesh_data.triangles) {
      pcl_msgs::msg::Vertices polygon;
      polygon.vertices = {triangle[0], triangle[1], triangle[2]};
      mesh_msg.polygons.push_back(std::move(polygon));
    }
    return mesh_msg;
  }

  PointCloud2 buildPointCloudMessage(const MeshData& mesh_data,
                                     const MesherInput& input,
                                     const std::string& frame_id) const {
    PointCloud2 cloud_msg;
    cloud_msg.header.stamp = input.header.stamp;
    cloud_msg.header.frame_id = frame_id;

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
    modifier.resize(mesh_data.vertices.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_normal_x(cloud_msg, "normal_x");
    sensor_msgs::PointCloud2Iterator<float> iter_normal_y(cloud_msg, "normal_y");
    sensor_msgs::PointCloud2Iterator<float> iter_normal_z(cloud_msg, "normal_z");
    sensor_msgs::PointCloud2Iterator<float> iter_u(cloud_msg, "u");
    sensor_msgs::PointCloud2Iterator<float> iter_v(cloud_msg, "v");

    for (size_t idx = 0; idx < mesh_data.vertices.size();
         ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_normal_x, ++iter_normal_y,
                ++iter_normal_z, ++iter_u, ++iter_v) {
      const auto& vertex = mesh_data.vertices[idx];
      *iter_x = static_cast<float>(vertex.position.x);
      *iter_y = static_cast<float>(vertex.position.y);
      *iter_z = static_cast<float>(vertex.position.z);
      *iter_normal_x = static_cast<float>(vertex.normal.x);
      *iter_normal_y = static_cast<float>(vertex.normal.y);
      *iter_normal_z = static_cast<float>(vertex.normal.z);
      *iter_u = vertex.uv[0];
      *iter_v = vertex.uv[1];
    }

    return cloud_msg;
  }

  std::string outputFrame(const MesherInput& input) const {
    if (!mesh_frame_id_.empty()) {
      return mesh_frame_id_;
    }
    return input.header.frame_id.empty() ? "world" : input.header.frame_id;
  }

  void initializeRerun() {
    std::optional<std::string> recording_id = std::nullopt;
    if (!rerun_recording_id_.empty()) {
      recording_id = rerun_recording_id_;
    }

    aria::viz::VisualizerRerun::Params rerun_params(
        rerun_app_id_, recording_id, rerun_host_);
    rerun_visualizer_ =
        std::make_unique<aria::viz::VisualizerRerun>(rerun_params);

    RCLCPP_INFO(get_logger(), "Enabled aria_viz Rerun logging at '%s'.",
                rerun_host_.c_str());
  }

  void logMeshToRerun(const MeshData& mesh_data, const MesherInput& input) {
    if (!rerun_visualizer_) {
      return;
    }

    rerun_visualizer_->setTimeNSec(stampToNanoseconds(input.header.stamp));

    if (log_rerun_camera_pose_) {
      rerun_visualizer_->drawTf(rerun_camera_entity_path_,
                                poseFromMsg(input.camera_pose), 0.3, false);
    }

    std::vector<gtsam::Point3> vertex_positions;
    std::vector<gtsam::Point3> vertex_normals;
    std::vector<aria::viz::MeshTexcoord> vertex_texcoords;
    vertex_positions.reserve(mesh_data.vertices.size());
    vertex_normals.reserve(mesh_data.vertices.size());
    vertex_texcoords.reserve(mesh_data.vertices.size());
    for (const auto& vertex : mesh_data.vertices) {
      vertex_positions.push_back(toGtsamPoint(vertex.position));
      vertex_normals.push_back(toGtsamPoint(vertex.normal));
      vertex_texcoords.push_back(vertex.uv);
    }

    std::vector<aria::viz::MeshTriangle> triangle_indices;
    triangle_indices.reserve(mesh_data.triangles.size());
    for (const auto& triangle : mesh_data.triangles) {
      triangle_indices.push_back(triangle);
    }

    const auto texture = imageMsgToCvMat(input.image);
    if (texture) {
      rerun_visualizer_->drawTexturedMesh(rerun_entity_path_,
                                          vertex_positions,
                                          triangle_indices,
                                          vertex_texcoords,
                                          *texture,
                                          {},
                                          vertex_normals,
                                          false);
    } else {
      rerun_visualizer_->drawMesh(rerun_entity_path_,
                                  vertex_positions,
                                  triangle_indices,
                                  {},
                                  vertex_normals,
                                  false);
    }
  }

  std::string mesh_frame_id_;
  bool publish_texture_{true};
  bool enable_rerun_{false};
  bool log_rerun_camera_pose_{true};
  double point_match_tolerance_px_{0.25};
  double min_ratio_largest_smallest_side_{0.2};
  double max_triangle_side_{2.0};
  size_t min_observations_{3};
  std::string rerun_host_;
  std::string rerun_app_id_;
  std::string rerun_recording_id_;
  std::string rerun_entity_path_;
  std::string rerun_camera_entity_path_;
  aria::viz::VisualizerRerun::UniquePtr rerun_visualizer_;
  rclcpp::Subscription<MesherInput>::SharedPtr input_sub_;
  rclcpp::Publisher<PolygonMesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr texture_pub_;
};

}  // namespace projective_mesher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<projective_mesher_ros::ProjectiveMesherNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
