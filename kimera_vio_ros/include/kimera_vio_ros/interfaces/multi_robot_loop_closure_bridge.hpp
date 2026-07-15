/**
 * @file multi_robot_loop_closure_bridge.hpp
 * @brief ROS 2 transport bridge for multi-robot loop closure inputs.
 */

#pragma once

#include <map>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <tuple>
#include <vector>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <kimera-vio/loopclosure/LcdOutputPacket.h>
#include <pose_graph_tools_msgs/msg/bow_queries.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <pose_graph_tools_msgs/msg/vlc_frames.hpp>
#include <pose_graph_tools_msgs/srv/pose_graph_query.hpp>
#include <pose_graph_tools_msgs/srv/vlc_frame_query.hpp>
#include <rclcpp/rclcpp.hpp>

namespace kimera_vio_ros::interfaces {

class MultiRobotLoopClosureBridge {
 public:
  explicit MultiRobotLoopClosureBridge(const rclcpp::Node::SharedPtr& node);
  ~MultiRobotLoopClosureBridge();

  void publishLcdOutput(const VIO::LcdOutput::ConstPtr& lcd_output);
  void flush();
  bool enabled() const { return enabled_; }

 private:
  using BowQueriesMsg = pose_graph_tools_msgs::msg::BowQueries;
  using PoseGraphMsg = pose_graph_tools_msgs::msg::PoseGraph;
  using PoseGraphEdgeMsg = pose_graph_tools_msgs::msg::PoseGraphEdge;
  using PoseGraphNodeMsg = pose_graph_tools_msgs::msg::PoseGraphNode;
  using VLCFrameMsg = pose_graph_tools_msgs::msg::VLCFrameMsg;
  using VLCFramesMsg = pose_graph_tools_msgs::msg::VLCFrames;
  using PoseGraphQuerySrv = pose_graph_tools_msgs::srv::PoseGraphQuery;
  using VLCFrameQuerySrv = pose_graph_tools_msgs::srv::VLCFrameQuery;
  using EdgeId = std::tuple<uint64_t, uint64_t, int32_t>;

  struct CachedFrame {
    uint64_t timestamp_ns{0};
    VIO::KeypointsCV keypoints_2d;
    VIO::Landmarks keypoints_3d;
    VIO::BearingVectors versors;
    decltype(VIO::LcdOutput::bow_vec_) bow_vec;
    VIO::OrbDescriptor descriptors_mat;
    gtsam::Pose3 T_base_cam;
    std::vector<float> scores;

    explicit CachedFrame(const VIO::LcdOutput& output);
  };

  void updatePoseGraph(const VIO::LcdOutput& output,
                       PoseGraphMsg* incremental);
  void queueDescriptor(VIO::FrameId frame_id, const CachedFrame& frame);
  void queueVerificationFrame(VIO::FrameId frame_id);
  bool getFrameMsg(VIO::FrameId frame_id, VLCFrameMsg* msg) const;
  PoseGraphMsg getPoseGraphMsg() const;
  void flushLocked(bool force);

  void poseGraphService(
      const std::shared_ptr<PoseGraphQuerySrv::Request> request,
      std::shared_ptr<PoseGraphQuerySrv::Response> response);
  void frameService(const std::shared_ptr<VLCFrameQuerySrv::Request> request,
                    std::shared_ptr<VLCFrameQuerySrv::Response> response);

  static void poseToMsg(const gtsam::Pose3& pose,
                        geometry_msgs::msg::Pose* msg);

  rclcpp::Node::SharedPtr node_;
  bool enabled_{false};
  uint16_t robot_id_{0};
  size_t descriptor_batch_size_{5};
  size_t descriptor_stride_{1};
  size_t verification_frame_batch_size_{50};
  bool publish_verification_frames_{true};
  double flush_period_s_{1.0};
  std::string map_frame_id_;

  rclcpp::Publisher<PoseGraphMsg>::SharedPtr pose_graph_pub_;
  rclcpp::Publisher<BowQueriesMsg>::SharedPtr descriptor_pub_;
  rclcpp::Publisher<VLCFramesMsg>::SharedPtr frame_pub_;
  rclcpp::Service<PoseGraphQuerySrv>::SharedPtr pose_graph_service_;
  rclcpp::Service<VLCFrameQuerySrv>::SharedPtr frame_service_;
  rclcpp::TimerBase::SharedPtr flush_timer_;

  std::map<VIO::FrameId, CachedFrame> frames_;
  std::map<uint64_t, PoseGraphNodeMsg> nodes_;
  std::map<EdgeId, PoseGraphEdgeMsg> edges_;
  std::set<uint64_t> sent_nodes_;
  std::set<EdgeId> sent_edges_;
  std::set<VIO::FrameId> queued_descriptor_ids_;
  std::set<VIO::FrameId> queued_verification_frame_ids_;
  BowQueriesMsg pending_descriptors_;
  VLCFramesMsg pending_frames_;
  mutable std::mutex mutex_;
};

}  // namespace kimera_vio_ros::interfaces
