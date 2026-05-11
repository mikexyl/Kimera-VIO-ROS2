/**
 * @file   ros_loop_closure_visualizer.hpp
 * @brief  Publishes loop closure and pose graph data to ROS 2.
 */

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pose_graph_tools_msgs/msg/bow_queries.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_edge.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_node.hpp>
#include <pose_graph_tools_msgs/msg/vlc_frames.hpp>
#include <pose_graph_tools_msgs/srv/vlc_frame_query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace kimera_vio_ros {
namespace interfaces {

class RosLoopClosureVisualizer {
 public:
  explicit RosLoopClosureVisualizer(const rclcpp::Node::SharedPtr& node);
  ~RosLoopClosureVisualizer() = default;

  void publishLcdOutput(const VIO::LcdOutput::ConstPtr& lcd_output);

 private:
  using BowQueriesMsg = pose_graph_tools_msgs::msg::BowQueries;
  using PoseGraphMsg = pose_graph_tools_msgs::msg::PoseGraph;
  using PoseGraphEdgeMsg = pose_graph_tools_msgs::msg::PoseGraphEdge;
  using PoseGraphNodeMsg = pose_graph_tools_msgs::msg::PoseGraphNode;
  using VLCFrameMsg = pose_graph_tools_msgs::msg::VLCFrameMsg;
  using VLCFramesMsg = pose_graph_tools_msgs::msg::VLCFrames;
  using VLCFrameQuerySrv = pose_graph_tools_msgs::srv::VLCFrameQuery;

  struct LcdFrame {
    uint64_t timestamp_ns = 0;
    VIO::KeypointsCV keypoints_2d;
    VIO::Landmarks keypoints_3d;
    VIO::BearingVectors versors;
    decltype(VIO::LcdOutput::bow_vec_) bow_vec;
    VIO::OrbDescriptor descriptors_mat;
    gtsam::Pose3 T_base_cam;
    std::vector<float> scores;

    explicit LcdFrame(const VIO::LcdOutput& lcd_output);
  };

  void publishTf(const VIO::LcdOutput::ConstPtr& lcd_output);
  void publishOptimizedTrajectory(const VIO::LcdOutput::ConstPtr& lcd_output);
  void publishPoseGraph(const VIO::LcdOutput::ConstPtr& lcd_output);
  void updateNodesAndEdges(const VIO::FrameIDTimestampMap& times,
                           const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values);
  void updateRejectedEdges();
  PoseGraphMsg getPosegraphMsg() const;
  void processBowQuery();
  void publishTimerCallback();
  void VLCServiceCallback(
      const std::shared_ptr<VLCFrameQuerySrv::Request> request,
      std::shared_ptr<VLCFrameQuerySrv::Response> response);
  bool getFrameMsg(size_t pose_id, VLCFrameMsg* frame_msg) const;

  static void poseToMsg(const gtsam::Pose3& pose, geometry_msgs::msg::Pose* msg);

 private:
  rclcpp::Node::SharedPtr node_;

  uint16_t robot_id_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PoseGraphMsg>::SharedPtr posegraph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<PoseGraphMsg>::SharedPtr posegraph_incremental_pub_;
  rclcpp::Publisher<BowQueriesMsg>::SharedPtr bow_query_pub_;
  rclcpp::Publisher<VLCFramesMsg>::SharedPtr vlc_frame_pub_;
  rclcpp::Service<VLCFrameQuerySrv>::SharedPtr vlc_frame_server_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<PoseGraphEdgeMsg> loop_closure_edges_;
  std::vector<PoseGraphEdgeMsg> odometry_edges_;
  std::vector<PoseGraphEdgeMsg> inlier_edges_;
  std::vector<PoseGraphNodeMsg> pose_graph_nodes_;

  std::vector<LcdFrame> frames_;

  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;

  int bow_batch_size_;
  int bow_skip_num_;
  bool publish_vlc_frames_;

  std::map<uint16_t, BowQueriesMsg> bow_queries_;
  VLCFramesMsg new_frames_msg_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  mutable std::mutex mutex_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros
