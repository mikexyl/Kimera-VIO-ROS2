#include "kimera_vio_ros/interfaces/multi_robot_loop_closure_bridge.hpp"

#include <gtest/gtest.h>

#include <gtsam/slam/BetweenFactor.h>

#include <chrono>
#include <memory>
#include <thread>

namespace kimera_vio_ros::interfaces {
namespace {

using namespace std::chrono_literals;
using BowQueries = pose_graph_tools_msgs::msg::BowQueries;
using JistRefinementBundles =
    pose_graph_tools_msgs::msg::JistRefinementBundles;
using PoseGraph = pose_graph_tools_msgs::msg::PoseGraph;
using VLCFrames = pose_graph_tools_msgs::msg::VLCFrames;

VIO::LcdOutput::Ptr makeOutput(VIO::FrameId frame_id,
                               bool include_graph = false) {
  const VIO::Timestamp timestamp = 1000000000 + frame_id;
  auto output = std::make_shared<VIO::LcdOutput>(timestamp);
  output->keyframe_id_ = frame_id;
  output->timestamp_kf_ = timestamp;
  output->bow_vec_ = {{0, 0.25}, {1, 0.75}};
  output->keypoints_2d_ = {cv::Point2f(10.0f, 20.0f)};
  output->keypoints_3d_ = {gtsam::Point3(0.5, 1.0, 2.0)};
  output->versors_ = {gtsam::Vector3(0.25, 0.5, 1.0)};
  output->landmark_ids_ = {1234};
  output->descriptors_mat_ = (cv::Mat_<float>(1, 4) << 1.0f, 2.0f, 3.0f, 4.0f);
  output->T_base_cam_ = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3),
                                     gtsam::Point3(0.4, 0.5, 0.6));
  if (include_graph) {
    output->states_.insert(0, gtsam::Pose3());
    output->states_.insert(
        1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
    output->timestamp_map_[0] = timestamp - 1;
    output->timestamp_map_[1] = timestamp;
    output->nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        0, 1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)),
        gtsam::noiseModel::Isotropic::Variance(6, 0.01)));
  }
  return output;
}

VIO::LcdVerificationFrame makeVerificationFrame(VIO::FrameId frame_id) {
  const auto output = makeOutput(frame_id);
  VIO::LcdVerificationFrame frame;
  frame.frame_id = frame_id;
  frame.timestamp = output->timestamp_kf_;
  frame.keypoints_2d = output->keypoints_2d_;
  frame.keypoints_3d = output->keypoints_3d_;
  frame.versors = output->versors_;
  frame.landmark_ids = output->landmark_ids_;
  frame.descriptors_mat = output->descriptors_mat_.clone();
  frame.T_base_cam = output->T_base_cam_;
  return frame;
}

rclcpp::NodeOptions enabledOptions(int descriptor_batch = 1,
                                   int frame_batch = 1,
                                   double flush_period = 1.0) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("multi_robot_bridge.enabled", true);
  options.append_parameter_override("multi_robot_bridge.descriptor_batch_size",
                                    descriptor_batch);
  options.append_parameter_override("multi_robot_bridge.descriptor_stride", 1);
  options.append_parameter_override(
      "multi_robot_bridge.verification_frame_batch_size", frame_batch);
  options.append_parameter_override(
      "multi_robot_bridge.publish_verification_frames", true);
  options.append_parameter_override("multi_robot_bridge.flush_period_s",
                                    flush_period);
  options.append_parameter_override("robot_id", 2);
  options.append_parameter_override("frame_id.map", "alpha/map");
  return options;
}

template <typename Predicate>
bool spinUntil(rclcpp::executors::SingleThreadedExecutor &executor,
               Predicate predicate) {
  for (size_t attempt = 0; attempt < 100; ++attempt) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

class MultiRobotBridgeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(MultiRobotBridgeTest, UsesExplicitGappedKeyframeIdAndNamespace) {
  auto observer = std::make_shared<rclcpp::Node>("observer");
  BowQueries descriptor;
  VLCFrames frames;
  auto descriptor_sub = observer->create_subscription<BowQueries>(
      "/alpha/kimera_vio/descriptors/global",
      rclcpp::QoS(10).reliable().transient_local(),
      [&](const BowQueries::SharedPtr msg) { descriptor = *msg; });
  auto frame_sub = observer->create_subscription<VLCFrames>(
      "/alpha/kimera_vio/frames/verification", rclcpp::QoS(10).reliable(),
      [&](const VLCFrames::SharedPtr msg) { frames = *msg; });
  auto node = std::make_shared<rclcpp::Node>("bridge", "/alpha/kimera_vio",
                                             enabledOptions());
  MultiRobotLoopClosureBridge bridge(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor, [&]() {
    return node->count_subscribers("descriptors/global") >= 1 &&
           node->count_subscribers("frames/verification") >= 1;
  }));
  bridge.publishLcdOutput(makeOutput(7));
  ASSERT_TRUE(spinUntil(executor, [&]() {
    return descriptor.queries.size() == 1 && frames.frames.size() == 1;
  }));

  ASSERT_EQ(descriptor.queries.size(), 1u);
  EXPECT_EQ(descriptor.queries.front().pose_id, 7u);
  EXPECT_EQ(descriptor.queries.front().robot_id, 2u);
  EXPECT_TRUE(descriptor.queries.front().bow_vector.word_ids.empty());
  EXPECT_EQ(descriptor.queries.front().bow_vector.word_values.size(), 2u);
  ASSERT_EQ(frames.frames.size(), 1u);
  EXPECT_EQ(frames.frames.front().pose_id, 7u);
  EXPECT_EQ(frames.frames.front().depths.front(), 2.0f);
  ASSERT_EQ(frames.frames.front().landmark_ids.size(), 1u);
  EXPECT_EQ(frames.frames.front().landmark_ids.front(), 1234);
  EXPECT_FALSE(frames.frames.front().descriptors_mat.data.empty());
  EXPECT_NEAR(frames.frames.front().t_base_cam.position.x, 0.4, 1e-9);
}

TEST_F(MultiRobotBridgeTest, DeduplicatesUpdatesAndServesFullSnapshots) {
  auto observer = std::make_shared<rclcpp::Node>("observer_graph");
  std::vector<PoseGraph> updates;
  auto graph_sub = observer->create_subscription<PoseGraph>(
      "/alpha/kimera_vio/pose_graph/updates", rclcpp::QoS(10).reliable(),
      [&](const PoseGraph::SharedPtr msg) { updates.push_back(*msg); });
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_graph", "/alpha/kimera_vio", enabledOptions());
  MultiRobotLoopClosureBridge bridge(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);

  auto output = makeOutput(11, true);
  bridge.publishLcdOutput(output);
  bridge.publishLcdOutput(output);
  executor.spin_some();
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates.front().edges.size(), 1u);
  EXPECT_EQ(updates.front().nodes.size(), 2u);

  auto client =
      observer->create_client<pose_graph_tools_msgs::srv::PoseGraphQuery>(
          "/alpha/kimera_vio/pose_graph/get");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request =
      std::make_shared<pose_graph_tools_msgs::srv::PoseGraphQuery::Request>();
  request->robot_id = 2;
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  EXPECT_EQ(response->pose_graph.edges.size(), 1u);
  EXPECT_EQ(response->pose_graph.nodes.size(), 2u);
}

TEST_F(MultiRobotBridgeTest, RepublishesRevisedOdometryFactors) {
  auto observer = std::make_shared<rclcpp::Node>("observer_revised_graph");
  std::vector<PoseGraph> updates;
  auto graph_sub = observer->create_subscription<PoseGraph>(
      "/alpha/kimera_vio/pose_graph/updates", rclcpp::QoS(10).reliable(),
      [&](const PoseGraph::SharedPtr msg) { updates.push_back(*msg); });
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_revised_graph", "/alpha/kimera_vio", enabledOptions());
  MultiRobotLoopClosureBridge bridge(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor, [&]() {
    return node->count_subscribers("pose_graph/updates") >= 1;
  }));

  auto output = makeOutput(12, true);
  bridge.publishLcdOutput(output);
  ASSERT_TRUE(spinUntil(executor, [&]() { return updates.size() == 1; }));
  output->nfg_ = gtsam::NonlinearFactorGraph();
  output->nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      0, 1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.25, 0.0, 0.0)),
      gtsam::noiseModel::Isotropic::Variance(6, 0.01)));
  bridge.publishLcdOutput(output);
  ASSERT_TRUE(spinUntil(executor, [&]() { return updates.size() == 2; }));
  output->states_.clear();
  output->states_.insert(
      0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.5, 0.0, 0.0)));
  output->states_.insert(
      1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.75, 0.0, 0.0)));
  bridge.publishLcdOutput(output);
  ASSERT_TRUE(spinUntil(executor, [&]() { return updates.size() == 3; }));

  ASSERT_EQ(updates.size(), 3u);
  ASSERT_EQ(updates.back().edges.size(), 1u);
  EXPECT_TRUE(updates.back().nodes.empty());
  EXPECT_NEAR(updates.back().edges.front().pose.position.x, 1.25, 1e-9);

  auto client =
      observer->create_client<pose_graph_tools_msgs::srv::PoseGraphQuery>(
          "/alpha/kimera_vio/pose_graph/get");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request =
      std::make_shared<pose_graph_tools_msgs::srv::PoseGraphQuery::Request>();
  request->robot_id = 2;
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  ASSERT_EQ(response->pose_graph.edges.size(), 1u);
  EXPECT_NEAR(response->pose_graph.edges.front().pose.position.x, 1.25, 1e-9);
}

TEST_F(MultiRobotBridgeTest, FlushesPartialBatchesOnTimer) {
  auto observer = std::make_shared<rclcpp::Node>("observer_timer");
  size_t descriptor_count = 0;
  auto descriptor_sub = observer->create_subscription<BowQueries>(
      "/alpha/kimera_vio/descriptors/global",
      rclcpp::QoS(10).reliable().transient_local(),
      [&](const BowQueries::SharedPtr msg) {
        descriptor_count += msg->queries.size();
      });
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_timer", "/alpha/kimera_vio", enabledOptions(10, 10, 0.02));
  MultiRobotLoopClosureBridge bridge(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);
  bridge.publishLcdOutput(makeOutput(13));
  for (size_t attempt = 0; attempt < 20 && descriptor_count == 0; ++attempt) {
    std::this_thread::sleep_for(10ms);
    executor.spin_some();
  }
  EXPECT_EQ(descriptor_count, 1u);
}

TEST_F(MultiRobotBridgeTest,
       PublishesFp32RefinementMatrixAndServesSelectedFrames) {
  auto observer = std::make_shared<rclcpp::Node>("observer_refinement");
  JistRefinementBundles received;
  auto refinement_sub = observer->create_subscription<JistRefinementBundles>(
      "/alpha/kimera_vio/descriptors/jist_refinement",
      rclcpp::QoS(10).reliable().transient_local(),
      [&](const JistRefinementBundles::SharedPtr msg) { received = *msg; });
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_refinement", "/alpha/kimera_vio", enabledOptions());
  MultiRobotLoopClosureBridge bridge(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);

  auto output = makeOutput(30);
  VIO::JistRefinementBundle bundle;
  bundle.sequence_endpoint_id = 30;
  bundle.frame_ids = {26, 26, 28, 29, 30};
  bundle.frame_descriptors = cv::Mat::zeros(5, 512, CV_32FC1);
  for (int row = 0; row < bundle.frame_descriptors.rows; ++row) {
    bundle.frame_descriptors.at<float>(row, row) = 1.0f;
  }
  for (const VIO::FrameId selected_id : {26, 28, 29, 30}) {
    bundle.verification_frames.push_back(
        makeVerificationFrame(selected_id));
  }
  output->jist_refinement_bundle = std::move(bundle);
  bridge.publishLcdOutput(output);

  ASSERT_TRUE(spinUntil(executor, [&]() {
    return received.bundles.size() == 1u;
  }));
  const auto& message = received.bundles.front();
  EXPECT_EQ(message.robot_id, 2u);
  EXPECT_EQ(message.sequence_pose_id, 30u);
  EXPECT_EQ(message.frame_ids,
            (std::vector<uint32_t>{26, 26, 28, 29, 30}));
  ASSERT_EQ(message.descriptor_dim, 512u);
  ASSERT_EQ(message.frame_descriptors.size(), 5u * 512u);
  for (size_t row = 0; row < 5u; ++row) {
    EXPECT_FLOAT_EQ(message.frame_descriptors[row * 512u + row], 1.0f);
  }

  auto client = observer->create_client<
      pose_graph_tools_msgs::srv::VLCFrameQuery>(
      "/alpha/kimera_vio/frames/verification/get");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request =
      std::make_shared<pose_graph_tools_msgs::srv::VLCFrameQuery::Request>();
  request->robot_id = 2;
  request->pose_ids = {26, 28};
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  ASSERT_EQ(response->frames.size(), 2u);
  EXPECT_EQ(response->frames[0].pose_id, 26u);
  EXPECT_EQ(response->frames[1].pose_id, 28u);
}

TEST_F(MultiRobotBridgeTest, SkipsNonSequenceOutputs) {
  auto observer = std::make_shared<rclcpp::Node>("observer_empty_descriptor");
  size_t descriptor_count = 0;
  size_t frame_count = 0;
  auto descriptor_sub = observer->create_subscription<BowQueries>(
      "/alpha/kimera_vio/descriptors/global",
      rclcpp::QoS(10).reliable().transient_local(),
      [&](const BowQueries::SharedPtr msg) {
        descriptor_count += msg->queries.size();
      });
  auto frame_sub = observer->create_subscription<VLCFrames>(
      "/alpha/kimera_vio/frames/verification", rclcpp::QoS(10).reliable(),
      [&](const VLCFrames::SharedPtr msg) {
        frame_count += msg->frames.size();
      });
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_empty_descriptor", "/alpha/kimera_vio", enabledOptions());
  MultiRobotLoopClosureBridge bridge(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);

  auto output = makeOutput(19);
  output->bow_vec_.clear();
  bridge.publishLcdOutput(output);
  executor.spin_some();

  EXPECT_EQ(descriptor_count, 0u);
  EXPECT_EQ(frame_count, 0u);
}

TEST_F(MultiRobotBridgeTest, RejectsIncompleteSequenceVerificationFrame) {
  auto observer = std::make_shared<rclcpp::Node>("observer_incomplete_frame");
  size_t frame_count = 0;
  auto frame_sub = observer->create_subscription<VLCFrames>(
      "/alpha/kimera_vio/frames/verification", rclcpp::QoS(10).reliable(),
      [&](const VLCFrames::SharedPtr msg) {
        frame_count += msg->frames.size();
      });
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_incomplete_frame", "/alpha/kimera_vio", enabledOptions());
  MultiRobotLoopClosureBridge bridge(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(node);

  auto output = makeOutput(23);
  output->descriptors_mat_.release();
  bridge.publishLcdOutput(output);
  executor.spin_some();

  EXPECT_EQ(frame_count, 0u);
}

TEST_F(MultiRobotBridgeTest, DisabledBridgeCreatesNoEndpoints) {
  auto node = std::make_shared<rclcpp::Node>(
      "bridge_disabled", "/alpha/kimera_vio", rclcpp::NodeOptions());
  MultiRobotLoopClosureBridge bridge(node);
  EXPECT_FALSE(bridge.enabled());
  EXPECT_EQ(node->count_publishers("pose_graph/updates"), 0u);
  bridge.publishLcdOutput(makeOutput(1, true));
}

} // namespace
} // namespace kimera_vio_ros::interfaces
