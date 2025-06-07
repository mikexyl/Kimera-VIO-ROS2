#pragma once

#include <aria_viz/visualizer_rerun.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <rclcpp/logging.hpp>

namespace kimera_vio_ros {

class RerunVisualizer : public VIO::Visualizer3D, aria::viz::VisualizerRerun {
 public:
  RerunVisualizer(std::string base_link_frame_id,
                  std::string map_frame_id,
                  std::string world_frame_id)
      : VIO::Visualizer3D(VIO::VisualizationType::kNone),
        aria::viz::VisualizerRerun(
            aria::viz::VisualizerRerun::Params("kimera_vio")),
        base_link_frame_id_(base_link_frame_id),
        map_frame_id_(map_frame_id),
        world_frame_id_(world_frame_id) {}

  virtual ~RerunVisualizer() = default;

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    this->setTimeNSec(input.timestamp_);
    std::filesystem::path entity_path(world_frame_id_);
    entity_path /= map_frame_id_;
    entity_path /= base_link_frame_id_;
    this->drawTf(
        entity_path, input.backend_output_->W_State_Blkf_.pose_, false);

    return std::make_unique<VIO::VisualizerOutput>();
  }

 private:
  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string world_frame_id_;
};

}  // namespace kimera_vio_ros