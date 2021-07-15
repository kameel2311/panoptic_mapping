#include "panoptic_mapping_ros/visualization/single_tsdf_visualizer.h"

#include <memory>
#include <utility>
#include <vector>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<
    SubmapVisualizer, SingleTsdfVisualizer, std::shared_ptr<Globals>>
    SingleTsdfVisualizer::registration_("single_tsdf");

void SingleTsdfVisualizer::Config::checkParams() const {
  checkParamConfig(submap_visualizer_config);
}

void SingleTsdfVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("submap_visualizer_config", &submap_visualizer_config);
}

SingleTsdfVisualizer::SingleTsdfVisualizer(const Config& config,
                                           std::shared_ptr<Globals> globals,
                                           bool print_config)
    : config_(config.checkValid()),
      SubmapVisualizer(config_.submap_visualizer_config, std::move(globals),
                       false) {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
}

void SingleTsdfVisualizer::reset() {
  // Erase all current tracking / cached data.
  info_ = SubmapVisInfo();
  info_.republish_everything = true;
  previous_submaps_ = nullptr;
}

void SingleTsdfVisualizer::clearMesh() {
  // Clear the current mesh from the rviz plugin.
  if (config_.submap_visualizer_config.visualize_mesh &&
      mesh_pub_.getNumSubscribers() > 0) {
    voxblox_msgs::MultiMesh msg;
    msg.header.stamp = ros::Time::now();
    msg.name_space = map_name_space_;
    mesh_pub_.publish(msg);
  }
}

std::vector<voxblox_msgs::MultiMesh> SingleTsdfVisualizer::generateMeshMsgs(
    SubmapCollection* submaps) {
  std::vector<voxblox_msgs::MultiMesh> result;

  // Get the single map.
  Submap& submap =
      *submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());

  // Setup message.
  voxblox_msgs::MultiMesh msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = submap.getFrameName();
  msg.name_space = map_name_space_;

  // Mark the whole mesh for re-publishing if requested.
  if (info_.republish_everything) {
    voxblox::BlockIndexList mesh_indices;
    submap.getMeshLayer().getAllAllocatedMeshes(&mesh_indices);
    for (const auto& block_index : mesh_indices) {
      submap.getMeshLayerPtr()->getMeshPtrByIndex(block_index)->updated = true;
    }
    info_.republish_everything = false;
  }

  // Set the voxblox internal color mode. Gray will be used for overwriting.
  voxblox::ColorMode color_mode_voxblox = voxblox::ColorMode::kGray;
  if (color_mode_ == ColorMode::kColor) {
    color_mode_voxblox = voxblox::ColorMode::kColor;
  } else if (color_mode_ == ColorMode::kNormals) {
    color_mode_voxblox = voxblox::ColorMode::kNormals;
  }

  voxblox::generateVoxbloxMeshMsg(submap.getMeshLayerPtr(), color_mode_voxblox,
                                  &msg.mesh);

  // Add removed blocks so they are cleared from the visualization as well.
  voxblox::BlockIndexList block_indices;
  submap.getTsdfLayer().getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : info_.previous_blocks) {
    if (std::find(block_indices.begin(), block_indices.end(), block_index) ==
        block_indices.end()) {
      voxblox_msgs::MeshBlock mesh_block;
      mesh_block.index[0] = block_index.x();
      mesh_block.index[1] = block_index.y();
      mesh_block.index[2] = block_index.z();
      msg.mesh.mesh_blocks.push_back(mesh_block);
    }
  }
  info_.previous_blocks = block_indices;

  if (msg.mesh.mesh_blocks.empty()) {
    // Nothing changed, don't send an empty msg which would reset the mesh.
    return result;
  }

  // Apply the submap color if necessary.
  if (color_mode_voxblox == voxblox::ColorMode::kGray) {
    if (!submap.hasClassLayer()) {
      LOG_IF(WARNING, config_.verbosity >= 2)
          << "Can not create color for mode '" << colorModeToString(color_mode_)
          << "' without existing class layer.";
    } else {
      for (auto& mesh_block : msg.mesh.mesh_blocks) {
        colorMeshBlock(submap, &mesh_block);
      }
    }
  }

  result.emplace_back(std::move(msg));
  return result;
}

void SingleTsdfVisualizer::colorMeshBlock(const Submap& submap,
                                          voxblox_msgs::MeshBlock* mesh_block) {
}

void SingleTsdfVisualizer::updateVisInfos(const SubmapCollection& submaps) {
  // Check whether the same submap collection is being visualized (cached data).
  if (previous_submaps_ != &submaps) {
    reset();
    previous_submaps_ = &submaps;
  }
}

void SingleTsdfVisualizer::setVisualizationMode(
    VisualizationMode visualization_mode) {
  // If there is a new visualization mode recompute the colors and
  // republish everything.
  if (visualization_mode == visualization_mode_) {
    return;
  }
  if (visualization_mode != VisualizationMode::kAll) {
    LOG(WARNING) << "Visualization mode '"
                 << visualizationModeToString(visualization_mode)
                 << "' is not supported, using 'all' instead.";
    visualization_mode_ = VisualizationMode::kAll;
  } else {
    visualization_mode_ = visualization_mode;
  }
  reset();
}

void SingleTsdfVisualizer::setColorMode(ColorMode color_mode) {
  // If there is a new color mode recompute the colors.
  if (color_mode == color_mode_) {
    return;
  }
  if (color_mode == ColorMode::kChange ||
      color_mode == ColorMode::kClassification) {
    LOG(WARNING) << "Color mode '" << colorModeToString(color_mode)
                 << "' is not supported, using 'color' instead.";
    color_mode_ = ColorMode::kColor;
  } else {
    color_mode_ = color_mode;
  }
  reset();
}

}  // namespace panoptic_mapping
