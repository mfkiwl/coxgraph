#include "coxgraph/map_comm/mesh_converter.h"

#include <limits>

namespace voxblox {

MeshConverter::Config MeshConverter::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<float>("interpolate_ratio", config.interpolate_ratio,
                          config.interpolate_ratio);
  return config;
}

// Stolen from voxblox rviz plugin
bool MeshConverter::convertToPointCloud() {
  if (mesh_.mesh_blocks.empty()) return false;
  timing::Timer recovered_poincloud_timer("recover_pointcloud");
  pointcloud_->clear();
  Pointcloud triangle;
  for (auto const& mesh_block : mesh_.mesh_blocks) {
    const BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                           mesh_block.index[2]);

    // translate vertex data from message to voxblox mesh
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      // Each vertex is given as its distance from the blocks origin in units of
      // (2*block_size), see mesh_vis.h for the slightly convoluted
      // justification of the 2.
      constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      const float mesh_x =
          (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
           static_cast<float>(index[0])) *
          mesh_.block_edge_length;
      const float mesh_y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(index[1])) *
          mesh_.block_edge_length;
      const float mesh_z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(index[2])) *
          mesh_.block_edge_length;

      pointcloud_->emplace_back(mesh_x, mesh_y, mesh_z);
      triangle.emplace_back(mesh_x, mesh_y, mesh_z);
      if (config_.interpolate_ratio > 0 && triangle.size() == 3) {
        for (int i = 0; i < 3 * std::round(config_.interpolate_ratio); i++) {
          Point p0 = triangle[0], p1 = triangle[1], p2 = triangle[2];
          Point t_p0_p1 = p1 - p0;
          float r1 = std::rand() % 100 / 100.0;
          Point interp_pt = p0 + r1 * t_p0_p1;

          Point t_p2_interp = p2 - interp_pt;
          float r2 = std::rand() % 100 / 100.0;
          interp_pt += r2 * t_p2_interp;
          pointcloud_->emplace_back(interp_pt);
        }
      }
    }
  }

  recovered_poincloud_timer.Stop();

  return true;
}

}  // namespace voxblox
