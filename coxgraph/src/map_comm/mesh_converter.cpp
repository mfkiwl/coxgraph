#include "coxgraph/map_comm/mesh_converter.h"

#include <limits>

namespace voxblox {

MeshConverter::Config MeshConverter::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<float>("voxel_size", config.voxel_size, config.voxel_size);
  return config;
}

// Stolen from voxblox rviz plugin
bool MeshConverter::convertToPointCloud() {
  if (mesh_.mesh_blocks.empty()) return false;
  timing::Timer recovered_poincloud_timer("recover_pointcloud");
  Pointcloud triangle;
  LOG(INFO) << "mesh size: " << mesh_.mesh_blocks.size();
  int n = 0;
  for (auto const& mesh_block : mesh_.mesh_blocks) {
    if (mesh_block.history.empty()) continue;
    CHECK_EQ(mesh_block.x.size() / 3, mesh_block.history.size());
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

      auto history = mesh_block.history[i / 3];

      triangle.emplace_back(mesh_x, mesh_y, mesh_z);
      if (triangle.size() == 3) {
        auto interp_pts = interpolateTriangle(triangle);
        for (auto id : history.history) {
          // TODO(mikexyl): use id to store points per frame, to use less memory
          pointcloud_[id]->insert(pointcloud_[id]->end(), triangle.begin(),
                                  triangle.end());
          pointcloud_[id]->insert(pointcloud_[id]->end(), interp_pts.begin(),
                                  interp_pts.end());
        }

        triangle.clear();
      }
    }
    n++;
  }

  LOG(INFO) << "processed " << n;

  CHECK_EQ(T_G_C_.size(), pointcloud_.size());
  recovered_poincloud_timer.Stop();

  return true;
}

}  // namespace voxblox
