#include "coxgraph_mod/mesh_converter.h"

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
  for (auto const& mesh_block : mesh_.mesh_blocks) {
    const BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                           mesh_block.index[2]);

    // If use mesh pointcloud interpolation, mesh will be connected, and
    // interpolate according to connected triangles
    size_t vertex_index = 0u;
    Mesh mesh;
    if (config_.interpolate_ratio > 0) {
      mesh.vertices.reserve(mesh_block.x.size());
      mesh.indices.reserve(mesh_block.x.size());
    }

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

      if (config_.interpolate_ratio > 0) {
        mesh.indices.push_back(vertex_index++);
        mesh.vertices.emplace_back(mesh_x, mesh_y, mesh_z);
      } else {
        pointcloud_->emplace_back(mesh_x, mesh_y, mesh_z);
      }
    }

    if (config_.interpolate_ratio > 0) {
      // connect mesh
      Mesh connected_mesh;
      createConnectedMesh(mesh, &connected_mesh);

      LOG(FATAL) << "Mesh pointcloud interpolation not implmented yet.";
    }
  }

  LOG(INFO) << "recovered point cloud size " << pointcloud_->size();

  recovered_poincloud_timer.Stop();

  return true;
}

}  // namespace voxblox
