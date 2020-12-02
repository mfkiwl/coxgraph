#ifndef COXGRAPH_SERVER_VISUALIZER_SERVER_VISUALIZER_H_
#define COXGRAPH_SERVER_VISUALIZER_SERVER_VISUALIZER_H_

#include <cblox/mesh/submap_mesher.h>
#include <ros/ros.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxgraph/tools/visualization/submap_visuals.h>

#include <string>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/pose_graph_interface.h"
#include "coxgraph/server/submap_collection.h"

namespace coxgraph {
namespace server {
class ServerVisualizer {
 public:
  ServerVisualizer(const CliSmConfig& submap_config,
                   const MeshIntegratorConfig& mesh_config)
      : submap_config_(submap_config),
        mesh_config_(mesh_config),
        submap_vis_(submap_config, mesh_config) {}

  ~ServerVisualizer() = default;

  void setMeshOpacity(float mesh_opacity) {
    submap_vis_.setMeshOpacity(mesh_opacity);
  }
  void setSubmapMeshColorMode(voxblox::ColorMode color_mode) {
    submap_vis_.setSubmapMeshColorMode(color_mode);
  }
  void setCombinedMeshColorMode(voxblox::ColorMode color_mode) {
    submap_vis_.setCombinedMeshColorMode(color_mode);
  }

  /**
   * @brief Get the Final Global Mesh object, other submaps will be added to
   * submap_collection and pose graph to optimize, then removed, so
   * submap_collection and pose graph will not be changed.
   *
   * @param submap_collection_ptr
   * @param pose_graph_interface
   * @param other_submaps
   */
  void getFinalGlobalMesh(const SubmapCollection::Ptr& submap_collection_ptr,
                          const PoseGraphInterface& pose_graph_interface,
                          const std::vector<CliSmIdPack>& other_submaps,
                          const std::string& mission_frame,
                          const ros::Publisher& publisher,
                          const std::string& file_path);

 private:
  CliSmConfig submap_config_;
  MeshIntegratorConfig mesh_config_;

  voxgraph::SubmapVisuals submap_vis_;
};  // namespace server

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_VISUALIZER_SERVER_VISUALIZER_H_
