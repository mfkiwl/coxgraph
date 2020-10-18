#ifndef COXGRAPH_SERVER_POSE_GRAPH_INTERFACE_H_
#define COXGRAPH_SERVER_POSE_GRAPH_INTERFACE_H_

#include <voxgraph/frontend/pose_graph_interface/pose_graph_interface.h>

#include <string>

namespace coxgraph {
namespace server {

class PoseGraphInterface : public voxgraph::PoseGraphInterface {
 public:
  PoseGraphInterface(
      const ros::NodeHandle& node_handle,
      const voxgraph::VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
      const voxblox::MeshIntegratorConfig& mesh_config,
      const std::string& visualizations_mission_frame, bool verbose = false)
      : voxgraph::PoseGraphInterface(node_handle, submap_collection_ptr,
                                     mesh_config, visualizations_mission_frame,
                                     verbose) {}
  ~PoseGraphInterface() = default;

  void optimize(bool enable_registration);

 private:
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_POSE_GRAPH_INTERFACE_H_
