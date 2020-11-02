#ifndef COXGRAPH_SERVER_POSE_GRAPH_INTERFACE_H_
#define COXGRAPH_SERVER_POSE_GRAPH_INTERFACE_H_

#include <voxgraph/backend/constraint/relative_pose_constraint.h>
#include <voxgraph/frontend/pose_graph_interface/pose_graph_interface.h>

#include <string>

#include "coxgraph/common.h"
#include "coxgraph/utils/ros_params.h"

namespace coxgraph {
namespace server {

class PoseGraphInterface : public voxgraph::PoseGraphInterface {
 public:
  using RelativePoseConstraint = voxgraph::RelativePoseConstraint;
  using RegistrationConstraint = voxgraph::RegistrationConstraint;

  PoseGraphInterface(
      const ros::NodeHandle& nh,
      const voxgraph::VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
      const voxblox::MeshIntegratorConfig& mesh_config,
      const std::string& visualizations_mission_frame, bool verbose = false)
      : voxgraph::PoseGraphInterface(nh, submap_collection_ptr, mesh_config,
                                     visualizations_mission_frame, verbose) {
    utils::setInformationMatrixFromRosParams(
        ros::NodeHandle(nh, "submap_relative_pose/information_matrix"),
        &sm_rp_info_matrix_);
  }
  ~PoseGraphInterface() = default;

  void optimize(bool enable_registration);

  void resetSubmapRelativePoseConstrains() {
    pose_graph_.resetSubmapRelativePoseConstraints();
  }

  void addSubmapRelativePoseConstraint(const SerSmId& first_submap_id,
                                       const SerSmId& second_submap_id,
                                       const Transformation& T_S1_S2);

  void addForceRegistrationConstraint(const SerSmId& first_submap_id,
                                      const SerSmId& second_submap_id);

 private:
  InformationMatrix sm_rp_info_matrix_;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_POSE_GRAPH_INTERFACE_H_
