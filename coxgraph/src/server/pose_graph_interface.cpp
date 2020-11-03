#include "coxgraph/server/pose_graph_interface.h"

#include <voxgraph/backend/constraint/relative_pose_constraint.h>

namespace coxgraph {
namespace server {

void PoseGraphInterface::optimize(bool enable_registration) {
  if (new_loop_closures_added_since_last_optimization_) {
    // Optimize the graph excluding the registration constraints
    pose_graph_.optimize(true);

    // Indicate that the new loop closures have been taken care off
    new_loop_closures_added_since_last_optimization_ = false;
  }

  // update registration constrains after loop closure optimized. Submaps
  // overlapping can only be determined after their relative poses computed by
  // loop closure optimization
  if (enable_registration) updateRegistrationConstraints();

  // Optimize the pose graph with all constraints enabled
  pose_graph_.optimize();

  // Publish debug visuals
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    pose_graph_vis_.publishPoseGraph(pose_graph_, visualization_mission_frame_,
                                     "optimized", pose_graph_pub_);
  }
}

void PoseGraphInterface::addSubmapRelativePoseConstraint(
    const SerSmId& first_submap_id, const SerSmId& second_submap_id,
    const Transformation& T_S1_S2) {
  RelativePoseConstraint::Config submap_rp_config;
  submap_rp_config.information_matrix = sm_rp_info_matrix_;
  submap_rp_config.origin_submap_id = first_submap_id;
  submap_rp_config.destination_submap_id = second_submap_id;
  submap_rp_config.T_origin_destination = T_S1_S2;

  // Add the constraint to the pose graph
  // TODO(mikexyl): since these should be called every time submap pose updated,
  // don't log it
  if (false) {
    std::cout << "Adding submap relative pose constraint\n"
              << "From: " << submap_rp_config.origin_submap_id << "\n"
              << "To: " << submap_rp_config.destination_submap_id << "\n"
              << "Submap currently being built in submap collection: "
              << submap_collection_ptr_->getActiveSubmapID() << "\n"
              << "t_s1_s2:\n"
              << submap_rp_config.T_origin_destination.getPosition() << "\n"
              << "yaw_s1_s2: " << submap_rp_config.T_origin_destination.log()[5]
              << "\n"
              << "Information matrix\n"
              << submap_rp_config.information_matrix << std::endl;
  }
  pose_graph_.addSubmapRelativePoseConstraint(submap_rp_config);
}

void PoseGraphInterface::addForceRegistrationConstraint(
    const SerSmId& first_submap_id, const SerSmId& second_submap_id) {
  RegistrationConstraint::Config constraint_config =
      measurement_templates_.registration;
  constraint_config.first_submap_id = first_submap_id;
  constraint_config.second_submap_id = second_submap_id;

  // Add pointers to both submaps
  constraint_config.first_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(first_submap_id);
  constraint_config.second_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(second_submap_id);
  CHECK_NOTNULL(constraint_config.first_submap_ptr);
  CHECK_NOTNULL(constraint_config.second_submap_ptr);

  // Add the constraint to the pose graph
  pose_graph_.addForceRegistrationConstraint(constraint_config);
}

}  // namespace server
}  // namespace coxgraph
