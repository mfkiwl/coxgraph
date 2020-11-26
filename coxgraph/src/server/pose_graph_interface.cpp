#include "coxgraph/server/pose_graph_interface.h"

#include <vector>

#include <voxgraph/backend/constraint/relative_pose_constraint.h>

namespace coxgraph {
namespace server {

void PoseGraphInterface::optimize(bool enable_registration) {
  pose_graph_.optimize(true);

  // update registration constrains after loop closure optimized. Submaps
  // overlapping can only be determined after their relative poses computed by
  // loop closure optimization
  if (enable_registration) updateRegistrationConstraints();

  // Optimize the pose graph with all constraints enabled
  pose_graph_.optimize();

  // Publish debug visuals
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    pose_graph_vis_.publishPoseGraph(pose_graph_, visualization_odom_frame_,
                                     "optimized", pose_graph_pub_);
  }
}

void PoseGraphInterface::updateSubmapRPConstraints() {
  resetSubmapRelativePoseConstrains();
  for (int cid = 0; cid < cox_submap_collection_ptr_->getClientNumber();
       cid++) {
    std::vector<SerSmId>* cli_ser_sm_ids =
        cox_submap_collection_ptr_->getSerSmIdsByCliId(cid);
    for (int i = 0; i < cli_ser_sm_ids->size() - 1; i++) {
      int j = i + 1;
      SerSmId sid_i = cli_ser_sm_ids->at(i);
      SerSmId sid_j = cli_ser_sm_ids->at(j);

      Transformation T_M_SMi =
          cox_submap_collection_ptr_->getSubmapPtr(sid_i)->getPose();
      Transformation T_M_SMj =
          cox_submap_collection_ptr_->getSubmapPtr(sid_j)->getPose();
      Transformation T_SMi_SMj = T_M_SMi.inverse() * T_M_SMj;
      addSubmapRelativePoseConstraint(sid_i, sid_j, T_SMi_SMj);
    }
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
  if (verbose_) {
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
