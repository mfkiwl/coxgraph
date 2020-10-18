#include "coxgraph/server/pose_graph_interface.h"

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

}  // namespace server
}  // namespace coxgraph
