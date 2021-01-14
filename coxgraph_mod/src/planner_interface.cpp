#include "coxgraph_mod/planner_interface.h"

#include <coxgraph_msgs/SetTargetPose.h>
#include <ros/ros.h>

namespace coxgraph {
namespace mod {
class PlannerInterface {
 public:
  PlannerInterface() {
    set_target_pose_cli_ =
        nh_.serviceClient<coxgraph_msgs::SetTargetPose>("set_target_pose");
  }
  ~PlannerInterface() = default;

  enum SetTargetResult { SUCCESS = 0, NOT_MERGED = 1, NO_AVAILABLE = 2 };
  bool setTargetPose(const geometry_msgs::Pose& target_pose, int* result) {
    coxgraph_msgs::SetTargetPose set_target_pose_srv;
    set_target_pose_srv.request.target_pose = target_pose;
    if (set_target_pose_cli_.call(set_target_pose_srv)) {
      *result = set_target_pose_srv.response.result;
      return true;
    } else {
      return false;
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceClient set_target_pose_cli_;
};

static PlannerInterface planner_interface;

bool setTargetPose(const geometry_msgs::Pose& target_pose, int* result) {
  return planner_interface.setTargetPose(target_pose, result);
}

}  // namespace mod
}  // namespace coxgraph
