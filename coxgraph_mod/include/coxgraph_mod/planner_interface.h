#ifndef COXGRAPH_MOD_PLANNER_INTERFACE_H_
#define COXGRAPH_MOD_PLANNER_INTERFACE_H_

#include <geometry_msgs/Pose.h>

namespace coxgraph {
namespace mod {

/**
 * @brief Set the Target Pose object
 *
 * @param target_pose target pose in client odom frame
 * @return true
 * @return false
 */

bool setTargetPose(const geometry_msgs::Pose& target_pose, int* result);

}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_PLANNER_INTERFACE_H_
