#ifndef COXGRAPH_UTILS_SUBMAP_POSE_LISTENER_H_
#define COXGRAPH_UTILS_SUBMAP_POSE_LISTENER_H_

#include <cblox_msgs/MapPoseUpdates.h>
#include <minkindr_conversions/kindr_msg.h>

#include <map>

#include "coxgraph/common.h"

namespace coxgraph {
namespace utils {
class SubmapPoseListener {
 public:
  SubmapPoseListener() : SubmapPoseListener(ros::NodeHandle()) {}
  explicit SubmapPoseListener(ros::NodeHandle nh_private) {
    nh_private.param("use_tf_submap_pose", use_tf_submap_pose_, false);
    if (!use_tf_submap_pose_)
      submap_pose_sub_ = nh_private.subscribe(
          "submap_poses", 10, &SubmapPoseListener::submapPoseCallback, this);
    else
      LOG(FATAL) << "bug unfixed when using submap pose from tf";

    nh_private.param("submap_pose_time_tolerance_ms",
                     submap_pose_time_tolerance_ms_,
                     submap_pose_time_tolerance_ms_);
  }

  ~SubmapPoseListener() = default;

  bool getSubmapPoseBlocking(int16_t submap_id, Transformation* T_G_Sm) {
    float loop_ms = 10;
    ros::Rate loop(ros::Duration(loop_ms / 1000));
    float wait_time_ms = 0;
    while (!submap_pose_map_.count(submap_id)) {
      loop.sleep();
      wait_time_ms += loop_ms;
      if (wait_time_ms > submap_pose_time_tolerance_ms_) {
        return false;
      }
    }
    *T_G_Sm = submap_pose_map_.find(submap_id)->second;
    return true;
  }

 private:
  tf::TransformListener tf_listener_;

  ros::Subscriber submap_pose_sub_;
  std::map<int16_t, Transformation> submap_pose_map_;
  void submapPoseCallback(const cblox_msgs::MapPoseUpdates& pose_update_msg) {
    for (auto const& map_header : pose_update_msg.map_headers) {
      kindr::minimal::QuatTransformationTemplate<double> T_G_Sm;
      tf::poseMsgToKindr(map_header.pose_estimate.map_pose, &T_G_Sm);
      auto submap_pose_kv = submap_pose_map_.find(map_header.id);
      if (submap_pose_kv == submap_pose_map_.end()) {
        submap_pose_map_.emplace(map_header.id, T_G_Sm.cast<FloatingPoint>());
      } else {
        submap_pose_kv->second = T_G_Sm.cast<FloatingPoint>();
      }
    }
  }

  bool use_tf_submap_pose_;
  float submap_pose_time_tolerance_ms_;
};
}  // namespace utils
}  // namespace coxgraph

#endif  // COXGRAPH_UTILS_SUBMAP_POSE_LISTENER_H_
