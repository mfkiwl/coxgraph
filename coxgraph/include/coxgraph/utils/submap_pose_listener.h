#ifndef COXGRAPH_UTILS_SUBMAP_POSE_LISTENER_H_
#define COXGRAPH_UTILS_SUBMAP_POSE_LISTENER_H_

#include <cblox_msgs/MapPoseUpdates.h>
#include <minkindr_conversions/kindr_msg.h>

#include <map>
#include <string>
#include <utility>

#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace utils {
class SubmapPoseListener {
 public:
  SubmapPoseListener()
      : SubmapPoseListener(ros::NodeHandle(), ros::NodeHandle("~")) {}
  explicit SubmapPoseListener(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : transformer_(nh, nh_private) {
    nh_private.param("use_tf_submap_pose", use_tf_submap_pose_, false);
    if (!use_tf_submap_pose_)
      submap_pose_sub_ = nh_private.subscribe(
          "submap_poses", 10, &SubmapPoseListener::submapPoseCallback, this);
    else
      LOG(FATAL) << "bug unfixed when using submap pose from tf";

    nh_private.param<float>("submap_pose_time_tolerance_ms",
                            submap_pose_time_tolerance_ms_, 500.0);

    nh_private.param<std::string>("map_frame_prefix", map_frame_prefix_,
                                  map_frame_prefix_);
  }

  ~SubmapPoseListener() = default;

  /**
   * @brief Get the Submap Pose in Blocking mode, if submap pose can't be find,
   * block
   *
   * @param submap_frame
   * @param T_G_Sm if from_frame is empty, this will be transform from client
   * map frame to submap frame
   * @param from_frame
   * @return true
   * @return false
   */
  bool getSubmapPoseBlocking(const std::string& submap_frame,
                             Transformation* T_G_Sm,
                             bool from_global_frame = false) {
    float loop_ms = 10;
    ros::Rate loop(ros::Duration(loop_ms / 1000));
    float wait_time_ms = 0;
    while (!submap_pose_map_.count(submap_frame)) {
      loop.sleep();
      wait_time_ms += loop_ms;
      if (wait_time_ms > submap_pose_time_tolerance_ms_) {
        LOG(ERROR) << submap_pose_time_tolerance_ms_
                   << "ms later, still hasn't receive pose for "
                   << submap_frame;
        return false;
      }
    }
    if (!from_global_frame) {
      *T_G_Sm = submap_pose_map_.find(submap_frame)->second;
    } else {
      CIdCSIdPair csid_pair = utils::resolveSubmapFrame(submap_frame);
      Transformation T_G_Cli;

      if (!transformer_.lookupTransform(
              map_frame_prefix_ + "_g",
              map_frame_prefix_ + "_" + std::to_string(csid_pair.first),
              ros::Time(0), &T_G_Cli)) {
        LOG(INFO) << "Failed to look up tf from " << map_frame_prefix_ + "_g"
                  << " to "
                  << map_frame_prefix_ + "_" + std::to_string(csid_pair.first);
        return false;
      } else {
        *T_G_Sm = T_G_Cli * submap_pose_map_.find(submap_frame)->second;
      }
    }
    return true;
  }

 private:
  voxblox::Transformer transformer_;

  ros::Subscriber submap_pose_sub_;
  std::map<std::string, Transformation> submap_pose_map_;
  void submapPoseCallback(const cblox_msgs::MapPoseUpdates& pose_update_msg) {
    CliId cid = utils::resolveMapFrame(pose_update_msg.header.frame_id);
    for (auto const& map_header : pose_update_msg.map_headers) {
      kindr::minimal::QuatTransformationTemplate<double> T_G_Sm;
      tf::poseMsgToKindr(map_header.pose_estimate.map_pose, &T_G_Sm);
      std::string submap_frame =
          utils::getSubmapFrame(std::make_pair(cid, map_header.id));
      auto submap_pose_kv = submap_pose_map_.find(submap_frame);
      if (submap_pose_kv == submap_pose_map_.end()) {
        LOG(INFO) << "Received submap pose for " << submap_frame;
        submap_pose_map_.emplace(submap_frame, T_G_Sm.cast<FloatingPoint>());
      } else {
        submap_pose_kv->second = T_G_Sm.cast<FloatingPoint>();
      }
    }
  }

  bool use_tf_submap_pose_;
  float submap_pose_time_tolerance_ms_;
  std::string map_frame_prefix_;
};
}  // namespace utils
}  // namespace coxgraph

#endif  // COXGRAPH_UTILS_SUBMAP_POSE_LISTENER_H_
