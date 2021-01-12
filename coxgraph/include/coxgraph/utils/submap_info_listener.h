#ifndef COXGRAPH_UTILS_SUBMAP_INFO_LISTENER_H_
#define COXGRAPH_UTILS_SUBMAP_INFO_LISTENER_H_

#include <cblox_msgs/MapPoseUpdates.h>
#include <minkindr_conversions/kindr_msg.h>

#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>

#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace utils {
class SubmapInfoListener {
 public:
  SubmapInfoListener()
      : SubmapInfoListener(ros::NodeHandle(), ros::NodeHandle("~")) {}
  explicit SubmapInfoListener(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : transformer_(nh, nh_private) {
    nh_private.param("use_tf_submap_pose", use_tf_submap_pose_, false);
    if (!use_tf_submap_pose_)
      submap_pose_sub_ = nh_private.subscribe(
          "submap_poses", 10, &SubmapInfoListener::submapPoseCallback, this);
    else
      LOG(FATAL) << "bug unfixed when using submap pose from tf";

    nh_private.param<float>("submap_pose_time_tolerance_ms",
                            submap_pose_time_tolerance_ms_, 500.0);

    nh_private.param<std::string>("map_frame_prefix", map_frame_prefix_,
                                  map_frame_prefix_);
  }

  ~SubmapInfoListener() = default;

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

  CIdCSIdPair getUnknownSubmapNearClient(
      const Transformation& T_G_C, const std::set<CIdCSIdPair>& known_submap_id,
      CIdCSIdPair* target_submap_id) {
    float min_max_overlap = 1.0;
    float min_dist = std::numeric_limits<float>().max();
    std::string csid_min_overlap;
    for (auto submap_bbox_kv : submap_aabb_map_) {
      if (known_submap_id.count(
              utils::resolveSubmapFrame(submap_bbox_kv.first)))
        continue;
      if (submap_bbox_kv.second.hasPosition(T_G_C.getPosition())) {
        float max_overlap_ratio = 0.0;
        for (auto csid_pair : known_submap_id) {
          CHECK(submap_aabb_map_.count(utils::getSubmapFrame(csid_pair)));
          BoundingBox other_bbox =
              submap_aabb_map_[utils::getSubmapFrame(csid_pair)];
          float overlap_ratio = submap_bbox_kv.second.overlapRatioWith(
                                    other_bbox) > max_overlap_ratio;
          if (overlap_ratio > kMaxOverlapRatio) continue;
          if (overlap_ratio > max_overlap_ratio) {
            max_overlap_ratio = overlap_ratio;
          }
        }
        if (max_overlap_ratio < min_max_overlap) {
          min_max_overlap = max_overlap_ratio;
          csid_min_overlap = submap_bbox_kv.first;
        } else if (max_overlap_ratio == min_max_overlap &&
                   submap_bbox_kv.second.distToPosition(T_G_C.getPosition()) <
                       min_dist) {
          min_dist = submap_bbox_kv.second.distToPosition(T_G_C.getPosition());
          csid_min_overlap = submap_bbox_kv.first;
        }
      }
    }
    return utils::resolveSubmapFrame(csid_min_overlap);
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

  ros::Subscriber submap_abb_sub_;
  std::map<std::string, BoundingBox> submap_aabb_map_;
  void submapBBoxCallback(const coxgraph_msgs::BoundingBox& bbox_msg) {
    std::string submap_name =
        utils::getSubmapFrame(std::make_pair(bbox_msg.cid, bbox_msg.csid));
    auto submap_abb_kv = submap_aabb_map_.find(submap_name);
    if (submap_abb_kv != submap_aabb_map_.end())
      submap_abb_kv->second = utils::getBBoxFromMsg(bbox_msg);
    else
      submap_aabb_map_.emplace(submap_name, utils::getBBoxFromMsg(bbox_msg));
  }

  bool use_tf_submap_pose_;
  float submap_pose_time_tolerance_ms_;
  std::string map_frame_prefix_;

  constexpr static float kMaxOverlapRatio = 0.4;
};

}  // namespace utils
}  // namespace coxgraph

#endif  // COXGRAPH_UTILS_SUBMAP_INFO_LISTENER_H_
