#ifndef COXGRAPH_UTILS_MSG_CONVERTER_H_
#define COXGRAPH_UTILS_MSG_CONVERTER_H_

#include <cblox_msgs/MapLayer.h>
#include <cblox_ros/submap_conversions.h>
#include <coxgraph_msgs/BoundingBox.h>
#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/ClientSubmapSrvResponse.h>
#include <coxgraph_msgs/MapFusion.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/LayerWithTrajectory.h>
#include <voxblox_ros/conversions.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <string>

#include "coxgraph/common.h"

namespace coxgraph {
namespace utils {

inline cblox_msgs::MapLayer tsdfEsdfMsgfromClientSubmap(
    const CliSm& submap, const std::string& frame_id) {
  cblox_msgs::MapLayer submap_esdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfEsdfSubmap>(submap, &submap_esdf_msg);
  submap_esdf_msg.map_header.pose_estimate.frame_id = frame_id;
  return submap_esdf_msg;
}

inline cblox_msgs::MapLayer tsdfMsgfromClientSubmap(
    const CliSm& submap, const std::string& frame_id) {
  cblox_msgs::MapLayer submap_tsdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfSubmap>(submap, &submap_tsdf_msg);
  submap_tsdf_msg.map_header.pose_estimate.frame_id = frame_id;
  return submap_tsdf_msg;
}

inline coxgraph_msgs::ClientSubmap msgFromCliSubmap(
    const CliSm& submap, const std::string& frame_id) {
  voxblox_msgs::Layer layer_msg;
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap.getTsdfMap().getTsdfLayer(), false, &layer_msg);
  voxblox_msgs::LayerWithTrajectory layer_with_trajectory_msg;
  layer_with_trajectory_msg.layer = layer_msg;

  LOG(INFO) << "debug: submap pose history size: "
            << submap.getPoseHistory().size();
  for (auto const& time_pose_kv : submap.getPoseHistory()) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = time_pose_kv.first;
    tf::poseKindrToMsg(time_pose_kv.second.cast<double>(), &pose_msg.pose);
    layer_with_trajectory_msg.trajectory.poses.emplace_back(pose_msg);
  }

  coxgraph_msgs::ClientSubmap cli_submap_msg;
  cli_submap_msg.layer_with_traj = layer_with_trajectory_msg;
  cli_submap_msg.map_header.id = submap.getID();
  cli_submap_msg.map_header.start = submap.getStartTime();
  cli_submap_msg.map_header.end = submap.getEndTime();
  cli_submap_msg.map_header.header.stamp = ros::Time::now();
  tf::poseKindrToMsg(submap.getPose().cast<double>(),
                     &cli_submap_msg.map_header.pose.map_pose);
  cli_submap_msg.map_header.pose.frame_id = frame_id;
  return cli_submap_msg;
}

/**
 * @brief Generate Client Submap from Message
 *
 * @param ser_sm_id
 * @param submap_config
 * @param submap_response
 * @param frame_id
 * @return CliSm::Ptr
 */
inline CliSm::Ptr cliSubmapFromMsg(
    const SerSmId& ser_sm_id, const CliSmConfig& submap_config,
    const coxgraph_msgs::ClientSubmap& submap_msg, std::string* frame_id) {
  // CHECK_EQ(submap_response.submap.layer_with_traj.trajectory.poses.size(),
  // 2);

  CliSm::Ptr submap_ptr(new CliSm(Transformation(), ser_sm_id, submap_config));

  // Naming copied from voxgraph
  for (const geometry_msgs::PoseStamped& pose_stamped :
       submap_msg.layer_with_traj.trajectory.poses) {
    TransformationD T_submap_base_link;
    tf::poseMsgToKindr(pose_stamped.pose, &T_submap_base_link);
    submap_ptr->addPoseToHistory(
        pose_stamped.header.stamp,
        T_submap_base_link.cast<voxblox::FloatingPoint>());
  }

  if (submap_ptr->getPoseHistory().size()) {
    TransformationD submap_pose;
    tf::poseMsgToKindr(submap_msg.map_header.pose.map_pose, &submap_pose);
    submap_ptr->setPose(submap_pose.cast<voxblox::FloatingPoint>());
    *frame_id = submap_msg.map_header.pose.frame_id;

    // Deserialize the submap TSDF
    if (!voxblox::deserializeMsgToLayer(
            submap_msg.layer_with_traj.layer,
            submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr())) {
      LOG(FATAL)
          << "Received a submap msg with an invalid TSDF. Skipping submap.";
    }
    submap_ptr->finishSubmap();
  }

  return submap_ptr;
}

inline CliSm::Ptr cliSubmapFromMsg(
    const SerSmId& ser_sm_id, const CliSmConfig& submap_config,
    const coxgraph_msgs::ClientSubmapSrvResponse& submap_response,
    std::string* frame_id) {
  return cliSubmapFromMsg(ser_sm_id, submap_config, submap_response.submap,
                          frame_id);
}

inline voxgraph_msgs::LoopClosure fromMapFusionMsg(
    const coxgraph_msgs::MapFusion& map_fusion_msg) {
  CHECK_EQ(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);
  voxgraph_msgs::LoopClosure loop_closure_msg;
  loop_closure_msg.from_timestamp = map_fusion_msg.from_timestamp;
  loop_closure_msg.to_timestamp = map_fusion_msg.to_timestamp;
  loop_closure_msg.transform = map_fusion_msg.transform;
  return loop_closure_msg;
}

inline coxgraph_msgs::BoundingBox msgFromBb(const BoundingBox& bounding_box) {
  coxgraph_msgs::BoundingBox bb_msg;
  bb_msg.min[0] = bounding_box.min[0];
  bb_msg.min[1] = bounding_box.min[1];
  bb_msg.min[2] = bounding_box.min[2];
  bb_msg.max[0] = bounding_box.max[0];
  bb_msg.max[1] = bounding_box.max[1];
  bb_msg.max[2] = bounding_box.max[2];
  return bb_msg;
}

}  // namespace utils
}  // namespace coxgraph

#endif  // COXGRAPH_UTILS_MSG_CONVERTER_H_
