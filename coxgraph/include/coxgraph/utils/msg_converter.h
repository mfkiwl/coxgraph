#ifndef COXGRAPH_UTILS_MSG_CONVERTER_H_
#define COXGRAPH_UTILS_MSG_CONVERTER_H_

#include <cblox_msgs/MapLayer.h>
#include <cblox_ros/submap_conversions.h>
#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/ClientSubmapSrvResponse.h>
#include <coxgraph_msgs/MapFusion.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/LayerWithTrajectory.h>
#include <voxblox_ros/conversions.h>

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

inline coxgraph_msgs::ClientSubmap msgFromClientSubmap(
    const CliSm& submap, const std::string& frame_id) {
  voxblox_msgs::Layer layer_msg;
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap.getTsdfMap().getTsdfLayer(), false, &layer_msg);
  voxblox_msgs::LayerWithTrajectory layer_with_trajectory_msg;
  layer_with_trajectory_msg.layer = layer_msg;

  // Give it two dummy poses with submap timeline stamped to sync timeline
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = submap.getStartTime();
  layer_with_trajectory_msg.trajectory.poses.emplace_back(pose_msg);
  pose_msg.header.stamp = submap.getEndTime();
  layer_with_trajectory_msg.trajectory.poses.emplace_back(pose_msg);

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
    const coxgraph_msgs::ClientSubmapSrvResponse& submap_response,
    std::string* frame_id) {
  CHECK_EQ(submap_response.submap.layer_with_traj.trajectory.poses.size(), 2);

  CliSm::Ptr submap_ptr(new CliSm(Transformation(), ser_sm_id, submap_config));
  // Deserialize the submap TSDF
  if (!voxblox::deserializeMsgToLayer(
          submap_response.submap.layer_with_traj.layer,
          submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr())) {
    LOG(FATAL)
        << "Received a submap msg with an invalid TSDF. Skipping submap.";
  }
  TransformationD submap_pose;
  tf::poseMsgToKindr(submap_response.submap.map_header.pose.map_pose,
                     &submap_pose);
  const Transformation T_M_O;
  submap_ptr->setPose(T_M_O);
  submap_ptr->transformSubmap(submap_pose.cast<voxblox::FloatingPoint>());
  *frame_id = submap_response.submap.map_header.pose.frame_id;
  return submap_ptr;
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

}  // namespace utils
}  // namespace coxgraph

#endif  // COXGRAPH_UTILS_MSG_CONVERTER_H_
