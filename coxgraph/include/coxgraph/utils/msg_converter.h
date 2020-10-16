#ifndef COXGRAPH_UTILS_MSG_CONVERTER_H_
#define COXGRAPH_UTILS_MSG_CONVERTER_H_

#include <cblox_msgs/MapLayer.h>
#include <cblox_ros/submap_conversions.h>
#include <coxgraph_msgs/ClientSubmapResponse.h>
#include <coxgraph_msgs/MapFusion.h>

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

inline CliSm::Ptr cliSubmapFromMsg(
    const CliSmConfig& submap_config,
    const coxgraph_msgs::ClientSubmapResponse& submap_response) {
  CliSm::Ptr submap_ptr(
      new CliSm(Transformation(), submap_response.submap_id, submap_config));
  // Deserialize the submap TSDF
  if (!voxblox::deserializeMsgToLayer(
          submap_response.sdf_layers.tsdf_layer,
          submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr())) {
    LOG(FATAL)
        << "Received a submap msg with an invalid TSDF. Skipping submap.";
  }
  submap_ptr->finishSubmap();
  return submap_ptr;
}

voxgraph_msgs::LoopClosure fromMapFusionMsg(
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
