#ifndef COXGRAPH_UTILS_MSG_CONVERTER_H_
#define COXGRAPH_UTILS_MSG_CONVERTER_H_

#include <cblox_msgs/MapLayer.h>
#include <cblox_ros/submap_conversions.h>
#include <coxgraph_msgs/ClientSubmapResponse.h>

#include <string>

#include "coxgraph/common.h"

namespace coxgraph {
namespace utils {

inline cblox_msgs::MapLayer tsdfEsdfMsgfromClientSubmap(
    const ClientSubmap& submap, const std::string& frame_id) {
  cblox_msgs::MapLayer submap_esdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfEsdfSubmap>(submap, &submap_esdf_msg);
  submap_esdf_msg.map_header.pose_estimate.frame_id = frame_id;
  return submap_esdf_msg;
}

inline cblox_msgs::MapLayer tsdfMsgfromClientSubmap(
    const ClientSubmap& submap, const std::string& frame_id) {
  cblox_msgs::MapLayer submap_tsdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfSubmap>(submap, &submap_tsdf_msg);
  submap_tsdf_msg.map_header.pose_estimate.frame_id = frame_id;
  return submap_tsdf_msg;
}

inline ClientSubmap::Ptr cliSubmapFromMsg(
    const ClientSubmapConfig& submap_config,
    const coxgraph_msgs::ClientSubmapResponse& submap_response) {
  ClientSubmap::Ptr submap_ptr(new ClientSubmap(
      Transformation(), submap_response.submap_id, submap_config));
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

}  // namespace utils
}  // namespace coxgraph

#endif  // COXGRAPH_UTILS_MSG_CONVERTER_H_
