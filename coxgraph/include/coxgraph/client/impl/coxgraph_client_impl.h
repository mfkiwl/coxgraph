#ifndef COXGRAPH_CLIENT_IMPL_COXGRAPH_CLIENT_IMPL_H_
#define COXGRAPH_CLIENT_IMPL_COXGRAPH_CLIENT_IMPL_H_

#include <coxgraph_msgs/TimeLine.h>

#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {

void CoxgraphClient::subscribeClientTopics() {
  submap_subscriber_.shutdown();
  submap_subscriber_ = nh_.subscribe(submap_topic_, submap_topic_queue_length_,
                                     &CoxgraphClient::submapCallback, this);
}

void CoxgraphClient::advertiseClientTopics() {
  time_line_pub_ =
      nh_private_.advertise<coxgraph_msgs::TimeLine>("timeline", 1, true);
}

void CoxgraphClient::advertiseClientServices() {
  publish_client_submap_srv_ = nh_private_.advertiseService(
      "publish_client_submap", &CoxgraphClient::publishClientSubmapCallback,
      this);
}

bool CoxgraphClient::publishClientSubmapCallback(
    coxgraph_msgs::ClientSubmap::Request& request,
    coxgraph_msgs::ClientSubmap::Response& response) {
  ClientSubmapId submap_id;
  if (submap_collection_ptr_->lookupActiveSubmapByTime(request.timestamp,
                                                       &submap_id)) {
    const ClientSubmap& submap = submap_collection_ptr_->getSubmap(submap_id);
    Transformation T_submap_t;
    if (submap.lookupPoseByTime(request.timestamp, &T_submap_t)) {
      response.submap_id = submap_id;
      tf::transformKindrToMsg(T_submap_t.cast<double>(), &response.transform);
      response.sdf_layers = utils::tsdfMsgfromClientSubmap(
          submap, frame_names_.output_mission_frame);
      return true;
    } else {
      LOG(WARNING) << "Client " << client_id_ << ": Requested time "
                   << request.timestamp << " has no corresponding robot pose!";
      return false;
    }
  } else {
    LOG(WARNING) << "Client " << client_id_
                 << ": No active submap containing requested time "
                 << request.timestamp << "!";
    return false;
  }
  return false;
}

void CoxgraphClient::submapCallback(
    const voxblox_msgs::LayerWithTrajectory& submap_msg) {
  VoxgraphMapper::submapCallback(submap_msg);
  publishTimeLine();
}

void CoxgraphClient::publishTimeLine() {
  coxgraph_msgs::TimeLine time_line_msg;
  time_line_msg.start =
      submap_collection_ptr_
          ->getSubmapConstPtr(submap_collection_ptr_->getFirstSubmapId())
          ->getStartTime();
  time_line_msg.end =
      submap_collection_ptr_
          ->getSubmapConstPtr(submap_collection_ptr_->getLastSubmapId())
          ->getEndTime();
  time_line_pub_.publish(time_line_msg);
}

}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_IMPL_COXGRAPH_CLIENT_IMPL_H_
