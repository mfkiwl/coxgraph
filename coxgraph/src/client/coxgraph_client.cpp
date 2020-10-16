#include "coxgraph/client/coxgraph_client.h"

#include <coxgraph_msgs/TimeLine.h>

#include "coxgraph/common.h"

namespace coxgraph {

void CoxgraphClient::subscribeClientTopics() {
  submap_subscriber_.shutdown();
  submap_subscriber_ = nh_.subscribe(submap_topic_, submap_topic_queue_length_,
                                     &CoxgraphClient::submapCallback, this);
}

void CoxgraphClient::advertiseClientTopics() {
  time_line_pub_ =
      nh_private_.advertise<coxgraph_msgs::TimeLine>("time_line", 1, true);
}

void CoxgraphClient::advertiseClientServices() {
  publish_client_submap_srv_ = nh_private_.advertiseService(
      "publish_client_submap", &CoxgraphClient::pubClientSubmapCallback, this);
}

bool CoxgraphClient::pubClientSubmapCallback(
    coxgraph_msgs::ClientSubmapSrv::Request& request,
    coxgraph_msgs::ClientSubmapSrv::Response& response) {
  CliSmId submap_id;
  if (submap_collection_ptr_->lookupActiveSubmapByTime(request.timestamp,
                                                       &submap_id)) {
    const CliSm& submap = submap_collection_ptr_->getSubmap(submap_id);
    Transformation T_submap_t;
    if (submap.lookupPoseByTime(request.timestamp, &T_submap_t)) {
      response.submap.map_header.id = submap_id;
      tf::transformKindrToMsg(T_submap_t.cast<double>(), &response.transform);
      response.submap =
          utils::msgFromClientSubmap(submap, frame_names_.output_mission_frame);
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
  if (!submap_collection_ptr_->empty()) publishTimeLine();
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
  LOG(INFO) << "Updating client time Line from " << time_line_msg.start
            << " to " << time_line_msg.end;
  time_line_pub_.publish(time_line_msg);
}

}  // namespace coxgraph
