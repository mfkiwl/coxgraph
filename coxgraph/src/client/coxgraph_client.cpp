#include "coxgraph/client/coxgraph_client.h"

#include <coxgraph_msgs/MapPoseUpdates.h>
#include <coxgraph_msgs/TimeLine.h>

#include <chrono>

#include "coxgraph/common.h"

namespace coxgraph {

void CoxgraphClient::subscribeClientTopics() {
  submap_subscriber_.shutdown();
  submap_subscriber_ = nh_.subscribe(submap_topic_, submap_topic_queue_length_,
                                     &CoxgraphClient::submapCallback, this);
}

void CoxgraphClient::advertiseClientTopics() {
  time_line_pub_ = nh_private_.advertise<coxgraph_msgs::TimeLine>(
      "time_line", publisher_queue_length_, true);
  map_pose_pub_ = nh_private_.advertise<coxgraph_msgs::MapPoseUpdates>(
      "map_pose_updates", publisher_queue_length_, true);
}

void CoxgraphClient::advertiseClientServices() {
  get_client_submap_srv_ = nh_private_.advertiseService(
      "get_client_submap", &CoxgraphClient::getClientSubmapCallback, this);
  get_all_client_submaps_srv_ = nh_private_.advertiseService(
      "get_all_submaps", &CoxgraphClient::getAllClientSubmapsCallback, this);
}

// TODO(mikexyl): add locks here, if optimizing is running, wait
bool CoxgraphClient::getClientSubmapCallback(
    coxgraph_msgs::ClientSubmapSrv::Request& request,
    coxgraph_msgs::ClientSubmapSrv::Response& response) {
  // if (optimization_async_handle_.valid() &&
  //     optimization_async_handle_.wait_for(std::chrono::milliseconds(100)) !=
  //         std::future_status::ready) {
  //   ROS_WARN(
  //       "Previous pose graph optimization still not complete. Client submap "
  //       "request ignored");
  //   return false;
  // }

  CliSmId submap_id;
  if (submap_collection_ptr_->lookupActiveSubmapByTime(request.timestamp,
                                                       &submap_id)) {
    const CliSm& submap = submap_collection_ptr_->getSubmap(submap_id);
    Transformation T_submap_t;
    if (submap.lookupPoseByTime(request.timestamp, &T_submap_t)) {
      response.submap.map_header.id = submap_id;
      tf::transformKindrToMsg(T_submap_t.cast<double>(), &response.transform);
      if (!ser_sm_id_pose_map_.count(submap_id)) {
        response.submap =
            utils::msgFromCliSubmap(submap, frame_names_.output_mission_frame);
        ser_sm_id_pose_map_.emplace(submap_id, submap.getPose());
        LOG(INFO) << log_prefix_ << " Submap " << submap_id
                  << " is successfully sent to server";
      } else {
        LOG(INFO) << log_prefix_ << " Submap " << submap_id
                  << " has already been sent to server";
      }
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

bool CoxgraphClient::getAllClientSubmapsCallback(
    coxgraph_msgs::SubmapsSrv::Request& request,  /////////
    coxgraph_msgs::SubmapsSrv::Response& response) {
  LOG(INFO) << log_prefix_
            << "Server is requesting all submaps! pausing submap process";
  uint8_t trials_ = 0;
  while (!submap_proc_mutex_.try_lock_for(std::chrono::milliseconds(500))) {
    LOG(INFO) << log_prefix_
              << "current submap is still being processed, waiting";
    trials_++;
    CHECK_LT(trials_, 3) << " Tried 3 times, submap process is still running";
  }
  LOG(INFO) << log_prefix_ << "Submap process is paused, sending all submaps";

  for (auto const& submap_ptr : submap_collection_ptr_->getSubmapPtrs()) {
    if (ser_sm_id_pose_map_.count(submap_ptr->getID())) continue;
    response.submaps.emplace_back(utils::msgFromCliSubmap(
        *submap_ptr, frame_names_.output_mission_frame));
  }

  submap_proc_mutex_.unlock();

  return true;
}

void CoxgraphClient::submapCallback(
    const voxblox_msgs::LayerWithTrajectory& submap_msg) {
  std::lock_guard<std::timed_mutex> submap_proc_lock(submap_proc_mutex_);
  VoxgraphMapper::submapCallback(submap_msg);
  if (submap_collection_ptr_->size()) {
    publishTimeLine();
    publishMapPoseUpdates();
  }
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
  LOG(INFO) << log_prefix_ << "Updating client time Line from "
            << time_line_msg.start << " to " << time_line_msg.end;
  time_line_pub_.publish(time_line_msg);
}

void CoxgraphClient::publishMapPoseUpdates() {
  TransformationVector submap_poses;
  submap_collection_ptr_->getSubmapPoses(&submap_poses);

  coxgraph_msgs::MapPoseUpdates map_pose_updates_msg;
  for (auto& sm_id_pose_kv : ser_sm_id_pose_map_) {
    if (!(submap_poses[sm_id_pose_kv.first] == sm_id_pose_kv.second)) {
      LOG(INFO) << log_prefix_ << "Updating pose of submap "
                << sm_id_pose_kv.first << " to server";
      sm_id_pose_kv.second = submap_poses[sm_id_pose_kv.first];
      map_pose_updates_msg.submap_id.emplace_back(sm_id_pose_kv.first);
      geometry_msgs::Pose new_pose;
      tf::poseKindrToMsg(sm_id_pose_kv.second.cast<double>(), &new_pose);
      map_pose_updates_msg.new_pose.emplace_back(new_pose);
    }
  }
  if (map_pose_updates_msg.submap_id.size())
    map_pose_pub_.publish(map_pose_updates_msg);
}

}  // namespace coxgraph
