#include "coxgraph/client/map_server.h"

#include <coxgraph_msgs/BoundingBox.h>
#include <coxgraph_msgs/MeshWithTrajectory.h>
#include <voxblox_msgs/MultiMesh.h>

#include <memory>
#include <string>

namespace coxgraph {
namespace client {

MapServer::Config MapServer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<bool>("publish_mesh_with_trajectory",
                         config.publish_mesh_with_trajectory,
                         config.publish_mesh_with_trajectory);
  return config;
}

void MapServer::subscribeToTopics() {
  kf_pose_sub_ = nh_private_.subscribe("keyframe_pose", 10,
                                       &MapServer::kfPoseCallback, this);
}

void MapServer::advertiseTopics() {
  if (config_.publish_mesh_with_trajectory)
    submap_mesh_pub_ = nh_private_.advertise<coxgraph_msgs::MeshWithTrajectory>(
        "submap_mesh_with_traj", 10, true);
  else
    submap_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::MultiMesh>("submap_mesh", 10, true);

  submap_mesh_pub_ = nh_private_.advertise<coxgraph_msgs::BoundingBox>(
      "submap_bbox", 10, true);
}

void MapServer::subscribeToServices() {
  get_submap_mesh_with_traj_cli_ =
      nh_private_.serviceClient<coxgraph_msgs::GetSubmapMeshWithTraj>(
          "get_submap_mesh_with_traj");
}

void MapServer::advertiseServices() {
  set_target_srv_ = nh_private_.advertiseService(
      "set_target", &MapServer::setTargetPositionCallback, this);
}

void MapServer::startTimers() {}

void MapServer::publishSubmapMesh(CliSmId csid, std::string /* world_frame */,
                                  const voxgraph::SubmapVisuals& submap_vis) {
  CliSm::ConstPtr submap_ptr = submap_collection_ptr_->getSubmapConstPtr(csid);
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection_ptr_->block_size());

  submap_vis.generateSubmapMesh(submap_ptr, voxblox::Color(),
                                mesh_layer_ptr.get());

  voxblox_msgs::MultiMesh mesh_msg;
  submap_vis.generateSubmapMeshMsg(mesh_layer_ptr, &mesh_msg.mesh);
  std::string submap_frame =
      "submap_" + std::to_string(csid) + "_" + std::to_string(client_id_);
  mesh_msg.header.frame_id = submap_frame;
  mesh_msg.name_space = submap_frame;

  if (config_.publish_mesh_with_trajectory) {
    coxgraph_msgs::MeshWithTrajectory mesh_with_traj_msg;
    mesh_with_traj_msg.mesh = mesh_msg;
    for (auto const& pose_kv : submap_ptr->getPoseHistory()) {
      if (!kf_timestamp_set_.count(pose_kv.first)) continue;
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = submap_frame;
      pose_msg.header.stamp = pose_kv.first;
      tf::poseKindrToMsg(pose_kv.second.cast<double>(), &pose_msg.pose);
      mesh_with_traj_msg.trajectory.poses.emplace_back(pose_msg);
    }

    submap_mesh_pub_.publish(mesh_with_traj_msg);
  } else {
    submap_mesh_pub_.publish(mesh_msg);
  }
}

void MapServer::requestNewSubmap() {}

bool MapServer::setTargetPositionCallback(
    coxgraph_msgs::SetTargetPoseRequest& request,      // NOLINT
    coxgraph_msgs::SetTargetPoseResponse& response) {  // NOLINT
  Transformation T_G_Cli, T_G_Tgt;
  TransformationD T_Cli_Tgt;
  if (!submap_info_listener_.lookupTfGlobalToCli(client_id_, &T_G_Cli)) {
    response.result = coxgraph_msgs::SetTargetPoseResponse::NOT_MERGED;
    return true;
  }
  tf::poseMsgToKindr(request.target_pose, &T_Cli_Tgt);
  T_G_Tgt = T_G_Cli * T_Cli_Tgt.cast<FloatingPoint>();
  CIdCSIdPair target_submap_id;
  if (submap_info_listener_.getUnknownSubmapNearClient(
          T_G_Tgt, submap_collection_ptr_->getSubmapCsidPairs(client_id_),
          &target_submap_id)) {
    coxgraph_msgs::GetSubmapMeshWithTraj get_submap_mesh_srv;
    get_submap_mesh_srv.request.cid = target_submap_id.first;
    get_submap_mesh_srv.request.csid = target_submap_id.second;
    if (get_submap_mesh_with_traj_cli_.call(get_submap_mesh_srv)) {
      submap_collection_ptr_->addSubmapFromMeshAsync(
          get_submap_mesh_srv.response.mesh_with_traj,
          get_submap_mesh_srv.request.cid, get_submap_mesh_srv.request.csid);
      response.result = coxgraph_msgs::SetTargetPoseResponse::SUCCESS;
    } else {
      response.result = coxgraph_msgs::SetTargetPoseResponse::NO_AVAILABLE;
    }
  } else {
    response.result = coxgraph_msgs::SetTargetPoseResponse::NO_AVAILABLE;
  }

  return true;
}

}  // namespace client
}  // namespace coxgraph
