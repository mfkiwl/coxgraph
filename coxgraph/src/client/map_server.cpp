#include "coxgraph/client/map_server.h"

#include <coxgraph_msgs/MeshWithTrajectory.h>

#include <memory>
#include <string>

namespace coxgraph {
namespace client {

MapServer::Config MapServer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<float>("publish_combined_maps_every_n_sec",
                          config.publish_combined_maps_every_n_sec,
                          config.publish_combined_maps_every_n_sec);
  nh_private.param<bool>("publish_traversable", config.publish_traversable,
                         config.publish_traversable);
  nh_private.param<float>("traversability_radius", config.traversability_radius,
                          config.traversability_radius);
  return config;
}

void MapServer::subscribeTopics() {}

void MapServer::advertiseTopics() {
  tsdf_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("combined_tsdf_out", 10, true);
  esdf_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("combined_esdf_out", 10, true);

  if (config_.publish_combined_maps_every_n_sec > 0.0) {
    map_pub_timer_ = nh_private_.createTimer(
        ros::Duration(config_.publish_combined_maps_every_n_sec),
        &MapServer::publishMapEvent, this);
  }

  if (config_.publish_traversable)
    traversable_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
        "traversable", 10, true);

  submap_mesh_pub_ = nh_private_.advertise<coxgraph_msgs::MeshWithTrajectory>(
      "submap_mesh", 10, true);
}

void MapServer::updatePastTsdf() {
  if (tsdf_pub_.getNumSubscribers() == 0 &&
      esdf_pub_.getNumSubscribers() == 0 &&
      traversable_pub_.getNumSubscribers() == 0)
    return;

  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  for (auto const& submap_ptr : submap_collection_ptr_->getSubmapPtrs()) {
    voxblox::mergeLayerAintoLayerB(submap_ptr->getTsdfMapPtr()->getTsdfLayer(),
                                   submap_ptr->getPose(),
                                   tsdf_map_->getTsdfLayerPtr());
  }

  if (config_.publish_on_update) publishMap();
}

void MapServer::publishMapEvent(const ros::TimerEvent& event) { publishMap(); }

void MapServer::publishMap() {
  publishTsdf();
  publishEsdf();
  publishTraversable();
}

void MapServer::publishTsdf() {
  if (tsdf_pub_.getNumSubscribers() > 0) {
    std::lock_guard<std::mutex> tsdf_layer_update_lock(
        tsdf_layer_update_mutex_);
    voxblox_msgs::Layer layer_msg;
    voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(tsdf_map_->getTsdfLayer(),
                                                     false, &layer_msg);
    layer_msg.action = voxblox_msgs::Layer::ACTION_RESET;
    tsdf_pub_.publish(layer_msg);
  }
}

void MapServer::publishEsdf() {
  if (esdf_pub_.getNumSubscribers() > 0) {
    std::lock_guard<std::mutex> esdf_layer_update_lock(
        esdf_layer_update_mutex_);
    updateEsdfBatch();
    voxblox_msgs::Layer layer_msg;
    voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(esdf_map_->getEsdfLayer(),
                                                     false, &layer_msg);

    layer_msg.action = voxblox_msgs::Layer::ACTION_RESET;
    esdf_pub_.publish(layer_msg);
  }
}

void MapServer::publishTraversable() {
  if (traversable_pub_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    voxblox::createFreePointcloudFromEsdfLayer(
        esdf_map_->getEsdfLayer(), config_.traversability_radius, &pointcloud);
    pointcloud.header.frame_id = frame_names_.input_odom_frame;
    traversable_pub_.publish(pointcloud);
  }
}

void MapServer::publishSubmapMesh(CliSmId csid, std::string world_frame,
                                  const voxgraph::SubmapVisuals& submap_vis) {
  CliSm::ConstPtr submap_ptr = submap_collection_ptr_->getSubmapConstPtr(csid);
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection_ptr_->block_size());

  submap_vis.generateSubmapMesh(submap_ptr, voxblox::Color(),
                                mesh_layer_ptr.get());

  coxgraph_msgs::MeshWithTrajectory mesh_with_traj_msg;
  voxblox_msgs::MultiMesh mesh_msg;
  submap_vis.generateSubmapMeshMsg(mesh_layer_ptr,
                                   &mesh_with_traj_msg.mesh.mesh);
  mesh_with_traj_msg.mesh.header.frame_id =
      "submap_" + std::to_string(csid) + "_" + std::to_string(client_id_);
  mesh_with_traj_msg.mesh.name_space =
      "submap_" + std::to_string(csid) + "_" + std::to_string(client_id_);

  for (auto const& pose_kv : submap_ptr->getPoseHistory()) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame;
    pose_msg.header.stamp = pose_kv.first;
    tf::poseKindrToMsg(pose_kv.second.cast<double>(), &pose_msg.pose);
    mesh_with_traj_msg.trajectory.poses.emplace_back(pose_msg);
  }

  submap_mesh_pub_.publish(mesh_with_traj_msg);
}

}  // namespace client
}  // namespace coxgraph
