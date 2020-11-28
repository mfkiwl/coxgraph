#include "coxgraph/client/map_server.h"

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

void MapServer::subscribeTopics() {
  if (config_.publish_combined_maps_every_n_sec == 0.0) return;
  active_tsdf_sub_ = nh_private_.subscribe(
      "active_tsdf_in", 10, &MapServer::activeTsdfCallback, this);
}

void MapServer::advertiseTopics() {
  if (config_.publish_combined_maps_every_n_sec == 0.0) return;
  tsdf_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("combined_tsdf_out", 10, true);
  esdf_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("combined_esdf_out", 10, true);
  map_pub_timer_ = nh_private_.createTimer(
      ros::Duration(config_.publish_combined_maps_every_n_sec),
      &MapServer::publishMap, this);
  if (config_.publish_traversable)
    traversable_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
        "traversable", 1, true);
}

void MapServer::activeTsdfCallback(const voxblox_msgs::Layer& layer_msg) {
  CHECK_EQ(layer_msg.layer_type, voxblox::voxel_types::kTsdf);
  CHECK_EQ(layer_msg.voxel_size,
           tsdf_map_->getTsdfLayerConstPtr()->voxel_size());
  CHECK_EQ(layer_msg.voxels_per_side,
           tsdf_map_->getTsdfLayerConstPtr()->voxels_per_side());
  if (!voxblox::deserializeMsgToLayer(layer_msg, active_tsdf_layer_)) {
    LOG(ERROR) << "Received an invalid TSDF.";
  }
}

void MapServer::updatePastTsdf() {
  if (config_.publish_combined_maps_every_n_sec == 0.0) return;
  past_tsdf_layer_->removeAllBlocks();
  for (auto const& submap_ptr : submap_collection_ptr_->getSubmapPtrs()) {
    voxblox::mergeLayerAintoLayerB(submap_ptr->getTsdfMapPtr()->getTsdfLayer(),
                                   submap_ptr->getPose(), past_tsdf_layer_);
  }
}

void MapServer::publishMap(const ros::TimerEvent& event) {
  mergeTsdfs();
  publishTsdf();
  publishEsdf();
  publishTraversable();
}

void MapServer::mergeTsdfs() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  LOG(INFO) << "debug: before merge active tsdf "
            << active_tsdf_layer_->getNumberOfAllocatedBlocks();
  voxblox::mergeLayerAintoLayerB(*active_tsdf_layer_,
                                 tsdf_map_->getTsdfLayerPtr());
  voxblox::mergeLayerAintoLayerB(*past_tsdf_layer_,
                                 tsdf_map_->getTsdfLayerPtr());
}

void MapServer::publishTsdf() {
  LOG(INFO) << "debug: publishing maps";
  if (tsdf_pub_.getNumSubscribers() > 0) {
    std::lock_guard<std::mutex> tsdf_layer_update_lock(
        tsdf_layer_update_mutex_);
    LOG(INFO) << "debug: publish layer size: "
              << tsdf_map_->getTsdfLayerPtr()->getMemorySize() << " "
              << tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks();
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
    LOG(INFO) << "input_odom_frame: " << frame_names_.input_odom_frame;
    LOG(INFO) << "traversable point cloud size: " << pointcloud.size();
  }
}

}  // namespace client
}  // namespace coxgraph
