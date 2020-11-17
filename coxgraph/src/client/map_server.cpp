#include "coxgraph/client/map_server.h"

namespace coxgraph {
namespace client {

MapServer::Config MapServer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<float>("publish_map_every_n_sec",
                          config.publish_map_every_n_sec,
                          config.publish_map_every_n_sec);
  return config;
}

void MapServer::subscribeTopics() {
  if (config_.publish_map_every_n_sec == 0.0) return;
  active_tsdf_sub_ = nh_private_.subscribe(
      "active_tsdf_in", 10, &MapServer::activeTsdfCallback, this);
}

void MapServer::advertiseTopics() {
  if (config_.publish_map_every_n_sec == 0.0) return;
  tsdf_pub_ = nh_private_.advertise<voxblox_msgs::Layer>("tsdf_out", 10, true);
  esdf_pub_ = nh_private_.advertise<voxblox_msgs::Layer>("esdf_out", 10, true);
  map_pub_timer_ =
      nh_private_.createTimer(ros::Duration(config_.publish_map_every_n_sec),
                              &MapServer::publishMap, this);
}

void MapServer::activeTsdfCallback(const voxblox_msgs::Layer& layer_msg) {
  CHECK_EQ(layer_msg.layer_type, voxblox::voxel_types::kTsdf);
  CHECK_EQ(layer_msg.voxel_size,
           tsdf_map_->getTsdfLayerConstPtr()->voxel_size());
  CHECK_EQ(layer_msg.voxels_per_side,
           tsdf_map_->getTsdfLayerConstPtr()->voxels_per_side());
  voxblox::Layer<voxblox::TsdfVoxel> tsdf_layer(layer_msg.voxel_size,
                                                layer_msg.voxels_per_side);
  if (!voxblox::deserializeMsgToLayer(layer_msg, &tsdf_layer)) {
    LOG(FATAL)
        << "Received a submap msg with an invalid TSDF. Skipping submap.";
  }

  std::lock_guard<std::mutex> tsdf_layer_update_lock(tsdf_layer_update_mutex_);
  voxblox::mergeLayerAintoLayerB(tsdf_layer, tsdf_map_->getTsdfLayerPtr());
}

void MapServer::updatePastTsdf() {
  if (config_.publish_map_every_n_sec == 0.0) return;
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  for (auto const& submap_ptr : submap_collection_ptr_->getSubmapPtrs()) {
    voxblox::mergeLayerAintoLayerB(submap_ptr->getTsdfMapPtr()->getTsdfLayer(),
                                   submap_ptr->getPose(),
                                   tsdf_map_->getTsdfLayerPtr());
  }
}

}  // namespace client
}  // namespace coxgraph
