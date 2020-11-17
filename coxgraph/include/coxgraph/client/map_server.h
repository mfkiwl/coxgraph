#ifndef COXGRAPH_CLIENT_MAP_SERVER_H_
#define COXGRAPH_CLIENT_MAP_SERVER_H_

#include <ros/ros.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/ros_params.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <memory>
#include <mutex>

#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace client {

class MapServer {
 public:
  struct Config {
    Config() : publish_map_every_n_sec(1.0) {}
    float publish_map_every_n_sec;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Map Server using Config:" << std::endl
        << "  Publish maps every: " << v.publish_map_every_n_sec << "s"
        << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<MapServer> Ptr;
  using VoxgraphSubmapCollection = voxgraph::VoxgraphSubmapCollection;

  MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            voxgraph::VoxgraphSubmap::Config map_config,
            VoxgraphSubmapCollection::Ptr submap_collection_ptr)
      : MapServer(nh, nh_private, getConfigFromRosParam(nh_private), map_config,
                  voxblox::getEsdfIntegratorConfigFromRosParam(nh_private),
                  submap_collection_ptr) {}

  MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            const Config& config, voxgraph::VoxgraphSubmap::Config map_config,
            voxblox::EsdfIntegrator::Config esdf_integrator_config,
            VoxgraphSubmapCollection::Ptr submap_collection_ptr)
      : nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)),
        submap_collection_ptr_(submap_collection_ptr) {
    tsdf_map_.reset(new voxblox::TsdfMap(
        static_cast<voxblox::TsdfMap::Config>(map_config)));
    esdf_map_.reset(new voxblox::EsdfMap(
        static_cast<voxblox::EsdfMap::Config>(map_config)));
    esdf_integrator_.reset(new voxblox::EsdfIntegrator(
        esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
        esdf_map_->getEsdfLayerPtr()));

    subscribeTopics();
    advertiseTopics();

    LOG(INFO) << config_;
  }

  ~MapServer() = default;

  void updatePastTsdf();

 private:
  void subscribeTopics();
  void advertiseTopics();

  void activeTsdfCallback(const voxblox_msgs::Layer& layer_msg);
  void publishMap(const ros::TimerEvent& event) {
    if (tsdf_pub_.getNumSubscribers() > 0) {
      std::lock_guard<std::mutex> tsdf_layer_update_lock(
          tsdf_layer_update_mutex_);
      voxblox_msgs::Layer layer_msg;
      voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
          tsdf_map_->getTsdfLayer(), false, &layer_msg);
      tsdf_pub_.publish(layer_msg);
    }

    if (esdf_pub_.getNumSubscribers() > 0) {
      std::lock_guard<std::mutex> esdf_layer_update_lock(
          esdf_layer_update_mutex_);
      updateEsdfBatch();
      voxblox_msgs::Layer layer_msg;
      voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(
          esdf_map_->getEsdfLayer(), false, &layer_msg);
      esdf_pub_.publish(layer_msg);
    }
  }

  Config config_;

  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer map_pub_timer_;
  ros::Publisher tsdf_pub_;
  ros::Publisher esdf_pub_;
  ros::Subscriber active_tsdf_sub_;
  ros::Subscriber active_esdf_sub_;

  voxblox::TsdfMap::Ptr tsdf_map_;
  std::mutex tsdf_layer_update_mutex_;

  voxblox::EsdfMap::Ptr esdf_map_;
  std::unique_ptr<voxblox::EsdfIntegrator> esdf_integrator_;
  std::mutex esdf_layer_update_mutex_;
  inline void updateEsdfBatch() {
    if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
      esdf_integrator_->updateFromTsdfLayerBatch();
    }
  }
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_MAP_SERVER_H_
