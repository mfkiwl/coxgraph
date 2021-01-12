#ifndef COXGRAPH_CLIENT_MAP_SERVER_H_
#define COXGRAPH_CLIENT_MAP_SERVER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxblox_ros/ros_params.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>
#include <voxgraph/tools/visualization/submap_visuals.h>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "coxgraph/common.h"
#include "coxgraph/map_comm/client_handler.h"
#include "coxgraph/map_comm/submap_collection.h"
#include "coxgraph/utils/msg_converter.h"
#include "coxgraph/utils/submap_info_listener.h"

namespace coxgraph {
namespace client {

class MapServer {
 public:
  struct Config {
    Config()
        : publish_combined_maps_every_n_sec(0.0),
          publish_on_update(true),
          publish_traversable(false),
          traversability_radius(1.0),
          publish_mesh_with_trajectory(true),
          request_new_submap_every_n_sec(1.0) {}
    float publish_combined_maps_every_n_sec;
    bool publish_on_update;
    bool publish_traversable;
    float traversability_radius;
    bool publish_mesh_with_trajectory;
    float request_new_submap_every_n_sec;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Map Server using Config:" << std::endl
        << "  Publish maps every: " << v.publish_combined_maps_every_n_sec
        << "s" << std::endl
        << "  Publish maps on update: "
        << static_cast<std::string>(v.publish_on_update ? "enabled"
                                                        : "disabled")
        << std::endl
        << "  Publish traversable: "
        << static_cast<std::string>(v.publish_traversable ? "enabled"
                                                          : "disabled")
        << std::endl
        << "  Traversability radius: " << v.traversability_radius << std::endl
        << "  Publish mesh with trjectory: "
        << static_cast<std::string>(v.publish_mesh_with_trajectory ? "enabled"
                                                                   : "disabled")
        << std::endl
        << "  Request new submap every: " << v.request_new_submap_every_n_sec
        << "s" << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<MapServer> Ptr;
  using VoxgraphSubmapCollection = voxgraph::VoxgraphSubmapCollection;

  MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            CliId client_id, int client_number,
            const voxgraph::VoxgraphSubmap::Config& map_config,
            const FrameNames& frame_names,
            const comm::SubmapCollection::Ptr& submap_collection_ptr)
      : MapServer(nh, nh_private, client_id, client_number,
                  getConfigFromRosParam(nh_private), map_config, frame_names,
                  voxblox::getEsdfIntegratorConfigFromRosParam(nh_private),
                  submap_collection_ptr) {}

  MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            CliId client_id, int client_number, const Config& config,
            const voxgraph::VoxgraphSubmap::Config& map_config,
            const FrameNames& frame_names,
            const voxblox::EsdfIntegrator::Config& esdf_integrator_config,
            const comm::SubmapCollection::Ptr& submap_collection_ptr)
      : nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)),
        client_id_(client_id),
        frame_names_(frame_names),
        submap_collection_ptr_(submap_collection_ptr) {
    tsdf_map_.reset(new voxblox::TsdfMap(
        static_cast<voxblox::TsdfMap::Config>(map_config)));
    esdf_map_.reset(new voxblox::EsdfMap(
        static_cast<voxblox::EsdfMap::Config>(map_config)));
    esdf_integrator_.reset(new voxblox::EsdfIntegrator(
        esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
        esdf_map_->getEsdfLayerPtr()));
    subscribeToTopics();
    advertiseTopics();

    comm::ClientHandler::initClientHandlers(
        nh_, nh_private_, frame_names_.input_odom_frame, map_config,
        submap_collection_ptr, client_number, &client_handlers_, client_id);
    startTimers();

    LOG(INFO) << config_;
  }

  ~MapServer() = default;

  void publishSubmapMesh(CliSmId csid, std::string world_frame,
                         const voxgraph::SubmapVisuals& submap_vis);

  // Publish bounding box in client odom frame
  void publishSubmapBBox(CliSmId csid) {
    submap_bbox_pub_.publish(utils::msgFromBb(
        submap_collection_ptr_->getSubmap(csid).getOdomFrameSubmapAabb()));
  }

  void updatePastTsdf();

 private:
  void subscribeToTopics();
  void advertiseTopics();
  void startTimers();

  void publishMapEvent(const ros::TimerEvent& event);
  void publishMap();
  void mergeTsdfs();
  void publishTsdf();
  void publishEsdf();
  void publishTraversable();

  Config config_;

  CliId client_id_;

  FrameNames frame_names_;

  comm::SubmapCollection::Ptr submap_collection_ptr_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer map_pub_timer_;
  ros::Publisher tsdf_pub_;
  ros::Publisher esdf_pub_;
  ros::Publisher traversable_pub_;

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

  // TODO(mikexyl): better move pubs to submap_info_listener
  ros::Publisher submap_mesh_pub_;
  ros::Publisher submap_bbox_pub_;

  ros::Subscriber kf_pose_sub_;
  std::set<ros::Time> kf_timestamp_set_;
  void kfPoseCallback(const nav_msgs::Odometry& kf_pose_msg) {
    if (kf_timestamp_set_.size() > kKfTimestampQueueSize)
      kf_timestamp_set_.erase(kf_timestamp_set_.begin());
    kf_timestamp_set_.emplace(kf_pose_msg.header.stamp);
  }
  constexpr static int kKfTimestampQueueSize = 400;

  utils::SubmapInfoListener submap_info_listener_;

  ros::Timer request_new_submap_timer_;
  void requestNewSubmapEvent(const ros::TimerEvent& event) {
    requestNewSubmap();
  }
  void requestNewSubmap();

  std::map<CliId, comm::ClientHandler::Ptr> client_handlers_;
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_MAP_SERVER_H_
