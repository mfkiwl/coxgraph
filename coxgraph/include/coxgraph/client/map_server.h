#ifndef COXGRAPH_CLIENT_MAP_SERVER_H_
#define COXGRAPH_CLIENT_MAP_SERVER_H_

#include <coxgraph_msgs/GetSubmapMeshWithTraj.h>
#include <coxgraph_msgs/MeshWithTrajectory.h>
#include <coxgraph_msgs/SetTargetPose.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
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
#include "coxgraph/map_comm/projected_map_server.h"
#include "coxgraph/map_comm/submap_collection.h"
#include "coxgraph/utils/msg_converter.h"
#include "coxgraph/utils/submap_info_listener.h"
#include "pcl/point_types.h"

namespace coxgraph {
namespace client {

class MapServer {
 public:
  struct Config {
    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Map Server using Config:" << std::endl
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
                  submap_collection_ptr) {}

  MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            CliId client_id, int client_number, const Config& config,
            const voxgraph::VoxgraphSubmap::Config& map_config,
            const FrameNames& frame_names,
            const comm::SubmapCollection::Ptr& submap_collection_ptr)
      : nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)),
        client_id_(client_id),
        frame_names_(frame_names),
        submap_collection_ptr_(submap_collection_ptr) {
    subscribeToTopics();
    advertiseTopics();
    advertiseServices();
    startTimers();

    LOG(INFO) << config_;
  }

  ~MapServer() = default;

 private:
  void subscribeToTopics();
  void advertiseTopics();
  void subscribeToServices();
  void advertiseServices();
  void startTimers();

  Config config_;

  CliId client_id_;

  FrameNames frame_names_;

  comm::SubmapCollection::Ptr submap_collection_ptr_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_MAP_SERVER_H_
