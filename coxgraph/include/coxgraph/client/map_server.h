#ifndef COXGRAPH_CLIENT_MAP_SERVER_H_
#define COXGRAPH_CLIENT_MAP_SERVER_H_

#include <coxgraph_msgs/GetSubmapMeshWithTraj.h>
#include <coxgraph_msgs/MeshWithTrajectory.h>
#include <coxgraph_msgs/SetTargetPose.h>
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
    Config() : publish_mesh_with_trajectory(true) {}
    bool publish_mesh_with_trajectory;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Map Server using Config:" << std::endl
        << "  Publish mesh with trjectory: "
        << static_cast<std::string>(v.publish_mesh_with_trajectory ? "enabled"
                                                                   : "disabled")
        << std::endl
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

  void publishSubmapMesh(CliSmId csid, std::string world_frame,
                         const voxgraph::SubmapVisuals& submap_vis);

  // Publish bounding box in client odom frame
  void publishSubmapBBox(CliSmId csid) {
    submap_bbox_pub_.publish(utils::msgFromBb(
        submap_collection_ptr_->getSubmap(csid).getOdomFrameSubmapAabb()));
  }

 private:
  void subscribeToTopics();
  void advertiseTopics();
  void subscribeToServices();
  void advertiseServices();
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

  ros::ServiceServer set_target_srv_;
  ros::ServiceClient get_submap_mesh_with_traj_cli_;
  bool setTargetPositionCallback(
      coxgraph_msgs::SetTargetPoseRequest& request,     // NOLINT
      coxgraph_msgs::SetTargetPoseResponse& response);  // NOLINT
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_MAP_SERVER_H_
