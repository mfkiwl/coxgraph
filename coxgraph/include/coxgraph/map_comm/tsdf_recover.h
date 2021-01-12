#ifndef COXGRAPH_MAP_COMM_TSDF_RECOVER_H_
#define COXGRAPH_MAP_COMM_TSDF_RECOVER_H_

#include <cblox_msgs/MapPoseUpdates.h>
#include <ros/ros.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/transformer.h>
#include <voxblox_ros/tsdf_server.h>

#include <map>
#include <string>

#include "coxgraph/map_comm/mesh_converter.h"

namespace voxblox {
class TsdfRecover : public TsdfServer {
 public:
  struct Config {
    Config() : publish_recovered_pointcloud(true), use_tf_submap_pose(false) {}
    bool publish_recovered_pointcloud;
    bool use_tf_submap_pose;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Tsdf Recover using Config:" << std::endl
        << "  Publish Recovered Pointcloud: "
        << static_cast<std::string>(v.publish_recovered_pointcloud ? "enabled"
                                                                   : "disabled")
        << std::endl
        << "  Use TF Submap Pose: "
        << static_cast<std::string>(v.use_tf_submap_pose ? "enabled"
                                                         : "disabled")
        << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  TsdfRecover(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)),
        TsdfServer(nh, nh_private, getTsdfMapConfigFromRosParam(nh_private),
                   getTsdfIntegratorConfigFromRosParam(nh_private),
                   getMeshIntegratorConfigFromRosParam(nh_private)) {
    MeshConverter::FOV fov;
    fov.max_ray_length_m = tsdf_integrator_->getConfig().max_ray_length_m;
    fov.min_ray_length_m = tsdf_integrator_->getConfig().min_ray_length_m;
    nh_private_.param<float>("fov_horizontal_degree", fov.horizontal_degree,
                             90);
    nh_private_.param<float>("fov_vertical_degree", fov.vertical_degree, 60);
    mesh_converter_.reset(new MeshConverter(nh_private_));

    subscribeToTopics();
    advertiseTopics();
  }

  ~TsdfRecover() = default;

 private:
  void subscribeToTopics();
  void advertiseTopics();

  void meshWithTrajCallback(const coxgraph_msgs::MeshWithTrajectory& mesh_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Config config_;

  ros::Subscriber mesh_sub_;
  ros::Publisher recovred_pointcloud_pub_;
  ros::Publisher in_fov_pointcloud_pub_;

  MeshConverter::Ptr mesh_converter_;

  tf::TransformListener tf_listener_;

  ros::Subscriber submap_pose_sub_;
  std::map<int16_t, Transformation> submap_pose_map_;
  void submapPoseCallback(const cblox_msgs::MapPoseUpdates& pose_update_msg) {
    for (auto const& map_header : pose_update_msg.map_headers) {
      kindr::minimal::QuatTransformationTemplate<double> T_G_Sm;
      tf::poseMsgToKindr(map_header.pose_estimate.map_pose, &T_G_Sm);
      auto submap_pose_kv = submap_pose_map_.find(map_header.id);
      if (submap_pose_kv == submap_pose_map_.end()) {
        submap_pose_map_.emplace(map_header.id, T_G_Sm.cast<FloatingPoint>());
      } else {
        submap_pose_kv->second = T_G_Sm.cast<FloatingPoint>();
      }
    }
  }

  bool getSubmapPoseBlocking(int16_t submap_id, Transformation* T_G_Sm) {
    float loop_ms = 10;
    ros::Rate loop(ros::Duration(loop_ms / 1000));
    float wait_time_ms = 0;
    while (!submap_pose_map_.count(submap_id)) {
      loop.sleep();
      wait_time_ms += loop_ms;
      if (wait_time_ms > 500) {
        return false;
      }
    }
    *T_G_Sm = submap_pose_map_.find(submap_id)->second;
    return true;
  }

  constexpr static float kSubmapPoseToleranceMs = 500;
};
}  // namespace voxblox

#endif  //  COXGRAPH_MAP_COMM_TSDF_RECOVER_H_
