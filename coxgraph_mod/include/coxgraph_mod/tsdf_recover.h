#ifndef COXGRAPH_MOD_TSDF_RECOVER_H_
#define COXGRAPH_MOD_TSDF_RECOVER_H_

#include <ros/ros.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/transformer.h>
#include <voxblox_ros/tsdf_server.h>

#include <string>

#include "coxgraph_mod/common.h"
#include "coxgraph_mod/mesh_converter.h"

namespace voxblox {
class TsdfRecover : public TsdfServer {
 public:
  struct Config {
    Config() : publish_recovered_pointcloud(true) {}
    bool publish_recovered_pointcloud;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Tsdf Recover using Config:" << std::endl
        << "  Publish Recovered Pointcloud: "
        << static_cast<std::string>(v.publish_recovered_pointcloud ? "enabled"
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
    mesh_converter_.reset(new MeshConverter(
        MeshConverter::getConfigFromRosParam(nh_private_), fov));

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
};
}  // namespace voxblox

#endif  // COXGRAPH_MOD_TSDF_RECOVER_H_
