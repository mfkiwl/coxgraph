#include "coxgraph_mod/tsdf_recover.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>

#include <string>

namespace voxblox {
TsdfRecover::Config TsdfRecover::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<bool>("publish_recovered_pointcloud",
                         config.publish_recovered_pointcloud,
                         config.publish_recovered_pointcloud);
  return config;
}

void TsdfRecover::subscribeToTopics() {
  mesh_sub_ = nh_.subscribe("submap_mesh_with_traj", 10,
                            &TsdfRecover::meshWithTrajCallback, this);
}

void TsdfRecover::advertiseTopics() {
  if (config_.publish_recovered_pointcloud)
    recovred_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            "recovered_pointcloud", 1, true);
  in_fov_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>("in_fov_pointcloud",
                                                            1, true);
}

void TsdfRecover::meshWithTrajCallback(
    const coxgraph_msgs::MeshWithTrajectory& mesh_msg) {
  timing::Timer mesh_process_timer("mesh_process");
  mesh_converter_->setMesh(mesh_msg.mesh.mesh);
  mesh_converter_->setTrajectory(mesh_msg.trajectory);
  mesh_converter_->convertToPointCloud();

  Transformation T_G_Sm;
  ros::Rate r(100);
  int i = 0;
  ros::Time timestamp = ros::Time(0);
  timing::Timer wait_for_tf_timer("wait_for_tf");
  // while (!transformer_.lookupTransform(
  //     world_frame_, mesh_msg.mesh.header.frame_id, now_timestamp, &T_G_Sm)) {
  // TODO(mikexyl): need better tf sync rules
  while (!tf_listener_.waitForTransform(mesh_msg.mesh.header.frame_id,
                                        world_frame_, timestamp,
                                        ros::Duration(1))) {
    LOG(WARNING) << "Failed to look up tf from " << world_frame_ << " to "
                 << static_cast<std::string>(mesh_msg.mesh.header.frame_id)
                 << " at " << timestamp << " !";
    r.sleep();
    if (i++ > 20)
      LOG(FATAL) << "Failed to look up tf from " << world_frame_ << " to "
                 << static_cast<std::string>(mesh_msg.mesh.header.frame_id)
                 << " at " << timestamp << " for 50ms !";
  }
  transformer_.lookupTransform(world_frame_, mesh_msg.mesh.header.frame_id,
                               timestamp, &T_G_Sm);
  wait_for_tf_timer.Stop();

  Transformation T_Sm_C;
  Pointcloud points_C;
  while (mesh_converter_->getPointcloudInNextFOV(&T_Sm_C, &points_C)) {
    if (points_C.empty()) continue;
    LOG_EVERY_N(INFO, 50) << "pose history size: "
                          << mesh_msg.trajectory.poses.size();
    LOG_EVERY_N(INFO, 50) << "in fov point cloud size: " << points_C.size();
    // TODO(mikexyl): only for navigation, no need color
    LOG_EVERY_N(INFO, 50) << std::endl
                          << "T_G_Sm: " << mesh_msg.mesh.header.frame_id
                          << std::endl
                          << T_G_Sm << std::endl
                          << "T_Sm_C:" << std::endl
                          << T_Sm_C;
    Colors no_colors(points_C.size(), Color());
    tsdf_integrator_->integratePointCloud(T_G_Sm * T_Sm_C, points_C, no_colors,
                                          false);
    pcl::PointCloud<pcl::PointXYZ> pointcloud_msg;
    pointcloud_msg.header.frame_id = world_frame_;
    Pointcloud points_G;
    transformPointcloud(T_G_Sm * T_Sm_C, points_C, &points_G);
    pointcloudToPclXYZ(points_G, &pointcloud_msg);
    in_fov_pointcloud_pub_.publish(pointcloud_msg);
  }

  if (config_.publish_recovered_pointcloud) {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_msg;
    pointcloud_msg.header.frame_id = mesh_msg.mesh.header.frame_id;
    pointcloudToPclXYZ(*mesh_converter_->getPointcloud(), &pointcloud_msg);
    recovred_pointcloud_pub_.publish(pointcloud_msg);
  }

  mesh_process_timer.Stop();

  if (verbose_) {
    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

}  // namespace voxblox