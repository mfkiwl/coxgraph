#include "coxgraph/map_comm/tsdf_recover.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>

#include <string>

#include "coxgraph/utils/msg_converter.h"

namespace voxblox {
TsdfRecover::Config TsdfRecover::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<bool>("publish_recovered_pointcloud",
                         config.publish_recovered_pointcloud,
                         config.publish_recovered_pointcloud);
  nh_private.param<bool>("use_tf_submap_pose", config.use_tf_submap_pose,
                         config.use_tf_submap_pose);
  return config;
}

void TsdfRecover::subscribeToTopics() {
  mesh_sub_ = nh_private_.subscribe("submap_mesh_with_traj", 10,
                                    &TsdfRecover::meshCallback, this);
  if (!config_.use_tf_submap_pose)
    submap_pose_sub_ = nh_private_.subscribe(
        "submap_poses", 10, &TsdfRecover::submapPoseCallback, this);
  else
    LOG(FATAL) << "Don't turn on use_tf_submap_pose, bug unfix";
}

void TsdfRecover::advertiseTopics() {
  if (config_.publish_recovered_pointcloud)
    recovered_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            "recovered_pointcloud", 1, true);
  frame_pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
      "in_fov_pointcloud", 1, true);
}

void TsdfRecover::meshCallback(const voxblox_msgs::Mesh& mesh_msg) {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  timing::Timer mesh_process_timer("mesh_process");
  mesh_converter_->setMesh(mesh_msg);
  mesh_converter_->convertToPointCloud();

  Transformation T_G_C;
  PointcloudPtr points_C(new Pointcloud());
  int i = 0;
  while (mesh_converter_->getNextPointcloud(&i, &T_G_C, &points_C)) {
    if (points_C->empty()) continue;

    // Only for navigation, no need color
    Colors no_colors(points_C->size(), Color());
    tsdf_integrator_->integratePointCloud(T_G_C, *points_C, no_colors, false);

    if (frame_pointcloud_pub_.getNumSubscribers() > 0) {
      pcl::PointCloud<pcl::PointXYZ> pointcloud_msg;
      pointcloud_msg.header.frame_id = world_frame_;
      Pointcloud points_G;
      transformPointcloud(T_G_C, *points_C, &points_G);
      pointcloudToPclXYZ(points_G, &pointcloud_msg);
      frame_pointcloud_pub_.publish(pointcloud_msg);
    }
  }

  if (config_.publish_recovered_pointcloud &&
      recovered_pointcloud_pub_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_msg;
    pointcloud_msg.header.frame_id = mesh_msg.header.frame_id;
    pointcloudToPclXYZ(mesh_converter_->getCombinedPointcloud(),
                       &pointcloud_msg);
    recovered_pointcloud_pub_.publish(pointcloud_msg);
  }

  mesh_process_timer.Stop();

  voxblox_msgs::LayerWithTrajectory layer_with_traj;
  serializeLayerAsMsg(tsdf_map_->getTsdfLayer(), false, &layer_with_traj.layer);
  layer_with_traj.trajectory = mesh_msg.trajectory;
  tsdf_map_pub_.publish(layer_with_traj);

  if (verbose_) {
    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

}  // namespace voxblox
