#ifndef COXGRAPH_SERVER_VISUALIZER_SERVER_VISUALIZER_H_
#define COXGRAPH_SERVER_VISUALIZER_SERVER_VISUALIZER_H_

#include <Open3D/Geometry/TriangleMesh.h>
#include <Open3D/Visualization/Visualizer/Visualizer.h>
#include <cblox/mesh/submap_mesher.h>
#include <ros/ros.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/MultiMesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxgraph/tools/visualization/submap_visuals.h>

#include <memory>
#include <string>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/pose_graph_interface.h"
#include "coxgraph/server/submap_collection.h"
#include "coxgraph/server/visualizer/mesh_collection.h"

namespace coxgraph {
namespace server {
class ServerVisualizer {
 public:
  struct Config {
    float mesh_opacity = 1.0;
    std::string submap_mesh_color_mode = "lambert_color";
    std::string combined_mesh_color_mode = "normals";
    float publish_submap_meshes_every_n_sec = 1.0;
    bool o3d_visualize = true;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "ServerVisualizer using Config:" << std::endl
        << std::endl
        << "  Mesh Opacity: " << v.mesh_opacity << std::endl
        << "  Submap Mesh Color Mode: " << v.submap_mesh_color_mode << std::endl
        << "  Combined Mesh Color Mode: " << v.combined_mesh_color_mode
        << std::endl
        << "  Publish Submap Meshes Every: "
        << v.publish_submap_meshes_every_n_sec << " s" << std::endl
        << "  o3d_visualize: " << v.o3d_visualize << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<ServerVisualizer> Ptr;

  ServerVisualizer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                   const CliSmConfig& submap_config,
                   const MeshIntegratorConfig& mesh_config)
      : nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)),
        submap_config_(submap_config),
        mesh_config_(mesh_config),
        submap_vis_(submap_config, mesh_config),
        mesh_collection_ptr_(new MeshCollection()) {
    LOG(INFO) << config_;

    setMeshOpacity(config_.mesh_opacity);
    setSubmapMeshColorMode(
        voxblox::getColorModeFromString(config_.submap_mesh_color_mode));
    setCombinedMeshColorMode(
        voxblox::getColorModeFromString(config_.combined_mesh_color_mode));

    advertiseTopics();

    if (config_.o3d_visualize) {
      o3d_vis_ = new open3d::visualization::Visualizer();
      o3d_vis_->CreateVisualizerWindow("global_mesh");
      o3d_vis_->GetRenderOption().mesh_color_option_ =
          open3d::visualization::RenderOption::MeshColorOption::Normal;
      combined_mesh_.reset(new open3d::geometry::TriangleMesh());
      // o3d_vis_->AddGeometry(combined_mesh_);
      o3d_vis_update_timer_ = nh_private_.createTimer(
          ros::Duration(0.01), &ServerVisualizer::o3dVisUpdateEvent, this);
    }
  }

  ~ServerVisualizer() = default;

  void setMeshOpacity(float mesh_opacity) {
    submap_vis_.setMeshOpacity(mesh_opacity);
  }
  void setSubmapMeshColorMode(voxblox::ColorMode color_mode) {
    submap_vis_.setSubmapMeshColorMode(color_mode);
  }
  void setCombinedMeshColorMode(voxblox::ColorMode color_mode) {
    submap_vis_.setCombinedMeshColorMode(color_mode);
  }

  void advertiseTopics() {
    combined_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::Mesh>("combined_mesh", 10, true);
    separated_mesh_pub_ = nh_private_.advertise<voxblox_msgs::MultiMesh>(
        "separated_mesh", 10, true);
    if (config_.publish_submap_meshes_every_n_sec > 0)
      submap_mesh_pub_timer_ = nh_private_.createTimer(
          ros::Duration(config_.publish_submap_meshes_every_n_sec),
          &ServerVisualizer::publishSubmapMeshesCallback, this);
  }

  MeshCollection::Ptr getMeshCollectionPtr() { return mesh_collection_ptr_; }

  void addSubmapMesh(CliId cid, CliSmId csid,
                     coxgraph_msgs::MeshWithTrajectory mesh_with_traj) {
    mesh_collection_ptr_->addSubmapMesh(cid, csid, mesh_with_traj);
  }

  /**
   * @brief Get the Final Global Mesh object, other submaps will be added to
   * submap_collection and pose graph to optimize, then removed, so
   * submap_collection and pose graph will not be changed.
   *
   * @param submap_collection_ptr
   * @param pose_graph_interface
   * @param other_submaps
   */
  void getFinalGlobalMesh(const SubmapCollection::Ptr& submap_collection_ptr,
                          const PoseGraphInterface& pose_graph_interface,
                          const std::vector<CliSmPack>& other_submaps,
                          const std::string& mission_frame,
                          const ros::Publisher& publisher,
                          const std::string& file_path);

  void getFinalGlobalMesh(const SubmapCollection::Ptr& submap_collection_ptr,
                          const PoseGraphInterface& pose_graph_interface,
                          const std::vector<CliSmPack>& other_submaps,
                          const std::string& mission_frame,
                          const std::string& file_path) {
    getFinalGlobalMesh(submap_collection_ptr, pose_graph_interface,
                       other_submaps, mission_frame, combined_mesh_pub_,
                       file_path);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Config config_;

  CliSmConfig submap_config_;
  MeshIntegratorConfig mesh_config_;

  voxgraph::SubmapVisuals submap_vis_;

  MeshCollection::Ptr mesh_collection_ptr_;
  ros::Timer submap_mesh_pub_timer_;
  ros::Publisher combined_mesh_pub_;
  ros::Publisher separated_mesh_pub_;
  void publishSubmapMeshesCallback(const ros::TimerEvent& event) {
    publishSubmapMeshes();
  }
  void publishSubmapMeshes() {
    for (auto& kv : *mesh_collection_ptr_->getSubmapMeshesPtr()) {
      kv.second.mesh.header.stamp = ros::Time::now();
      kv.second.mesh.mesh.header.stamp = ros::Time::now();
      separated_mesh_pub_.publish(kv.second.mesh);
    }
  }

  open3d::visualization::Visualizer* o3d_vis_;
  std::shared_ptr<open3d::geometry::TriangleMesh> combined_mesh_;
  ros::Timer o3d_vis_update_timer_;
  void o3dVisUpdateEvent(const ros::TimerEvent& /*event*/) {
    o3d_vis_->PollEvents();
    o3d_vis_->UpdateRender();
  }
};  // namespace server

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_VISUALIZER_SERVER_VISUALIZER_H_
