#include "coxgraph/server/visualizer/server_visualizer.h"

#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/submap_collection.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace server {

ServerVisualizer::Config ServerVisualizer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<float>("mesh_opacity", config.mesh_opacity, 1.0);
  nh_private.param<std::string>("submap_mesh_color_mode",
                                config.submap_mesh_color_mode, "lambert_color");
  nh_private.param<std::string>("combined_mesh_color_mode",
                                config.combined_mesh_color_mode, "normals");
  nh_private.param("publish_submap_meshes_every_n_sec",
                   config.publish_submap_meshes_every_n_sec,
                   config.publish_submap_meshes_every_n_sec);
  nh_private.param("o3d_visualize", config.o3d_visualize, config.o3d_visualize);
  nh_private.param("submap_registration/enabled", config.registration_enable,
                   config.registration_enable);
  return config;
}

void ServerVisualizer::getFinalGlobalMesh(
    const SubmapCollection::Ptr& submap_collection_ptr,
    const PoseGraphInterface& pose_graph_interface,
    const std::vector<CliSmPack>& other_submaps,
    const std::string& mission_frame, const ros::Publisher& publisher,
    const std::string& file_path) {
  LOG(INFO) << "Generating final mesh";

  SubmapCollection::Ptr global_submap_collection_ptr(
      new SubmapCollection(*submap_collection_ptr));
  PoseGraphInterface global_pg_interface(pose_graph_interface,
                                         global_submap_collection_ptr);

  for (auto const& submap_pack : other_submaps) {
    global_submap_collection_ptr->addSubmap(
        submap_pack.submap_ptr, submap_pack.cid, submap_pack.cli_sm_id);
    global_pg_interface.addSubmap(submap_pack.submap_ptr->getID());
  }
  if (global_submap_collection_ptr->getSubmapConstPtrs().empty()) return;

  global_pg_interface.updateSubmapRPConstraints();

  auto opt_async =
      std::async(std::launch::async, &PoseGraphInterface::optimize,
                 &global_pg_interface, config_.registration_enable);

  while (opt_async.wait_for(std::chrono::milliseconds(100)) !=
         std::future_status::ready) {
    LOG_EVERY_N(INFO, 10) << "Global optimzation is still running...";
  }
  LOG(INFO) << "Optimization finished, generating global mesh...";

  global_pg_interface.updateSubmapCollectionPoses();

  auto pose_map = global_pg_interface.getPoseMap();

  boost::filesystem::path mesh_p_o3d(file_path);
  boost::filesystem::path mesh_p_voxblox(file_path);
  mesh_p_o3d.append("global_mesh_o3d.ply");
  mesh_p_voxblox.append("global_mesh_voxblox.ply");

  if (config_.o3d_visualize) {
    // Combine mesh
    o3d_vis_->ClearGeometries();
    std::shared_ptr<open3d::geometry::TriangleMesh> combined_mesh(
        new open3d::geometry::TriangleMesh());
    for (auto const& submap :
         global_submap_collection_ptr->getSubmapConstPtrs()) {
      auto submap_mesh = utils::o3dMeshFromMsg(*submap->mesh_pointcloud_);
      if (submap_mesh == nullptr) continue;
      submap_mesh->Transform(
          pose_map[submap->getID()].cast<double>().getTransformationMatrix());
      *combined_mesh += *submap_mesh;
      combined_mesh->MergeCloseVertices(0.06);
      combined_mesh->RemoveDuplicatedVertices();
      combined_mesh->RemoveDuplicatedTriangles();
    }
    combined_mesh->FilterSmoothTaubin(100);
    combined_mesh->ComputeVertexNormals();
    combined_mesh->ComputeTriangleNormals();
    o3d_vis_->AddGeometry(combined_mesh);
    o3d_vis_->UpdateGeometry(combined_mesh);
    open3d::io::WriteTriangleMesh(mesh_p_o3d.string(), *combined_mesh);
  }

  submap_vis_.saveAndPubCombinedMesh(*global_submap_collection_ptr,
                                     mission_frame, publisher,
                                     mesh_p_voxblox.string());

  std::map<SerSmId, CIdCSIdPair> sm_cli_ids;
  for (auto const& submap : global_submap_collection_ptr->getSubmapPtrs()) {
    auto csid_pair =
        global_submap_collection_ptr->getCliIdPairBySsid(submap->getID());
    sm_cli_ids.emplace(submap->getID(),
                       std::make_pair(csid_pair.first, csid_pair.second));
  }

  LOG(INFO) << "Global mesh generated, published and saved to " << file_path;

  global_submap_collection_ptr->savePoseHistoryToFile(file_path);

  LOG(INFO) << "Trajectory saved to " << file_path;
}

}  // namespace server
}  // namespace coxgraph
