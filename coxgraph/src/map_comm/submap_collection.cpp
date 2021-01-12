#include "coxgraph/map_comm/submap_collection.h"

#include <voxblox/integrator/merge_integration.h>

#include <memory>
#include <utility>
#include <vector>

#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace comm {

void SubmapCollection::addSubmap(const CliSm::Ptr& submap_ptr, const CliId& cid,
                                 const CliSmId& csid) {
  CHECK(submap_ptr != nullptr);
  voxgraph::VoxgraphSubmapCollection::addSubmap(submap_ptr);

  std::lock_guard<std::mutex> id_update_lock(id_update_mutex_);
  sm_cli_id_map_.emplace(submap_ptr->getID(), CIdCSIdPair(cid, csid));
  if (!cli_ser_sm_id_map_.count(submap_ptr->getID())) {
    cli_ser_sm_id_map_.emplace(cid, std::vector<SerSmId>());
  }
  cli_ser_sm_id_map_[cid].emplace_back(submap_ptr->getID());
  sm_id_ori_pose_map_.emplace(submap_ptr->getID(), submap_ptr->getPose());
}

void SubmapCollection::addSubmapFromMeshAsync(
    const coxgraph_msgs::MeshWithTrajectory::Ptr& submap_mesh, const CliId& cid,
    const CliSmId& csid) {
  if (!recover_threads_.count(cid))
    recover_threads_.emplace(cid, std::thread());
  else
    recover_threads_[cid].join();
  recover_threads_[cid] = std::thread(&SubmapCollection::addSubmapFromMesh,
                                      this, submap_mesh, cid, csid);
}

void SubmapCollection::addSubmapFromMesh(
    const coxgraph_msgs::MeshWithTrajectory::Ptr& submap_mesh, const CliId& cid,
    const CliSmId& csid) {
  voxblox::timing::Timer recover_tsdf_timer("recover_tsdf");

  // If a submap is already generated from tsdf, don't add it
  SerSmId ssid;
  if (getSerSmIdByCliSmId(cid, csid, &ssid)) return;

  mesh_collection_ptr_->addSubmapMesh(submap_mesh, cid, csid);

  CIdCSIdPair csid_pair =
      utils::resolveSubmapFrame(submap_mesh->mesh.header.frame_id);

  CliSm submap = draftNewSubmap();
  voxblox::MeshConverter mesh_converter(nh_private_);
  voxblox::TsdfIntegratorBase::Ptr tsdf_integrator =
      voxblox::TsdfIntegratorFactory::create(
          tsdf_integrator_method_, tsdf_integrator_config_,
          submap.getTsdfMapPtr()->getTsdfLayerPtr());

  Transformation T_Sm_C;
  voxblox::Pointcloud points_C;

  mesh_converter.setMesh(submap_mesh->mesh.mesh);
  mesh_converter.setTrajectory(submap_mesh->trajectory);
  mesh_converter.convertToPointCloud();

  while (mesh_converter.getPointcloudInNextFOV(&T_Sm_C, &points_C)) {
    // LOG(INFO) << "in fov points: " << points_C.size();
    if (points_C.empty()) continue;

    // Only for navigation, no need color
    tsdf_integrator->integratePointCloud(T_Sm_C, points_C, voxblox::Colors(),
                                         false);
  }

  {
    std::lock_guard<std::mutex> recovered_submap_map_lock(
        recovered_submap_map_mutex_);
    recovered_submap_map_.emplace(submap_mesh->mesh.header.frame_id,
                                  std::make_shared<CliSm>(std::move(submap)));
  }

  recover_tsdf_timer.Stop();
  LOG(INFO) << voxblox::timing::Timing::Print();
}

// TODO(mikexyl): delete this
Transformation SubmapCollection::mergeToCliMap(const CliSm::Ptr& submap_ptr) {
  CHECK(exists(submap_ptr->getID()));

  auto const& cli_map_ptr = getSubmapPtr(submap_ptr->getID());
  // TODO(mikexyl): only merge layers now, merge more if needed, and
  // theoretically not need to transform layer since it's already done when
  // generating submap from msg
  voxblox::mergeLayerAintoLayerB(
      submap_ptr->getTsdfMapPtr()->getTsdfLayer(),
      cli_map_ptr->getTsdfMapPtr()->getTsdfLayerPtr());

  cli_map_ptr->finishSubmap();
  return submap_ptr->getPose() * cli_map_ptr->getPose().inverse();
}

voxblox::TsdfMap::Ptr SubmapCollection::getProjectedMap() {
  voxblox::TsdfMap::Ptr combined_tsdf_map =
      voxgraph::VoxgraphSubmapCollection::getProjectedMap();

  // Also project mesh-recovered submaps
  for (auto const& submap_kv : recovered_submap_map_) {
    Transformation T_G_Sm;
    if (submap_info_listener_.getSubmapPoseBlocking(submap_kv.first, &T_G_Sm,
                                                    true))
      voxblox::mergeLayerAintoLayerB(
          submap_kv.second->getTsdfMap().getTsdfLayer(), T_G_Sm,
          combined_tsdf_map->getTsdfLayerPtr());
    else
      LOG(ERROR) << "Failed to project " << submap_kv.first;
  }
  LOG(INFO)
      << "Projected tsdf blocks: "
      << combined_tsdf_map->getTsdfLayerPtr()->getNumberOfAllocatedBlocks();

  return combined_tsdf_map;
}

}  // namespace comm
}  // namespace coxgraph
