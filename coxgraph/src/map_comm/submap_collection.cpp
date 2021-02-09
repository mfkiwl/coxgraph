#include "coxgraph/map_comm/submap_collection.h"

#include <voxblox/integrator/merge_integration.h>

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace comm {

void SubmapCollection::addSubmap(CliSm&& submap, const CliId& cid,
                                 const CliSmId& csid) {
  voxgraph::VoxgraphSubmapCollection::addSubmap(submap);

  addSubmapID(submap.getID(), cid, csid);
}

void SubmapCollection::addSubmap(const CliSm::Ptr& submap_ptr, const CliId& cid,
                                 const CliSmId& csid) {
  CHECK(submap_ptr != nullptr);
  voxgraph::VoxgraphSubmapCollection::addSubmap(submap_ptr);

  addSubmapID(submap_ptr->getID(), cid, csid);
}

void SubmapCollection::addSubmapID(const SerSmId& ssid, const CliId& cid,
                                   const CliSmId& csid) {
  std::lock_guard<std::mutex> id_update_lock(id_update_mutex_);
  sm_cli_id_map_.emplace(ssid, CIdCSIdPair(cid, csid));
  if (!cli_ser_sm_id_map_.count(ssid)) {
    cli_ser_sm_id_map_.emplace(cid, std::vector<SerSmId>());
  }
  cli_ser_sm_id_map_[cid].emplace_back(ssid);
  sm_id_ori_pose_map_.emplace(ssid, getSubmapPtr(ssid)->getPose());
}

void SubmapCollection::addSubmapFromMeshAsync(
    const voxblox_msgs::Mesh& submap_mesh, const CliId& cid,
    const CliSmId& csid) {
  if (!recover_threads_.count(cid))
    recover_threads_.emplace(cid, std::thread());
  else
    recover_threads_[cid].join();
  recover_threads_[cid] = std::thread(&SubmapCollection::addSubmapFromMesh,
                                      this, submap_mesh, cid, csid);
}

void SubmapCollection::addSubmapFromMesh(const voxblox_msgs::Mesh& submap_mesh,
                                         const CliId& cid,
                                         const CliSmId& csid) {
  voxblox::timing::Timer recover_tsdf_timer("recover_tsdf");

  // If a submap is already generated from tsdf, don't add it
  SerSmId ssid;
  if (getSerSmIdByCliSmId(cid, csid, &ssid)) return;

  mesh_collection_ptr_->addSubmapMesh(submap_mesh, cid, csid);

  CIdCSIdPair csid_pair =
      utils::resolveSubmapFrame(submap_mesh.header.frame_id);

  CliSm submap = draftNewSubmap();
  voxblox::MeshConverter mesh_converter(nh_private_);
  voxblox::TsdfIntegratorBase::Ptr tsdf_integrator =
      voxblox::TsdfIntegratorFactory::create(
          tsdf_integrator_method_, tsdf_integrator_config_,
          submap.getTsdfMapPtr()->getTsdfLayerPtr());

  Transformation T_Sm_C;
  voxblox::PointcloudPtr points_C(new voxblox::Pointcloud());

  mesh_converter.setMesh(submap_mesh);
  mesh_converter.convertToPointCloud();

  int i = 0;
  while (mesh_converter.getNextPointcloud(&i, &T_Sm_C, &points_C)) {
    // LOG(INFO) << "in fov points: " << points_C.size();
    if (points_C->empty()) continue;

    // Only for navigation, no need color
    tsdf_integrator->integratePointCloud(T_Sm_C, *points_C, voxblox::Colors(),
                                         false);
  }

  if (!optimize_recovered_map_) {
    std::lock_guard<std::mutex> recovered_submap_map_lock(
        recovered_submap_map_mutex_);
    recovered_submap_map_.emplace(submap_mesh.header.frame_id,
                                  std::make_shared<CliSm>(std::move(submap)));
  } else {
    CliSm new_submap = draftNewSubmap();
    addSubmap(std::move(new_submap), cid, csid);
  }

  recover_tsdf_timer.Stop();
  LOG(INFO) << voxblox::timing::Timing::Print();
}

std::set<CIdCSIdPair> SubmapCollection::getSubmapCsidPairs(CliId cid) {
  std::set<CIdCSIdPair> csid_pairs;
  for (auto submap_ptr : getSubmapPtrs()) {
    csid_pairs.emplace(cid, submap_ptr->getID());
  }

  for (auto submap_kv : recovered_submap_map_) {
    csid_pairs.emplace(utils::resolveSubmapFrame(submap_kv.first));
  }

  return csid_pairs;
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
