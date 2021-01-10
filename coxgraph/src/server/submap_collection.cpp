#include "coxgraph/server/submap_collection.h"

#include <voxblox/integrator/merge_integration.h>

#include <utility>
#include <vector>

#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
namespace server {

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

void SubmapCollection::addSubmap(
    const coxgraph_msgs::MeshWithTrajectory& submap_mesh, const CliId& cid,
    const CliSmId& csid) {
  // If a submap is already generated from tsdf, don't add it
  SerSmId ssid;
  if (getSerSmIdByCliSmId(cid, csid, &ssid)) return;

  mesh_collection_ptr_->addSubmapMesh(submap_mesh, cid, csid);

  voxblox::timing::Timer wait_for_tf_timer("wait_for_tf");
  CIdCSIdPair csid_pair =
      utils::resolveSubmapFrame(submap_mesh.mesh.header.frame_id);

  CliSm submap = draftNewSubmap();
  tsdf_integrator_->setLayer(submap.getTsdfMapPtr()->getTsdfLayerPtr());

  Transformation T_Sm_C;
  voxblox::Pointcloud points_C;

  while (mesh_converter_ptr_->getPointcloudInNextFOV(&T_Sm_C, &points_C)) {
    if (points_C.empty()) continue;

    // Only for navigation, no need color
    voxblox::Colors no_colors(points_C.size(), voxblox::Color());
    tsdf_integrator_->integratePointCloud(T_Sm_C, points_C, no_colors, false);
  }

  recovered_submap_map_.emplace(submap_mesh.mesh.header.frame_id,
                                std::move(submap));
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

}  // namespace server
}  // namespace coxgraph
