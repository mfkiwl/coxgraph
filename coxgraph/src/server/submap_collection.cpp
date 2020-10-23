#include "coxgraph/server/submap_collection.h"

#include <voxblox/integrator/merge_integration.h>

#include <vector>

namespace coxgraph {
namespace server {

Transformation SubmapCollection::addSubmap(const CliSm::Ptr& submap,
                                           const CliId& cid,
                                           const CliSmId& cli_sm_id) {
  LOG(INFO) << "debug: adding submap (sid, cid, csmid) " << submap->getID()
            << " " << static_cast<int>(cid) << " " << cli_sm_id;
  voxgraph::VoxgraphSubmapCollection::addSubmap(submap);
  sm_cli_id_map_.emplace(submap->getID(), CliIdSmIdPair(cid, cli_sm_id));
  if (!cli_ser_sm_id_map_.count(submap->getID())) {
    cli_ser_sm_id_map_.emplace(cid, std::vector<SerSmId>());
  }
  cli_ser_sm_id_map_[cid].emplace_back(submap->getID());
  sm_id_ori_pose_map_.emplace(submap->getID(), submap->getPose());
  return Transformation();
}

Transformation SubmapCollection::mergeToCliMap(const CliSm::Ptr& submap) {
  CHECK(exists(submap->getID()));

  auto const& cli_map_ptr = getSubmapPtr(submap->getID());
  // TODO(mikexyl): only merge layers now, merge more if needed, and
  // theoretically not need to transform layer since it's already done when
  // generating submap from msg
  voxblox::mergeLayerAintoLayerB(
      submap->getTsdfMapPtr()->getTsdfLayer(),
      cli_map_ptr->getTsdfMapPtr()->getTsdfLayerPtr());

  cli_map_ptr->finishSubmap();
  return submap->getPose() * cli_map_ptr->getPose().inverse();
}

}  // namespace server
}  // namespace coxgraph
