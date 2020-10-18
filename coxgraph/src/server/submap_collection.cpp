#include "coxgraph/server/submap_collection.h"

#include <voxblox/integrator/merge_integration.h>

namespace coxgraph {
namespace server {

Transformation CliMapCollection::addSubmap(const CliSm::Ptr& submap) {
  CHECK_LE(size(), client_number_);
  if (exists(submap->getID())) {
    return mergeToCliMap(submap);
  } else {
    voxgraph::VoxgraphSubmapCollection::addSubmap(submap);
    return Transformation();
  }
}

Transformation CliMapCollection::mergeToCliMap(const CliSm::Ptr& submap) {
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
