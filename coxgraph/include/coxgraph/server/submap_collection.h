#ifndef COXGRAPH_SERVER_SUBMAP_COLLECTION_H_
#define COXGRAPH_SERVER_SUBMAP_COLLECTION_H_

#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <memory>
#include <unordered_map>
#include <utility>

#include "coxgraph/common.h"

namespace coxgraph {
namespace server {

class CliMapCollection : public voxgraph::VoxgraphSubmapCollection {
 public:
  typedef std::shared_ptr<CliMapCollection> Ptr;

  CliMapCollection(const voxgraph::VoxgraphSubmap::Config& submap_config,
                   int8_t client_number, bool verbose = false)
      : voxgraph::VoxgraphSubmapCollection(submap_config, verbose),
        client_number_(client_number) {}
  ~CliMapCollection() = default;

  Transformation addSubmap(const CliSm::Ptr& submap);

 private:
  typedef std::pair<CliId, CliId> CliIdPair;
  typedef std::pair<CliId, CliSmId> CliIdSmIdPair;
  typedef std::unordered_map<SerSmId, CliIdSmIdPair> SmCliIdMap;

  Transformation mergeToCliMap(const CliSm::Ptr& submap);

  const int8_t client_number_;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_SUBMAP_COLLECTION_H_
