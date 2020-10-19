#ifndef COXGRAPH_SERVER_SUBMAP_COLLECTION_H_
#define COXGRAPH_SERVER_SUBMAP_COLLECTION_H_

#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "coxgraph/common.h"

namespace coxgraph {
namespace server {

class SubmapCollection : public voxgraph::VoxgraphSubmapCollection {
 public:
  typedef std::shared_ptr<SubmapCollection> Ptr;

  SubmapCollection(const voxgraph::VoxgraphSubmap::Config& submap_config,
                   int8_t client_number, bool verbose = false)
      : voxgraph::VoxgraphSubmapCollection(submap_config, verbose),
        client_number_(client_number) {}
  ~SubmapCollection() = default;

  Transformation addSubmap(const CliSm::Ptr& submap, const CliId& cid,
                           const CliSmId& cli_sm_id);

  inline std::timed_mutex* getPosesUpdateMutex() {
    return &submap_poses_update_mutex;
  }

  inline std::vector<SerSmId>* getSerSmIdsByCliId(const CliId& cid) {
    CHECK(cli_ser_sm_id_map_.count(cid));
    return &cli_ser_sm_id_map_[cid];
  }

 private:
  typedef std::pair<CliId, CliId> CliIdPair;
  typedef std::pair<CliId, CliSmId> CliIdSmIdPair;
  typedef std::unordered_map<SerSmId, CliIdSmIdPair> SmCliIdMap;
  typedef std::map<CliId, std::vector<SerSmId>> CliSerSmIdMap;

  Transformation mergeToCliMap(const CliSm::Ptr& submap);

  const int8_t client_number_;

  SmCliIdMap sm_cli_id_map_;
  CliSerSmIdMap cli_ser_sm_id_map_;

  std::timed_mutex submap_poses_update_mutex;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_SUBMAP_COLLECTION_H_
