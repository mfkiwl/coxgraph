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

  // Copy constructor without copy mutex
  SubmapCollection(const SubmapCollection& rhs)
      : voxgraph::VoxgraphSubmapCollection(
            static_cast<voxgraph::VoxgraphSubmapCollection>(rhs)),
        client_number_(rhs.client_number_),
        sm_cli_id_map_(rhs.sm_cli_id_map_),
        cli_ser_sm_id_map_(rhs.cli_ser_sm_id_map_),
        sm_id_ori_pose_map_(rhs.sm_id_ori_pose_map_) {}

  ~SubmapCollection() = default;

  const int8_t& getClientNumber() const { return client_number_; }

  Transformation addSubmap(const CliSm::Ptr& submap_ptr, const CliId& cid,
                           const CliSmId& cli_sm_id);

  inline bool getSerSmIdsByCliId(const CliId& cid,
                                 std::vector<SerSmId>* ser_sids) {
    if (cli_ser_sm_id_map_.count(cid)) {
      *ser_sids = cli_ser_sm_id_map_[cid];
      return true;
    } else {
      return false;
    }
  }

  inline bool getSerSmIdByCliSmId(const CliId& cid, const CliSmId& cli_sm_id,
                                  SerSmId* ser_sm_id) {
    CHECK(ser_sm_id != nullptr);
    for (auto ser_sm_id_v : cli_ser_sm_id_map_[cid]) {
      if (sm_cli_id_map_[ser_sm_id_v].second == cli_sm_id) {
        *ser_sm_id = ser_sm_id_v;
        return true;
      }
    }
    return false;
  }

  inline bool getCliSmIdsByCliId(const CliId& cid,
                                 std::vector<CliSmId>* cli_sids) {
    CHECK(cli_sids != nullptr);
    cli_sids->clear();
    for (auto ser_sm_id_v : cli_ser_sm_id_map_[cid]) {
      cli_sids->emplace_back(sm_cli_id_map_[ser_sm_id_v].second);
    }
    return !cli_sids->empty();
  }

  inline void updateOriPose(const SerSmId& ser_sm_id,
                            const Transformation& pose) {
    CHECK(exists(ser_sm_id));
    sm_id_ori_pose_map_[ser_sm_id] = pose;
  }
  inline Transformation getOriPose(const SerSmId& ser_sm_id) {
    CHECK(sm_id_ori_pose_map_.count(ser_sm_id));
    return sm_id_ori_pose_map_[ser_sm_id];
  }

  inline std::timed_mutex* getPosesUpdateMutex() {
    return &submap_poses_update_mutex;
  }

 private:
  typedef std::pair<CliId, CliId> CliIdPair;
  typedef std::unordered_map<SerSmId, CIdCSIdPair> SmCliIdMap;
  typedef std::unordered_map<CliId, std::vector<SerSmId>> CliSerSmIdMap;

  Transformation mergeToCliMap(const CliSm::Ptr& submap_ptr);

  const int8_t client_number_;

  SmCliIdMap sm_cli_id_map_;
  CliSerSmIdMap cli_ser_sm_id_map_;

  std::unordered_map<SerSmId, Transformation> sm_id_ori_pose_map_;

  std::timed_mutex submap_poses_update_mutex;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_SUBMAP_COLLECTION_H_
