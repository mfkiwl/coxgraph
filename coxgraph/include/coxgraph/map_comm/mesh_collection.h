#ifndef COXGRAPH_MAP_COMM_MESH_COLLECTION_H_
#define COXGRAPH_MAP_COMM_MESH_COLLECTION_H_

#include <coxgraph/common.h>
#include <coxgraph_msgs/MeshWithTrajectory.h>

#include <map>
#include <memory>
#include <mutex>
#include <utility>

namespace coxgraph {
namespace comm {

class MeshCollection {
 public:
  typedef std::shared_ptr<MeshCollection> Ptr;

  using CSIdMeshMap = std::map<CIdCSIdPair, coxgraph_msgs::MeshWithTrajectory>;
  using CSIdMeshMapPtr = std::shared_ptr<CSIdMeshMap>;

  MeshCollection() : csid_mesh_map_ptr_(new CSIdMeshMap()) {}
  ~MeshCollection() = default;

  void addSubmapMesh(const coxgraph_msgs::MeshWithTrajectory& mesh_with_traj,
                     CliId cid, CliSmId csid) {
    std::lock_guard<std::mutex> mesh_lock(mesh_mutex_);
    if (!csid_mesh_map_ptr_->count(std::make_pair(cid, csid)))
      csid_mesh_map_ptr_->emplace(std::make_pair(cid, csid), mesh_with_traj);
    else
      (*csid_mesh_map_ptr_)[std::make_pair(cid, csid)] = mesh_with_traj;
  }

  CSIdMeshMapPtr getSubmapMeshesPtr() { return csid_mesh_map_ptr_; }

 private:
  CSIdMeshMapPtr csid_mesh_map_ptr_;

  std::mutex mesh_mutex_;
};

}  // namespace comm
}  // namespace coxgraph

#endif  // COXGRAPH_MAP_COMM_MESH_COLLECTION_H_
