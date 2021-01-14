#ifndef COXGRAPH_MAP_COMM_SUBMAP_COLLECTION_H_
#define COXGRAPH_MAP_COMM_SUBMAP_COLLECTION_H_

#include <coxgraph_msgs/MeshWithTrajectory.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/ros_params.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/map_comm/mesh_collection.h"
#include "coxgraph/map_comm/mesh_converter.h"
#include "coxgraph/utils/submap_info_listener.h"

namespace coxgraph {
namespace comm {

class SubmapCollection : public voxgraph::VoxgraphSubmapCollection {
 public:
  typedef std::shared_ptr<SubmapCollection> Ptr;

  SubmapCollection(const voxgraph::VoxgraphSubmap::Config& submap_config,
                   int8_t client_number,
                   MeshCollection::Ptr mesh_collection_ptr,
                   const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                   bool verbose = false)
      : voxgraph::VoxgraphSubmapCollection(submap_config, verbose),
        nh_private_(nh_private),
        client_number_(client_number),
        tsdf_integrator_config_(
            voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)),
        mesh_collection_ptr_(mesh_collection_ptr),
        submap_info_listener_(nh, nh_private) {
    nh_private_.param<std::string>("method", tsdf_integrator_method_,
                                   "projective");
  }

  SubmapCollection(const voxgraph::VoxgraphSubmapCollection& submap_collection,
                   int8_t client_number,
                   MeshCollection::Ptr mesh_collection_ptr,
                   const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : voxgraph::VoxgraphSubmapCollection(submap_collection),
        nh_private_(nh_private),
        client_number_(client_number),
        tsdf_integrator_config_(
            voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)),
        mesh_collection_ptr_(mesh_collection_ptr),
        submap_info_listener_(nh, nh_private) {
    nh_private_.param<std::string>("method", tsdf_integrator_method_,
                                   "projective");
  }

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

  void addSubmap(const CliSm::Ptr& submap_ptr, const CliId& cid,
                 const CliSmId& csid);

  void addSubmapFromMesh(const coxgraph_msgs::MeshWithTrajectory& submap_mesh,
                         const CliId& cid, const CliSmId& csid);

  void addSubmapFromMeshAsync(
      const coxgraph_msgs::MeshWithTrajectory& submap_mesh, const CliId& cid,
      const CliSmId& csid);

  inline bool getSerSmIdsByCliId(const CliId& cid,
                                 std::vector<SerSmId>* ser_sids) {
    std::lock_guard<std::mutex> id_update_lock(id_update_mutex_);
    if (cli_ser_sm_id_map_.count(cid)) {
      *ser_sids = cli_ser_sm_id_map_[cid];
      return true;
    } else {
      return false;
    }
  }

  inline bool getSerSmIdByCliSmId(const CliId& cid, const CliSmId& csid,
                                  SerSmId* ssid) {
    CHECK(ssid != nullptr);
    std::lock_guard<std::mutex> id_update_lock(id_update_mutex_);
    if (!cli_ser_sm_id_map_.count(cid)) return false;
    for (auto ser_sm_id_v : cli_ser_sm_id_map_[cid]) {
      if (sm_cli_id_map_[ser_sm_id_v].second == csid) {
        *ssid = ser_sm_id_v;
        return true;
      }
    }
    return false;
  }

  inline bool getCliSmIdsByCliId(const CliId& cid,
                                 std::vector<CliSmId>* csids) {
    CHECK(csids != nullptr);
    std::lock_guard<std::mutex> id_update_lock(id_update_mutex_);
    csids->clear();
    for (auto ser_sm_id_v : cli_ser_sm_id_map_[cid]) {
      csids->emplace_back(sm_cli_id_map_[ser_sm_id_v].second);
    }
    return !csids->empty();
  }

  inline void updateOriPose(const SerSmId& ssid, const Transformation& pose) {
    CHECK(exists(ssid));
    sm_id_ori_pose_map_[ssid] = pose;
  }
  inline Transformation getOriPose(const SerSmId& ssid) {
    CHECK(sm_id_ori_pose_map_.count(ssid));
    return sm_id_ori_pose_map_[ssid];
  }

  inline std::timed_mutex* getPosesUpdateMutex() {
    return &submap_poses_update_mutex;
  }

  inline size_t getNumberOfRecoveredSubmaps() const {
    return recovered_submap_map_.size();
  }

  voxblox::TsdfMap::Ptr getProjectedMap();

  /**
   * @brief Get the Submap Csid Pairs object, only for client mode
   *
   * @param cid client id of this client
   * @return std::set<CIdCSIdPair>
   */
  std::set<CIdCSIdPair> getSubmapCsidPairs(CliId cid);

 private:
  typedef std::pair<CliId, CliId> CliIdPair;
  typedef std::unordered_map<SerSmId, CIdCSIdPair> SmCliIdMap;
  typedef std::map<CliId, std::vector<SerSmId>> CliSerSmIdMap;

  Transformation mergeToCliMap(const CliSm::Ptr& submap_ptr);

  ros::NodeHandle nh_private_;

  const int8_t client_number_;

  SmCliIdMap sm_cli_id_map_;
  CliSerSmIdMap cli_ser_sm_id_map_;

  std::map<CliId, std::thread> recover_threads_;
  std::string tsdf_integrator_method_;
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  MeshCollection::Ptr mesh_collection_ptr_;
  std::map<std::string, CliSm::Ptr> recovered_submap_map_;
  std::mutex recovered_submap_map_mutex_;

  std::unordered_map<SerSmId, Transformation> sm_id_ori_pose_map_;

  utils::SubmapInfoListener submap_info_listener_;

  std::mutex id_update_mutex_;
  std::timed_mutex submap_poses_update_mutex;
};

}  // namespace comm
}  // namespace coxgraph

#endif  // COXGRAPH_MAP_COMM_SUBMAP_COLLECTION_H_
