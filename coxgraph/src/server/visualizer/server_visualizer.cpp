#include "coxgraph/server/visualizer/server_visualizer.h"

#include <chrono>
#include <future>
#include <string>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/submap_collection.h"
#include "ros/topic.h"

namespace coxgraph {
namespace server {

// TODO(mikexyl): check origin params not changed
void ServerVisualizer::getFinalGlobalMesh(
    const SubmapCollection::Ptr& submap_collection_ptr,
    const PoseGraphInterface& pose_graph_interface,
    const std::vector<CliSmIdPack>& other_submaps,
    const std::string& mission_frame, const ros::Publisher& publisher,
    const std::string& file_path) {
  LOG(INFO) << "Generating final mesh";

  SubmapCollection::Ptr global_submap_collection_ptr(
      new SubmapCollection(*submap_collection_ptr));
  PoseGraphInterface global_pg_interface(pose_graph_interface,
                                         global_submap_collection_ptr);
  for (auto const& submap_pack : other_submaps) {
    global_submap_collection_ptr->addSubmap(
        submap_pack.submap_ptr, submap_pack.cid, submap_pack.cli_sm_id);
    global_pg_interface.addSubmap(submap_pack.submap_ptr->getID());
  }

  TransformationVector submap_poses;
  global_submap_collection_ptr->getSubmapPoses(&submap_poses);
  LOG(INFO) << "before optimize";
  for (int i = 0; i < submap_poses.size(); i++) {
    LOG(INFO) << i << std::endl << submap_poses[i];
  }

  global_pg_interface.updateSubmapRPConstraints();

  auto opt_async = std::async(std::launch::async, &PoseGraphInterface::optimize,
                              &global_pg_interface, true);

  while (opt_async.wait_for(std::chrono::milliseconds(100)) !=
         std::future_status::ready) {
    LOG_EVERY_N(INFO, 10) << "Global optimzation is still running...";
  }
  LOG(INFO) << "Optimization finished, generating global mesh...";

  auto pose_map = global_pg_interface.getPoseMap();
  LOG(INFO) << "pose graph results";
  for (auto const& pose_kv : pose_map)
    LOG(INFO) << pose_kv.first << std::endl << pose_kv.second;

  LOG(INFO) << "Evaluating Residuals of Map Fusion Constraints";
  global_pg_interface.printResiduals(
      PoseGraphInterface::ConstraintType::RelPose);

  global_pg_interface.updateSubmapCollectionPoses();

  global_submap_collection_ptr->getSubmapPoses(&submap_poses);
  LOG(INFO) << "after optimize";
  for (int i = 0; i < submap_poses.size(); i++) {
    LOG(INFO) << i << std::endl << submap_poses[i];
  }

  // TODO(mikexyl): also update client map tf using global opt

  submap_vis_.saveAndPubCombinedMesh(*global_submap_collection_ptr,
                                     mission_frame, publisher, file_path);

  LOG(INFO) << "Global mesh generated, published and saved to " << file_path;
}

}  // namespace server
}  // namespace coxgraph
