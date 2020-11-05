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
  global_pg_interface.updateSubmapRPConstraints();

  auto opt_async = std::async(std::launch::async, &PoseGraphInterface::optimize,
                              &global_pg_interface, true);

  while (opt_async.wait_for(std::chrono::milliseconds(100)) !=
         std::future_status::ready) {
    LOG_EVERY_N(INFO, 10) << "Global optimzation is still running...";
  }
  LOG(INFO) << "Optimization finished, generating global mesh...";

  global_pg_interface.updateSubmapCollectionPoses();

  // TODO(mikexyl): also update client map tf using global opt

  submap_vis_.saveAndPubCombinedMesh(*global_submap_collection_ptr,
                                     mission_frame, publisher, file_path);

  LOG(INFO) << "Global mesh generated, published and saved to " << file_path;
}
}  // namespace server
}  // namespace coxgraph
