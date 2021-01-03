#ifndef COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
#define COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_

#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/ClientSubmapSrv.h>
#include <coxgraph_msgs/PoseHistorySrv.h>
#include <coxgraph_msgs/SubmapsSrv.h>
#include <coxgraph_msgs/TimeLine.h>
#include <voxgraph/frontend/voxgraph_mapper.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <map>
#include <mutex>
#include <string>

#include "coxgraph/client/map_server.h"
#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {

class CoxgraphClient : public voxgraph::VoxgraphMapper {
 public:
  CoxgraphClient(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : VoxgraphMapper(nh, nh_private) {
    int client_id;
    nh_private.param<int>("client_id", client_id, -1);
    client_id_ = static_cast<CliId>(client_id);
    advertiseClientTopics();
    advertiseClientServices();
    if (client_id_ < 0) {
      LOG(FATAL) << "Invalid Client Id " << client_id_;
    } else {
      LOG(INFO) << "Started Coxgraph Client " << client_id_;
    }
    log_prefix_ = "Client " + std::to_string(client_id_) + ": ";

    map_server_.reset(new MapServer(nh_, nh_private_, client_id_,
                                    submap_config_, frame_names_,
                                    submap_collection_ptr_));
  }

  ~CoxgraphClient() = default;

  inline const CliId& getClientId() const { return client_id_; }

  void advertiseClientTopics();
  void advertiseClientServices();

  bool getClientSubmapCallback(
      coxgraph_msgs::ClientSubmapSrv::Request& request,     // NOLINT
      coxgraph_msgs::ClientSubmapSrv::Response& response);  // NOLINT

  bool getAllClientSubmapsCallback(
      coxgraph_msgs::SubmapsSrv::Request& request,     // NOLINT
      coxgraph_msgs::SubmapsSrv::Response& response);  // NOLINT

  bool getPoseHistory(
      coxgraph_msgs::PoseHistorySrv::Request& request,      // NOLINT
      coxgraph_msgs::PoseHistorySrv::Response& response) {  // NOLINT
    response.pose_history.pose_history =
        submap_collection_ptr_->getPoseHistory();
    boost::filesystem::path p(request.file_path);
    p.append("coxgraph_client_traj_" + std::to_string(client_id_) + ".txt");
    savePoseHistory(p.string());
    return true;
  }

  bool submapCallback(
      const voxblox_msgs::LayerWithTrajectory& submap_msg) override;

 private:
  using VoxgraphMapper = voxgraph::VoxgraphMapper;
  using MapServer = client::MapServer;
  typedef std::map<CliSmId, Transformation> SmIdTfMap;

  void publishTimeLine();
  void publishSubmapPoseTFs() override;

  void savePoseHistory(std::string file_path);

  CliId client_id_;
  std::string log_prefix_;

  ros::Publisher time_line_pub_;
  ros::Publisher submap_mesh_pub_;
  ros::ServiceServer get_client_submap_srv_;
  ros::ServiceServer get_all_client_submaps_srv_;
  ros::ServiceServer get_pose_history_srv_;

  SmIdTfMap ser_sm_id_pose_map_;

  std::timed_mutex submap_proc_mutex_;

  MapServer::Ptr map_server_;
};

}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
