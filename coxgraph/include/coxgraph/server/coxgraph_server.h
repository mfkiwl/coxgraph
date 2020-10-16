#ifndef COXGRAPH_SERVER_COXGRAPH_SERVER_H_
#define COXGRAPH_SERVER_COXGRAPH_SERVER_H_

#include <coxgraph_msgs/MapFusion.h>
#include <ros/ros.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/ros_params.h>
#include <voxgraph/frontend/pose_graph_interface/pose_graph_interface.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>
#include <voxgraph/tools/ros_params.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/client_handler.h"

namespace coxgraph {

class CoxgraphServer {
 public:
  struct Config {
    Config()
        : client_number(0),
          map_fusion_topic("map_fusion_in"),
          map_fusion_queue_size(10),
          refusion_interval(ros::Duration(2)),
          fixed_map_client_id(0),
          output_mission_frame("mission") {}
    int32_t client_number;
    std::string map_fusion_topic;
    int32_t map_fusion_queue_size;
    ros::Duration refusion_interval;
    int32_t fixed_map_client_id;
    std::string output_mission_frame;

    friend inline std::ostream& operator<<(std::ostream& s,
                                           const CoxgraphServer::Config& v) {
      s << std::endl
        << "Coxgraph Server using Config:" << std::endl
        << "  Client Number: " << v.client_number << std::endl
        << "  Map Fusion Topic: " << v.map_fusion_topic << std::endl
        << "  Map Fusion Queue Size: " << v.map_fusion_queue_size << std::endl
        << "  Client Map Refusion Interval: " << v.refusion_interval << " s"
        << std::endl
        << "  Map Fixed for Client Id: " << v.fixed_map_client_id << std::endl
        << "  Output Mission Frame: " << v.output_mission_frame << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  CoxgraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : CoxgraphServer(
            nh, nh_private, getConfigFromRosParam(nh_private),
            voxgraph::getVoxgraphSubmapConfigFromRosParams(nh_private),
            voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

  CoxgraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 const Config& config, const CliSmConfig& submap_config,
                 const voxblox::MeshIntegratorConfig& mesh_config)
      : nh_(nh),
        nh_private_(nh_private),
        verbose_(false),
        config_(config),
        submap_config_(submap_config),
        submap_collection_ptr_(
            std::make_shared<SubmapCollection>(submap_config_)),
        pose_graph_interface_(nh_private, submap_collection_ptr_, mesh_config,
                              config.output_mission_frame) {
    nh_private.param<bool>("verbose", verbose_, verbose_);
    LOG(INFO) << "Verbose: " << verbose_;
    LOG(INFO) << config_;

    subscribeTopics();
    initClientHandlers(nh, nh_private);

    LOG(INFO) << client_handlers_[0]->getConfig();
  }
  ~CoxgraphServer() = default;

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

 private:
  using ClientHandler = server::ClientHandler;
  using ReqState = ClientHandler::ReqState;
  using SubmapCollection = voxgraph::VoxgraphSubmapCollection;
  using PoseGraphInterface = voxgraph::PoseGraphInterface;
  typedef std::pair<CliId, CliId> CliIdPair;

  void initClientHandlers(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);

  void subscribeTopics();

  void mapFusionMsgCallback(const coxgraph_msgs::MapFusion& map_fusion_msg);
  void loopClosureCallback(const CliId& client_id,
                           const voxgraph_msgs::LoopClosure& loop_closure_msg);
  void mapFusionCallback(const coxgraph_msgs::MapFusion& map_fusion_msg,
                         bool future = false);
  void addToMFFuture(const coxgraph_msgs::MapFusion& map_fusion_msg);
  void processMFFuture();
  bool needFusion(const CliId& cid_a, const ros::Time& time_a,
                  const CliId& cid_b, const ros::Time& time_b);
  bool resetNeedFusion(const CliId& cid_a, const ros::Time& time_a,
                       const CliId& cid_b, const ros::Time& time_b);

  bool fuseMap(const CliId& cid_a, const CliSmId& submap_id_a,
               const CliSm::Ptr& submap_a, const Transformation& T_submap_t_a,
               const CliId& cid_b, const CliSmId& submap_id_b,
               const CliSm::Ptr& submap_b, const Transformation& T_submap_t_b);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber map_fusion_sub_;

  // Verbosity and debug mode
  bool verbose_;

  const Config config_;

  const CliSmConfig submap_config_;
  SubmapCollection::Ptr submap_collection_ptr_;
  PoseGraphInterface pose_graph_interface_;

  std::vector<ClientHandler::Ptr> client_handlers_;
  std::vector<bool> need_fusion_;
  std::vector<ros::Time> last_fusion_time_;

  std::deque<coxgraph_msgs::MapFusion> map_fusion_msgs_future_;

  constexpr static uint8_t kMaxClientNum = 2;
};

}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_COXGRAPH_SERVER_H_
