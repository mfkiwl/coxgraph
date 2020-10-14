#ifndef COXGRAPH_SERVER_COXGRAPH_SERVER_H_
#define COXGRAPH_SERVER_COXGRAPH_SERVER_H_

#include <coxgraph_msgs/MapFusion.h>
#include <ros/ros.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <string>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/client_handler.h"

namespace coxgraph {

class CoxgraphServer {
 public:
  using ClientHandler = server::ClientHandler;

  struct Config {
    Config()
        : client_number(0),
          map_fusion_topic("loop_closure_in"),
          map_fusion_queue_size(10) {}
    int32_t client_number;
    std::string map_fusion_topic;
    int32_t map_fusion_queue_size;

    friend inline std::ostream& operator<<(std::ostream& s,
                                           const CoxgraphServer::Config& v) {
      s << std::endl
        << "Coxgraph Server using Config:" << std::endl
        << "  Client Number: " << v.client_number << std::endl
        << "  Loop Closure Topic: " << v.map_fusion_topic << std::endl
        << "  Loop Closure Queue Size: " << v.map_fusion_queue_size << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  CoxgraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        verbose_(false),
        config_(getConfigFromRosParam(nh_private)),
        client_handlers_(
            std::vector<ClientHandler::Ptr>(config_.client_number, nullptr)) {
    nh_private.param<bool>("verbose", verbose_, verbose_);
    LOG(INFO) << "Verbose: " << verbose_;
    LOG(INFO) << config_;

    initClientHandlers(nh, nh_private);
    LOG(INFO) << client_handlers_[0]->getConfig();
  }
  ~CoxgraphServer() = default;

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

 private:
  void initClientHandlers(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);

  void subscribeTopics();

  void mapFusionMsgCallback(const coxgraph_msgs::MapFusion& map_fusion_msg);
  void loopClosureCallback(const ClientId& client_id,
                           const voxgraph_msgs::LoopClosure& loop_closure_msg);
  void mapFusionCallback(const coxgraph_msgs::MapFusion& map_fusion_msg);

  static voxgraph_msgs::LoopClosure fromMapFusionMsg(
      const coxgraph_msgs::MapFusion& map_fusion_msg);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber map_fusion_sub_;

  // Verbosity and debug mode
  bool verbose_;

  Config config_;

  std::vector<ClientHandler::Ptr> client_handlers_;
};

}  // namespace coxgraph

#include "coxgraph/server/impl/coxgraph_server_impl.h"

#endif  // COXGRAPH_SERVER_COXGRAPH_SERVER_H_
