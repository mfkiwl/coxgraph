#ifndef COXGRAPH_SERVER_COXGRAPH_SERVER_H_
#define COXGRAPH_SERVER_COXGRAPH_SERVER_H_

#include <ros/ros.h>

#include "coxgraph/common.h"
#include "coxgraph/server/client_handler.h"

namespace coxgraph {

class CoxgraphServer {
 public:
  using ClientHandler = server::ClientHandler;

  struct Config {
    Config() : client_number(0) {}
    int32_t client_number;

    friend inline std::ostream& operator<<(std::ostream& s,
                                           const CoxgraphServer::Config& v) {
      s << std::endl
        << "Coxgraph Server using Config:" << std::endl
        << "  Client Number: " << v.client_number << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  CoxgraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : verbose_(false), config_(getConfigFromRosParam(nh_private)) {
    nh_private.param<bool>("verbose", verbose_, verbose_);
    LOG(INFO) << "Verbose: " << verbose_;
    LOG(INFO) << config_;
  }
  ~CoxgraphServer() = default;

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Verbosity and debug mode
  bool verbose_;

  Config config_;
};

}  // namespace coxgraph

#include "coxgraph/server/impl/coxgraph_server_impl.h"

#endif  // COXGRAPH_SERVER_COXGRAPH_SERVER_H_
