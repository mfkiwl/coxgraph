#ifndef COXGRAPH_SERVER_IMPL_COXGRAPH_SERVER_IMPL_H_
#define COXGRAPH_SERVER_IMPL_COXGRAPH_SERVER_IMPL_H_

namespace coxgraph {

CoxgraphServer::Config CoxgraphServer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  CoxgraphServer::Config config;
  nh_private.param<int>("client_number", config.client_number,
                        config.client_number);
  LOG_IF(FATAL, !(config.client_number > 0 && config.client_number <= 2))
      << "Invalid client number, must > 0, and only max 2 clients supported "
         "now. Given: "
      << config.client_number;
  return config;
}

}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_IMPL_COXGRAPH_SERVER_IMPL_H_