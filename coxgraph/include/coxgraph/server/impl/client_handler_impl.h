#ifndef COXGRAPH_SERVER_IMPL_CLIENT_HANDLER_IMPL_H_
#define COXGRAPH_SERVER_IMPL_CLIENT_HANDLER_IMPL_H_

#include <string>

namespace coxgraph {
namespace server {

ClientHandler::Config ClientHandler::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  ClientHandler::Config config;
  nh_private.param<std::string>("client_name_prefix", config.client_name_prefix,
                                config.client_name_prefix);
  nh_private.param<std::string>("client_loop_closure_topic",
                                config.client_loop_closure_topic,
                                config.client_loop_closure_topic);
  nh_private.param<int>("ch_pub_queue_length", config.pub_queue_length,
                        config.pub_queue_length);
  return config;
}

bool ClientHandler::sendLoopClosureMsg(
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  loop_closure_pub_.publish(loop_closure_msg);
}

bool ClientHandler::requestSubmapByTime(const ros::Time& timestamp,
                                        ClientSubmapId* submap_id,
                                        const ClientSubmap::Ptr& submap_ptr) {}

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_IMPL_CLIENT_HANDLER_IMPL_H_
