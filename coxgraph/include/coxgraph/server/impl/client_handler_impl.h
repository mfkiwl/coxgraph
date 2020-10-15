#ifndef COXGRAPH_SERVER_IMPL_CLIENT_HANDLER_IMPL_H_
#define COXGRAPH_SERVER_IMPL_CLIENT_HANDLER_IMPL_H_

#include <coxgraph/utils/msg_converter.h>

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

void ClientHandler::publishTopics() {
  loop_closure_pub_ = nh_.advertise<voxgraph_msgs::LoopClosure>(
      client_node_name_ + "/" + config_.client_loop_closure_topic,
      config_.pub_queue_length, true);
}

void ClientHandler::subscribeToServices() {
  pub_client_submap_client_ = nh_.serviceClient<coxgraph_msgs::ClientSubmap>(
      client_node_name_ + "/publish_client_submap");
}

bool ClientHandler::sendLoopClosureMsg(
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  loop_closure_pub_.publish(loop_closure_msg);
}

bool ClientHandler::requestSubmapByTime(const ros::Time& timestamp,
                                        ClientSubmapId* submap_id,
                                        ClientSubmap::Ptr* submap_ptr,
                                        Transformation* T_submap_t) {
  coxgraph_msgs::ClientSubmap client_submap_msg;
  client_submap_msg.request.timestamp = timestamp;
  if (pub_client_submap_client_.call(client_submap_msg)) {
    *submap_id = client_submap_msg.response.submap_id;
    *submap_ptr =
        utils::cliSubmapFromMsg(submap_config_, client_submap_msg.response);
    tf::transformMsgToKindr<voxblox::FloatingPoint>(
        client_submap_msg.response.transform, T_submap_t);
    return true;
  }
  return false;
}

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_IMPL_CLIENT_HANDLER_IMPL_H_
