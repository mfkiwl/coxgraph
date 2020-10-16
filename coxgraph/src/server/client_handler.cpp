#include "coxgraph/server/client_handler.h"

#include <coxgraph_msgs/TimeLine.h>

#include <string>

#include "coxgraph/utils/msg_converter.h"

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

void ClientHandler::subscribeToTopics() {
  time_line_sub_ = nh_.subscribe(client_node_name_ + "/time_line", 1,
                                 &ClientHandler::timeLineCallback, this);
}

void ClientHandler::timeLineCallback(
    const coxgraph_msgs::TimeLine& time_line_msg) {
  if (time_line_.start != time_line_msg.start ||
      time_line_.end != time_line_msg.end) {
    time_line_.start = time_line_msg.start;
    time_line_.end = time_line_msg.end;
    time_line_updated_ = true;
  }
  // TODO(mikexyl): make CH id a log prefix string
  LOG(INFO) << "CH " << static_cast<int>(client_id_)
            << ": Received Updated client time Line from "
            << time_line_msg.start << " to " << time_line_msg.end << std::endl;
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

ClientHandler::ReqState ClientHandler::requestSubmapByTime(
    const ros::Time& timestamp, const SerSmId& ser_sm_id, CliSmId* cli_sm_id,
    CliSm::Ptr* submap, Transformation* T_submap_t) {
  if (!time_line_.hasTime(timestamp)) return ReqState::FUTURE;

  coxgraph_msgs::ClientSubmap client_submap_msg;
  client_submap_msg.request.timestamp = timestamp;
  if (pub_client_submap_client_.call(client_submap_msg)) {
    *cli_sm_id = client_submap_msg.response.submap_id;
    *submap = utils::cliSubmapFromMsg(ser_sm_id, submap_config_,
                                      client_submap_msg.response);
    tf::transformMsgToKindr<voxblox::FloatingPoint>(
        client_submap_msg.response.transform, T_submap_t);
    return ReqState::SUCCESS;
  }
  return ReqState::FAILED;
}

}  // namespace server
}  // namespace coxgraph
