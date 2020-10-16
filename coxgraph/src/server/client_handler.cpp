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
  nh_private.param<std::string>("client_loop_closure_topic_suffix",
                                config.client_loop_closure_topic_suffix,
                                config.client_loop_closure_topic_suffix);
  nh_private.param<std::string>("client_map_pose_update_topic_suffix",
                                config.client_map_pose_update_topic_suffix,
                                config.client_map_pose_update_topic_suffix);
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
  updateTimeLine(time_line_msg.start, time_line_msg.end);
}

void ClientHandler::advertiseTopics() {
  loop_closure_pub_ = nh_.advertise<voxgraph_msgs::LoopClosure>(
      client_node_name_ + "/" + config_.client_loop_closure_topic_suffix,
      config_.pub_queue_length, true);
  sm_pose_update_pub_ = nh_.advertise<coxgraph_msgs::MapPoseUpdate>(
      client_node_name_ + "/" + config_.client_map_pose_update_topic_suffix,
      config_.pub_queue_length, true);
}

void ClientHandler::subscribeToServices() {
  pub_client_submap_client_ = nh_.serviceClient<coxgraph_msgs::ClientSubmapSrv>(
      client_node_name_ + "/publish_client_submap");
}

ClientHandler::ReqState ClientHandler::requestSubmapByTime(
    const ros::Time& timestamp, const SerSmId& ser_sm_id, CliSmId* cli_sm_id,
    CliSm::Ptr* submap, Transformation* T_submap_t) {
  if (!time_line_.hasTime(timestamp)) return ReqState::FUTURE;

  coxgraph_msgs::ClientSubmapSrv cli_submap_srv;
  cli_submap_srv.request.timestamp = timestamp;
  if (pub_client_submap_client_.call(cli_submap_srv)) {
    *cli_sm_id = cli_submap_srv.response.submap.map_header.id;
    *submap = utils::cliSubmapFromMsg(
        ser_sm_id, submap_config_, cli_submap_srv.response, &mission_frame_id_);
    tf::transformMsgToKindr<voxblox::FloatingPoint>(
        cli_submap_srv.response.transform, T_submap_t);
    return ReqState::SUCCESS;
  }
  return ReqState::FAILED;
}

}  // namespace server
}  // namespace coxgraph
