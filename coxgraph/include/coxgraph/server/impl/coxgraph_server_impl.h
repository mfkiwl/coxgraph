#ifndef COXGRAPH_SERVER_IMPL_COXGRAPH_SERVER_IMPL_H_
#define COXGRAPH_SERVER_IMPL_COXGRAPH_SERVER_IMPL_H_

#include <memory>
#include <string>

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

  nh_private.param<std::string>("map_fusion_topic", config.map_fusion_topic,
                                config.map_fusion_topic);
  nh_private.param<int>("map_fusion_queue_size", config.map_fusion_queue_size,
                        config.map_fusion_queue_size);
  return config;
}

void CoxgraphServer::initClientHandlers(const ros::NodeHandle& nh,
                                        const ros::NodeHandle& nh_private) {
  for (int i = 0; i < config_.client_number; i++) {
    client_handlers_.emplace_back(
        new ClientHandler(nh, nh_private, i, submap_config_));
  }
}

void CoxgraphServer::subscribeTopics() {
  map_fusion_sub_ =
      nh_.subscribe(config_.map_fusion_topic, config_.map_fusion_queue_size,
                    &CoxgraphServer::mapFusionMsgCallback, this);
}

void CoxgraphServer::mapFusionMsgCallback(
    const coxgraph_msgs::MapFusion& map_fusion_msg) {
  if (map_fusion_msg.from_client_id == map_fusion_msg.to_client_id) {
    LOG(INFO) << "Received loop closure msg in client "
              << map_fusion_msg.from_client_id << " from "
              << map_fusion_msg.from_timestamp << " to "
              << map_fusion_msg.to_timestamp;
    loopClosureCallback(map_fusion_msg.from_client_id,
                        fromMapFusionMsg(map_fusion_msg));
  } else {
    LOG(INFO) << "Received map fusion msg from client "
              << map_fusion_msg.from_client_id << " at "
              << map_fusion_msg.from_timestamp << " to client "
              << map_fusion_msg.to_client_id << " at "
              << map_fusion_msg.to_timestamp;
    mapFusionCallback(map_fusion_msg);
  }
}

void CoxgraphServer::loopClosureCallback(
    const ClientId& client_id,
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  client_handlers_[client_id]->sendLoopClosureMsg(loop_closure_msg);
}

void CoxgraphServer::mapFusionCallback(
    const coxgraph_msgs::MapFusion& map_fusion_msg) {
  CHECK_NE(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);

  ClientSubmap::Ptr submap_a, submap_b;
  ClientSubmapId submap_id_a, submap_id_b;
  Transformation T_submap_t_a, T_submap_t_b;
  bool ok_a, ok_b;
  ok_a = client_handlers_[map_fusion_msg.from_client_id]->requestSubmapByTime(
      map_fusion_msg.from_timestamp, &submap_id_a, &submap_a, &T_submap_t_a);
  ok_b = client_handlers_[map_fusion_msg.to_client_id]->requestSubmapByTime(
      map_fusion_msg.to_timestamp, &submap_id_b, &submap_b, &T_submap_t_b);
}

voxgraph_msgs::LoopClosure CoxgraphServer::fromMapFusionMsg(
    const coxgraph_msgs::MapFusion& map_fusion_msg) {
  CHECK_EQ(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);
  voxgraph_msgs::LoopClosure loop_closure_msg;
  loop_closure_msg.from_timestamp = map_fusion_msg.from_timestamp;
  loop_closure_msg.to_timestamp = map_fusion_msg.to_timestamp;
  loop_closure_msg.transform = map_fusion_msg.transform;
  return loop_closure_msg;
}

}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_IMPL_COXGRAPH_SERVER_IMPL_H_
