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
    need_fusion_.emplace_back(true);
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
    const coxgraph_msgs::MapFusion& map_fusion_msg, bool future) {
  CHECK_NE(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);

  ClientSubmap::Ptr submap_a, submap_b;
  ClientSubmapId submap_id_a, submap_id_b;
  Transformation T_submap_t_a, T_submap_t_b;
  const ClientId& cid_a = map_fusion_msg.from_client_id;
  const ClientId& cid_b = map_fusion_msg.to_client_id;
  const ros::Time& time_a = map_fusion_msg.from_timestamp;
  const ros::Time& time_b = map_fusion_msg.to_timestamp;

  if (!needFusion(cid_a, time_a, cid_b, time_b)) return;

  ReqState ok_a, ok_b;
  ok_a = client_handlers_[cid_a]->requestSubmapByTime(time_a, &submap_id_a,
                                                      &submap_a, &T_submap_t_a);
  ok_b = client_handlers_[cid_b]->requestSubmapByTime(time_b, &submap_id_b,
                                                      &submap_b, &T_submap_t_b);
  if (future) {
    fuseMap();
  } else {
    LOG_IF(INFO, ok_a == ReqState::FAILED)
        << "Requesting submap from Client " << map_fusion_msg.from_client_id
        << " failed!";
    LOG_IF(INFO, ok_a == ReqState::SUCCESS)
        << "Received submap from Client " << map_fusion_msg.from_client_id
        << " with layer memory "
        << submap_a->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
    LOG_IF(INFO, ok_a == ReqState::FUTURE)
        << "Requested timestamp from Client " << map_fusion_msg.from_client_id
        << " is ahead of client timeline, map fusion msg saved for later";
    LOG_IF(INFO, ok_b == ReqState::FAILED)
        << "Requesting submap from Client " << map_fusion_msg.to_client_id
        << " failed!";
    LOG_IF(INFO, ok_b == ReqState::SUCCESS)
        << "Received submap from Client " << map_fusion_msg.to_client_id
        << " with layer memory "
        << submap_b->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
    LOG_IF(INFO, ok_b == ReqState::FUTURE)
        << "Requested timestamp from Client " << map_fusion_msg.to_client_id
        << " is ahead of client timeline, map fusion msg saved for later";
    if ((ok_a == ReqState::SUCCESS && ok_b == ReqState::FUTURE) ||
        (ok_b == ReqState::SUCCESS && ok_a == ReqState::FUTURE)) {
      addToMFFuture(map_fusion_msg);
    }
    if (ok_a == ReqState::SUCCESS && ok_b == ReqState::SUCCESS) {
      fuseMap();
    } else {
      processMFFuture();
    }
  }
}

void CoxgraphServer::addToMFFuture(
    const coxgraph_msgs::MapFusion& map_fusion_msg) {
  map_fusion_msgs_future_.emplace_back(map_fusion_msg);
  if (map_fusion_msgs_future_.size() > config_.map_fusion_queue_size) {
    // TODO(mikexyl): if simply drop the first one, the remained map fusion msgs
    // have similar timestamp, i.e. all timestamps in msgs may be still ahead of
    // clients. so we drop the last msg, to make sure at least fuse the first
    // messages
    map_fusion_msgs_future_.pop_back();
    LOG(INFO) << "Future map fusion too many, dropping the last msg";
  }
}

void CoxgraphServer::processMFFuture() {
  bool processed_any = false;
  for (auto it = map_fusion_msgs_future_.begin();
       it != map_fusion_msgs_future_.end(); it++) {
    coxgraph_msgs::MapFusion map_fusion_msg = *it;
    const ClientId& cid_a = map_fusion_msg.from_client_id;
    const ClientId& cid_b = map_fusion_msg.to_client_id;
    const ros::Time& time_a = map_fusion_msg.from_timestamp;
    const ros::Time& time_b = map_fusion_msg.to_timestamp;
    if (client_handlers_[cid_a]->isTimeLineUpdated() &&
        client_handlers_[cid_b]->isTimeLineUpdated() &&
        client_handlers_[cid_a]->hasTime(time_a) &&
        client_handlers_[cid_b]->hasTime(time_b) &&
        needFusion(cid_a, time_a, cid_b, time_b)) {
      mapFusionCallback(map_fusion_msg, true);
      processed_any = true;
      client_handlers_[cid_a]->resetTimeLineUpdated();
      client_handlers_[cid_b]->resetTimeLineUpdated();
    }
  }
  // Reset timeline update flag, and clear all future map fusion to avoid
  // unnecessary computation
  if (processed_any) map_fusion_msgs_future_.clear();
}

bool CoxgraphServer::needFusion(const ClientId& client_id_a,
                                const ros::Time& time_a,
                                const ClientId& client_id_b,
                                const ros::Time& time_b) {
  if ((client_id_a != 0 && need_fusion_[client_id_a]) ||
      (client_id_b != 0 && need_fusion_[client_id_b]))
    return true;
  return false;
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
