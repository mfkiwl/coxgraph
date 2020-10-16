#include "coxgraph/server/coxgraph_server.h"

#include <memory>
#include <string>

#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {

CoxgraphServer::Config CoxgraphServer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  CoxgraphServer::Config config;

  nh_private.param<int>("client_number", config.client_number,
                        config.client_number);
  LOG_IF(FATAL,
         !(config.client_number > 0 && config.client_number <= kMaxClientNum))
      << "Invalid client number, must > 0, and only max 2 clients supported "
         "now. Given: "
      << config.client_number;

  nh_private.param<std::string>("map_fusion_topic", config.map_fusion_topic,
                                config.map_fusion_topic);
  nh_private.param<int>("map_fusion_queue_size", config.map_fusion_queue_size,
                        config.map_fusion_queue_size);
  float refuse_interval;
  nh_private.param<float>("refus_interval", refuse_interval, refuse_interval);
  config.refuse_interval.fromSec(refuse_interval);
  nh_private.param<int>("fixed_map_client_id", config.fixed_map_client_id,
                        config.fixed_map_client_id);
  nh_private.param<std::string>("output_mission_frame",
                                config.output_mission_frame,
                                config.output_mission_frame);
  return config;
}

void CoxgraphServer::initClientHandlers(const ros::NodeHandle& nh,
                                        const ros::NodeHandle& nh_private) {
  CHECK_LT(config_.fixed_map_client_id, kMaxClientNum);
  for (int i = 0; i < config_.client_number; i++) {
    client_handlers_.emplace_back(
        new ClientHandler(nh, nh_private, i, submap_config_));
    force_fuse_.emplace_back(true);
    fused_time_line_.emplace_back(TimeLine());
  }
  force_fuse_[config_.fixed_map_client_id] = false;
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
                        utils::fromMapFusionMsg(map_fusion_msg));
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
    const CliId& client_id,
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  client_handlers_[client_id]->sendLoopClosureMsg(loop_closure_msg);
}

void CoxgraphServer::mapFusionCallback(
    const coxgraph_msgs::MapFusion& map_fusion_msg, bool future) {
  CHECK_NE(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);

  // TODO(mikexyl): make ifs cleaner
  CliSm::Ptr submap_a, submap_b;
  CliSmId submap_id_a, submap_id_b;
  Transformation T_submap_t_a, T_submap_t_b;
  const CliId& cid_a = map_fusion_msg.from_client_id;
  const CliId& cid_b = map_fusion_msg.to_client_id;
  const ros::Time& time_a = map_fusion_msg.from_timestamp;
  const ros::Time& time_b = map_fusion_msg.to_timestamp;

  if (!needRefuse(cid_a, time_a, cid_b, time_b)) return;

  ReqState ok_a, ok_b;
  bool has_time_a = client_handlers_[cid_a]->hasTime(time_b);
  bool has_time_b = client_handlers_[cid_b]->hasTime(time_b);
  if (has_time_a && has_time_b) {
    ok_a = client_handlers_[cid_a]->requestSubmapByTime(
        time_a, submap_collection_ptr_->getNextSubmapID(), &submap_id_a,
        &submap_a, &T_submap_t_a);
    ok_b = client_handlers_[cid_b]->requestSubmapByTime(
        time_b, submap_collection_ptr_->getNextSubmapID() + 1, &submap_id_b,
        &submap_b, &T_submap_t_b);
    CHECK_NE(ok_a, ReqState::FUTURE);
    CHECK_NE(ok_b, ReqState::FUTURE);
    if (verbose_) {
      LOG_IF(INFO, ok_a == ReqState::FAILED)
          << "Requesting submap from Client " << map_fusion_msg.from_client_id
          << " failed!";

      LOG_IF(INFO, ok_a == ReqState::FUTURE)
          << "Requested timestamp from Client " << map_fusion_msg.from_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      LOG_IF(INFO, ok_b == ReqState::FAILED)
          << "Requesting submap from Client " << map_fusion_msg.to_client_id
          << " failed!";
      LOG_IF(INFO, ok_b == ReqState::FUTURE)
          << "Requested timestamp from Client " << map_fusion_msg.to_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
    }
    LOG_IF(INFO, ok_a == ReqState::SUCCESS)
        << "Received submap from Client " << map_fusion_msg.from_client_id
        << " with layer memory "
        << submap_a->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
    LOG_IF(INFO, ok_b == ReqState::SUCCESS)
        << "Received submap from Client " << map_fusion_msg.to_client_id
        << " with layer memory "
        << submap_b->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
  }

  if (future) {
    CHECK_EQ(ok_a, ReqState::SUCCESS);
    CHECK_EQ(ok_b, ReqState::SUCCESS);
    fuseMap(cid_a, submap_id_a, submap_a, T_submap_t_a,  // NOLINT
            cid_b, submap_id_b, submap_b, T_submap_t_b);
  } else {
    if (has_time_a && has_time_b) {
      if ((ok_a == ReqState::SUCCESS && ok_b == ReqState::FUTURE) ||
          (ok_b == ReqState::SUCCESS && ok_a == ReqState::FUTURE)) {
        addToMFFuture(map_fusion_msg);
      }
      if (ok_a == ReqState::SUCCESS && ok_b == ReqState::SUCCESS) {
        fuseMap(cid_a, submap_id_a, submap_a, T_submap_t_a,  // NOLINT
                cid_b, submap_id_b, submap_b, T_submap_t_b);
      }
    } else {
      LOG_IF(INFO, !has_time_a)
          << "Map Fusion timestamp from Client "
          << map_fusion_msg.from_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      LOG_IF(INFO, !has_time_b)
          << "Map Fusion timestamp from Client " << map_fusion_msg.to_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      addToMFFuture(map_fusion_msg);
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
    if (verbose_) LOG(INFO) << "Processing saved future MF msgs";
    coxgraph_msgs::MapFusion map_fusion_msg = *it;
    const CliId& cid_a = map_fusion_msg.from_client_id;
    const CliId& cid_b = map_fusion_msg.to_client_id;
    const ros::Time& time_a = map_fusion_msg.from_timestamp;
    const ros::Time& time_b = map_fusion_msg.to_timestamp;
    if (client_handlers_[cid_a]->hasTime(time_a) &&
        client_handlers_[cid_b]->hasTime(time_b) &&
        needRefuse(cid_a, time_a, cid_b, time_b)) {
      if (verbose_)
        LOG(INFO) << "  successfully processed a MF msg, clearing MF msg queue";
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

bool CoxgraphServer::needRefuse(const CliId& cid_a, const ros::Time& time_a,
                                const CliId& cid_b, const ros::Time& time_b) {
  CHECK_EQ(force_fuse_[config_.fixed_map_client_id], false);
  // TODO(mikexyl): update need fusion flag based on time since last fusion
  if ((cid_a != config_.fixed_map_client_id &&
       (isTimeNeedRefuse(cid_a, time_a) || force_fuse_[cid_a])) ||
      (cid_b != config_.fixed_map_client_id &&
       (isTimeNeedRefuse(cid_b, time_b) || force_fuse_[cid_b])))
    return true;
  return false;
}

bool CoxgraphServer::resetNeedRefuse(const CliId& cid_a,
                                     const ros::Time& time_a,
                                     const CliId& cid_b,
                                     const ros::Time& time_b) {
  CHECK_EQ(force_fuse_[config_.fixed_map_client_id], false);
  force_fuse_[cid_a] = false;
  force_fuse_[cid_b] = false;

  return true;
}

bool CoxgraphServer::fuseMap(const CliId& cid_a, const CliSmId& submap_id_a,
                             const CliSm::Ptr& submap_a,
                             const Transformation& T_submap_t_a,
                             const CliId& cid_b, const CliSmId& submap_id_b,
                             const CliSm::Ptr& submap_b,
                             const Transformation& T_submap_t_b) {
  LOG(INFO) << "Fusing: " << std::endl
            << "  Client: " << static_cast<int>(cid_a)
            << " -> Submap: " << static_cast<int>(submap_id_a) << std::endl
            << "  Client: " << static_cast<int>(cid_b)
            << " -> Submap: " << static_cast<int>(submap_id_b);
  LOG_IF(INFO, verbose_) << " T_submap_t_a: " << std::endl << T_submap_t_a;
  LOG_IF(INFO, verbose_) << " T_submap_t_b: " << std::endl << T_submap_t_b;

  submap_collection_ptr_->addSubmap(submap_a);
  submap_collection_ptr_->addSubmap(submap_b);
}

}  // namespace coxgraph
