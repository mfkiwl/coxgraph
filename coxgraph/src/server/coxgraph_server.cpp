#include "coxgraph/server/coxgraph_server.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

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
  nh_private.param<float>("refuse_interval", refuse_interval, refuse_interval);
  config.refuse_interval.fromSec(refuse_interval);
  nh_private.param<int>("fixed_map_client_id", config.fixed_map_client_id,
                        config.fixed_map_client_id);
  nh_private.param<std::string>("output_mission_frame",
                                config.output_mission_frame,
                                config.output_mission_frame);
  nh_private.param<bool>("submap_registration/enabled",
                         config.enable_registration_constraints,
                         config.enable_registration_constraints);
  nh_private.param<bool>("submap_relative_pose/enabled",
                         config.enable_submap_relative_pose_constraints,
                         config.enable_submap_relative_pose_constraints);
  nh_private.param<int>("publisher_queue_length", config.publisher_queue_length,
                        config.publisher_queue_length);
  return config;
}

void CoxgraphServer::initClientHandlers(const ros::NodeHandle& nh,
                                        const ros::NodeHandle& nh_private) {
  CHECK_LT(config_.fixed_map_client_id, kMaxClientNum);
  for (int i = 0; i < config_.client_number; i++) {
    client_handlers_.emplace_back(new ClientHandler(
        nh, nh_private, i, submap_config_, submap_collection_ptr_));
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

void CoxgraphServer::advertiseTopics() {
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", config_.publisher_queue_length, true);
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", config_.publisher_queue_length, true);
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
  client_handlers_[client_id]->pubLoopClosureMsg(loop_closure_msg);
}

bool CoxgraphServer::mapFusionCallback(
    const coxgraph_msgs::MapFusion& map_fusion_msg, bool future) {
  CHECK_NE(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);

  // TODO(mikexyl): make ifs cleaner
  CliSm::Ptr submap_a, cli_sm_id_b;
  CliSmId cli_sm_id_a, submap_id_b;
  Transformation T_A_t1, T_B_t2;
  const CliId& cid_a = map_fusion_msg.from_client_id;
  const CliId& cid_b = map_fusion_msg.to_client_id;
  const ros::Time& t1 = map_fusion_msg.from_timestamp;
  const ros::Time& t2 = map_fusion_msg.to_timestamp;
  TransformationD T_t1_t2;
  tf::transformMsgToKindr(map_fusion_msg.transform, &T_t1_t2);

  if (!needRefuse(cid_a, t1, cid_b, t2)) return true;
  CHECK((!fused_time_line_[cid_a].hasTime(t1)) ||
        (!fused_time_line_[cid_b].hasTime(t2)));

  ReqState ok_a, ok_b;
  bool has_time_a = client_handlers_[cid_a]->hasTime(t1);
  bool has_time_b = client_handlers_[cid_b]->hasTime(t2);
  if (has_time_a && has_time_b) {
    // Use client id as submap id to merge submap to client map
    ok_a = client_handlers_[cid_a]->requestSubmapByTime(
        t1, submap_collection_ptr_->getNextSubmapID(), &cli_sm_id_a, &submap_a,
        &T_A_t1);
    ok_b = client_handlers_[cid_b]->requestSubmapByTime(
        t2, submap_collection_ptr_->getNextSubmapID() + 1, &submap_id_b,
        &cli_sm_id_b, &T_B_t2);

    CHECK_NE(ok_a, ReqState::FUTURE);
    CHECK_NE(ok_b, ReqState::FUTURE);

    if (verbose_) {
      LOG_IF(INFO, ok_a == ReqState::FAILED && verbose_)
          << "Requesting submap from Client " << map_fusion_msg.from_client_id
          << " failed!";

      LOG_IF(INFO, ok_a == ReqState::FUTURE && verbose_)
          << "Requested timestamp from Client " << map_fusion_msg.from_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      LOG_IF(INFO, ok_b == ReqState::FAILED && verbose_)
          << "Requesting submap from Client " << map_fusion_msg.to_client_id
          << " failed!";
      LOG_IF(INFO, ok_b == ReqState::FUTURE && verbose_)
          << "Requested timestamp from Client " << map_fusion_msg.to_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
    }
    LOG_IF(INFO, ok_a == ReqState::SUCCESS && verbose_)
        << "Received submap from Client " << map_fusion_msg.from_client_id
        << " with layer memory "
        << submap_a->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
    LOG_IF(INFO, ok_b == ReqState::SUCCESS && verbose_)
        << "Received submap from Client " << map_fusion_msg.to_client_id
        << " with layer memory "
        << cli_sm_id_b->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
  }

  bool fused_any = false;
  if (future) {
    CHECK_EQ(ok_a, ReqState::SUCCESS);
    CHECK_EQ(ok_b, ReqState::SUCCESS);
    fused_any = fuseMap(cid_a, t1, cli_sm_id_a, submap_a, T_A_t1,  // NOLINT
                        cid_b, t2, submap_id_b, cli_sm_id_b, T_B_t2,
                        T_t1_t2.cast<voxblox::FloatingPoint>());
  } else {
    if (has_time_a && has_time_b) {
      if ((ok_a == ReqState::SUCCESS && ok_b == ReqState::FUTURE) ||
          (ok_b == ReqState::SUCCESS && ok_a == ReqState::FUTURE)) {
        addToMFFuture(map_fusion_msg);
      }
      if (ok_a == ReqState::SUCCESS && ok_b == ReqState::SUCCESS) {
        fused_any = fuseMap(cid_a, t1, cli_sm_id_a, submap_a, T_A_t1,  // NOLINT
                            cid_b, t2, submap_id_b, cli_sm_id_b, T_B_t2,
                            T_t1_t2.cast<voxblox::FloatingPoint>());
      }
    } else {
      LOG_IF(INFO, !has_time_a && verbose_)
          << "Map Fusion timestamp from Client "
          << map_fusion_msg.from_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      LOG_IF(INFO, !has_time_b && verbose_)
          << "Map Fusion timestamp from Client " << map_fusion_msg.to_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      addToMFFuture(map_fusion_msg);
      processMFFuture();
    }
  }

  if (fused_any) {
    updateNeedRefuse(cid_a, t1, cid_b, t2);
    return true;
  }
  return false;
}

void CoxgraphServer::addToMFFuture(
    const coxgraph_msgs::MapFusion& map_fusion_msg) {
  if (map_fusion_msgs_future_.size() < config_.map_fusion_queue_size)
    map_fusion_msgs_future_.emplace_back(map_fusion_msg);
  else
    LOG_IF(INFO, verbose_) << "Future map fusion too many, new msg ignored";
}

void CoxgraphServer::processMFFuture() {
  LOG(INFO) << "Processing Future MF msg";
  bool processed_any = false;
  for (auto it = map_fusion_msgs_future_.begin();
       it != map_fusion_msgs_future_.end(); it++) {
    coxgraph_msgs::MapFusion map_fusion_msg = *it;
    const CliId& cid_a = map_fusion_msg.from_client_id;
    const CliId& cid_b = map_fusion_msg.to_client_id;
    const ros::Time& t1 = map_fusion_msg.from_timestamp;
    const ros::Time& t2 = map_fusion_msg.to_timestamp;
    if (client_handlers_[cid_a]->hasTime(t1) &&
        client_handlers_[cid_b]->hasTime(t2)) {
      if (mapFusionCallback(map_fusion_msg, true)) {
        processed_any = true;
        map_fusion_msgs_future_.erase(it--);
      }
    }
  }
  // Reset timeline update flag, and clear all future map fusion to avoid
  // unnecessary computation
  if (processed_any) {
    LOG_IF(INFO, verbose_)
        << "Successfully processed a MF msg, clearing MF msg queue";
    // map_fusion_msgs_future_.clear();
  }
}

bool CoxgraphServer::needRefuse(const CliId& cid_a, const ros::Time& t1,
                                const CliId& cid_b, const ros::Time& t2) {
  CHECK_EQ(force_fuse_[config_.fixed_map_client_id], false);
  // TODO(mikexyl): update need fusion flag based on time since last fusion
  if ((cid_a != config_.fixed_map_client_id &&
       (isTimeNeedRefuse(cid_a, t1) || force_fuse_[cid_a])) ||
      (cid_b != config_.fixed_map_client_id &&
       (isTimeNeedRefuse(cid_b, t2) || force_fuse_[cid_b])))
    return true;
  return false;
}

bool CoxgraphServer::updateNeedRefuse(const CliId& cid_a, const ros::Time& t1,
                                      const CliId& cid_b, const ros::Time& t2) {
  CHECK_EQ(force_fuse_[config_.fixed_map_client_id], false);
  force_fuse_[cid_a] = false;
  force_fuse_[cid_b] = false;
  // TODO(mikexyl): logically timeline update here shouldn't return false,
  // investigate this
  fused_time_line_[cid_a].update(t1);
  fused_time_line_[cid_b].update(t2);
  return true;
}

bool CoxgraphServer::fuseMap(const CliId& cid_a, const ros::Time& t1,
                             const CliSmId& cli_sm_id_a,
                             const CliSm::Ptr& submap_a,
                             const Transformation& T_A_t1, const CliId& cid_b,
                             const ros::Time& t2, const CliSmId& submap_id_b,
                             const CliSm::Ptr& cli_sm_id_b,
                             const Transformation& T_B_t2,
                             const Transformation& T_t1_t2) {
  LOG(INFO) << "Fusing: " << std::endl
            << "  Client: " << static_cast<int>(cid_a)
            << " -> Submap: " << static_cast<int>(cli_sm_id_a) << std::endl
            << "  Client: " << static_cast<int>(cid_b)
            << " -> Submap: " << static_cast<int>(submap_id_b);
  LOG_IF(INFO, verbose_) << " T_A_t1: " << std::endl << T_A_t1;
  LOG_IF(INFO, verbose_) << " T_B_t2: " << std::endl << T_B_t2;

  bool prev_result = false;

  if (optimization_async_handle_.valid() &&
      optimization_async_handle_.wait_for(std::chrono::milliseconds(10)) !=
          std::future_status::ready) {
    LOG(INFO)
        << "Previous pose graph optimization not yet complete. Waiting...";
    optimization_async_handle_.wait();
  }

  prev_result = optimization_async_handle_.valid()
                    ? (optimization_async_handle_.get() == OptState::OK)
                    : true;
  LOG(INFO) << "Result of Last Optimization" << prev_result;
  // sm_cli_id_map_.emplace(submap_a->getID(), CliIdSmIdPair(cid_a,
  // cli_sm_id_a)); sm_cli_id_map_.emplace(cli_sm_id_b->getID(),
  // CliIdSmIdPair(cid_b, submap_id_b));

  submap_collection_ptr_->addSubmap(submap_a, cid_a, cli_sm_id_a);
  submap_collection_ptr_->addSubmap(cli_sm_id_b, cid_b, submap_id_b);

  pose_graph_interface_.addSubmap(submap_a->getID());
  pose_graph_interface_.addSubmap(cli_sm_id_b->getID());

  // TODO(mikexyl): transform T_t1_t2 based on cli map frame
  Transformation T_A_B = T_A_t1 * T_t1_t2 * T_B_t2.inverse();
  pose_graph_interface_.addLoopClosureMeasurement(submap_a->getID(),
                                                  cli_sm_id_b->getID(), T_A_B);

  if (config_.enable_submap_relative_pose_constraints)
    updateSubmapRPConstraints();

  optimization_async_handle_ =
      std::async(std::launch::async, &CoxgraphServer::optimizePoseGraph, this,
                 this->config_.enable_registration_constraints);

  return prev_result;
}

void CoxgraphServer::updateSubmapRPConstraints() {
  // TODO(mikexyl): constraints wrong
  pose_graph_interface_.resetSubmapRelativePoseConstrains();
  for (int cid = 0; cid < config_.client_number; cid++) {
    std::vector<SerSmId>* cli_ser_sm_ids =
        submap_collection_ptr_->getSerSmIdsByCliId(cid);
    for (int i = 0; i < cli_ser_sm_ids->size(); i++) {
      for (int j = i + 1; j < cli_ser_sm_ids->size(); j++) {
        SerSmId sid_i = cli_ser_sm_ids->at(i);
        SerSmId sid_j = cli_ser_sm_ids->at(j);

        Transformation T_M_SMi =
            submap_collection_ptr_->getSubmapPtr(sid_i)->getPose();
        Transformation T_M_SMj =
            submap_collection_ptr_->getSubmapPtr(sid_j)->getPose();
        pose_graph_interface_.addSubmapRelativePoseConstraint(
            sid_i, sid_j, T_M_SMi.inverse() * T_M_SMj);
      }
    }
  }
}

CoxgraphServer::OptState CoxgraphServer::optimizePoseGraph(
    bool enable_registration) {
  // Optimize the pose graph
  ROS_INFO("Optimizing the pose graph");

  if (!submap_collection_ptr_->getPosesUpdateMutex()->try_lock_for(
          std::chrono::milliseconds(kPoseUpdateWaitMs))) {
    LOG(INFO) << "Waited " << kPoseUpdateWaitMs
              << "ms, pose update still unfinished, skipped optimization";
    return OptState::SKIPPED;
  }

  pose_graph_interface_.optimize(enable_registration);

  // Update the submap poses
  pose_graph_interface_.updateSubmapCollectionPoses();

  // Publish fused tfs between client mission frames and global mission frame
  publishTfCliMissionGlobal();

  // Publish Optimized Maps
  publishMaps();

  submap_collection_ptr_->getPosesUpdateMutex()->unlock();

  // Report successful completion
  return OptState::OK;
}

void CoxgraphServer::publishSmGlobalTf() {
  for (const SerSmId& submap_id : submap_collection_ptr_->getIDs()) {
  }
}

void CoxgraphServer::updateTfGlobalCli() {
  std::vector<SerSmId> ser_sm_ids = submap_collection_ptr_->getIDs();
}

void CoxgraphServer::publishMaps(const ros::Time& time) {
  LOG(INFO) << "Publishing meshes";
  if (combined_mesh_pub_.getNumSubscribers() > 0) {
    ThreadingHelper::launchBackgroundThread(
        &SubmapVisuals::publishCombinedMesh, &submap_vis_,
        *(static_cast<voxgraph::VoxgraphSubmapCollection::Ptr>(
            submap_collection_ptr_)),
        config_.output_mission_frame, combined_mesh_pub_);
  }
  if (separated_mesh_pub_.getNumSubscribers() > 0) {
    ThreadingHelper::launchBackgroundThread(
        &SubmapVisuals::publishSeparatedMesh, &submap_vis_,
        *(static_cast<voxgraph::VoxgraphSubmapCollection::Ptr>(
            submap_collection_ptr_)),
        config_.output_mission_frame, separated_mesh_pub_);
  }
}

}  // namespace coxgraph
