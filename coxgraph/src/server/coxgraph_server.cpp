#include "coxgraph/server/coxgraph_server.h"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "coxgraph/common.h"
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
  nh_private.param<bool>("loop_closure/enabled",
                         config.enable_map_fusion_constraints,
                         config.enable_map_fusion_constraints);
  nh_private.param<bool>("submap_relative_pose/enabled",
                         config.enable_submap_relative_pose_constraints,
                         config.enable_submap_relative_pose_constraints);
  nh_private.param<bool>("enable_client_loop_closure",
                         config.enable_client_loop_clousure,
                         config.enable_client_loop_clousure);
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
  if (config_.enable_client_loop_clousure)
    client_handlers_[client_id]->pubLoopClosureMsg(loop_closure_msg);
}

bool CoxgraphServer::mapFusionCallback(
    const coxgraph_msgs::MapFusion& map_fusion_msg, bool future) {
  CHECK_NE(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);

  // TODO(mikexyl): make ifs cleaner
  CliSm::Ptr submap_a, submap_b;
  CliSmId cli_sm_id_a, cli_sm_id_b;
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
    // TODO(mikexyl): add a service to request submap id, publish submap only if
    // submap id not requested before
    ok_a = client_handlers_[cid_a]->requestSubmapByTime(
        t1, submap_collection_ptr_->getNextSubmapID(), &cli_sm_id_a, &submap_a,
        &T_A_t1);
    ok_b = client_handlers_[cid_b]->requestSubmapByTime(
        t2, submap_collection_ptr_->getNextSubmapID() + 1, &cli_sm_id_b,
        &submap_b, &T_B_t2);

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
        << submap_b->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
  }

  bool fused_any = false;
  if (future) {
    CHECK_EQ(ok_a, ReqState::SUCCESS);
    CHECK_EQ(ok_b, ReqState::SUCCESS);
    fused_any = fuseMap(cid_a, t1, cli_sm_id_a, submap_a, T_A_t1,  // NOLINT
                        cid_b, t2, cli_sm_id_b, submap_b, T_B_t2,
                        T_t1_t2.cast<voxblox::FloatingPoint>());
  } else {
    if (has_time_a && has_time_b) {
      if ((ok_a == ReqState::SUCCESS && ok_b == ReqState::FUTURE) ||
          (ok_b == ReqState::SUCCESS && ok_a == ReqState::FUTURE)) {
        addToMFFuture(map_fusion_msg);
      }
      if (ok_a == ReqState::SUCCESS && ok_b == ReqState::SUCCESS) {
        fused_any = fuseMap(cid_a, t1, cli_sm_id_a, submap_a, T_A_t1,  // NOLINT
                            cid_b, t2, cli_sm_id_b, submap_b, T_B_t2,
                            T_t1_t2.cast<voxblox::FloatingPoint>());
      }
    } else {
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
}

void CoxgraphServer::processMFFuture() {
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
    // LOG_IF(INFO, verbose_)
    // << "Successfully processed a MF msg, clearing MF msg queue";
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
                             const ros::Time& t2, const CliSmId& cli_sm_id_b,
                             const CliSm::Ptr& submap_b,
                             const Transformation& T_B_t2,
                             const Transformation& T_t1_t2) {
  LOG(INFO) << "Fusing: " << std::endl
            << "  Client: " << static_cast<int>(cid_a)
            << " -> Submap: " << static_cast<int>(cli_sm_id_a) << std::endl
            << "  Client: " << static_cast<int>(cid_b)
            << " -> Submap: " << static_cast<int>(cli_sm_id_b);
  LOG_IF(INFO, verbose_) << " T_A_t1: " << std::endl << T_A_t1;
  LOG_IF(INFO, verbose_) << " T_B_t2: " << std::endl << T_B_t2;
  LOG_IF(INFO, verbose_) << " T_t1_t2: " << std::endl << T_t1_t2;

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

  // TODO(mikexyl): add a duplicate check before adding
  SerSmId ser_sm_id_a, ser_sm_id_b;
  if (submap_a->getPoseHistory().empty()) {
    // TODO(mikexyl): no need to update submap pose here, because if a submap is
    // already before, its pose will be updated via map pose update topic
    ser_sm_id_a =
        submap_collection_ptr_->getSerSmIdByCliSmId(cid_a, cli_sm_id_a);
  } else {
    ser_sm_id_a = submap_a->getID();
    submap_collection_ptr_->addSubmap(submap_a, cid_a, cli_sm_id_a);
    pose_graph_interface_.addSubmap(submap_a->getID());
  }

  if (submap_b->getPoseHistory().empty()) {
    ser_sm_id_b =
        submap_collection_ptr_->getSerSmIdByCliSmId(cid_b, cli_sm_id_b);
  } else {
    ser_sm_id_b = submap_b->getID();
    submap_collection_ptr_->addSubmap(submap_b, cid_b, cli_sm_id_b);
    pose_graph_interface_.addSubmap(submap_b->getID());
  }

  if (config_.enable_map_fusion_constraints) {
    // TODO(mikexyl): transform T_t1_t2 based on cli map frame
    Transformation T_A_B = T_A_t1 * T_t1_t2 * T_B_t2.inverse();
    pose_graph_interface_.addLoopClosureMeasurement(ser_sm_id_a, ser_sm_id_b,
                                                    T_A_B);
    geometry_msgs::Transform pose;
    tf::transformKindrToMsg(T_A_B.cast<double>(), &pose);
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.rotation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double PI = 3.14159265;
    LOG(INFO) << roll / PI * 180 << " " << pitch / PI * 180 << " "
              << yaw / PI * 180;
  }

  pose_graph_interface_.addForceRegistrationConstraint(ser_sm_id_a,
                                                       ser_sm_id_b);

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
    for (int i = 0; i < cli_ser_sm_ids->size() - 1; i++) {
      int j = i + 1;
      SerSmId sid_i = cli_ser_sm_ids->at(i);
      SerSmId sid_j = cli_ser_sm_ids->at(j);

      Transformation T_M_SMi =
          submap_collection_ptr_->getSubmapPtr(sid_i)->getPose();
      Transformation T_M_SMj =
          submap_collection_ptr_->getSubmapPtr(sid_j)->getPose();
      Transformation T_SMi_SMj = T_M_SMi.inverse() * T_M_SMj;
      pose_graph_interface_.addSubmapRelativePoseConstraint(sid_i, sid_j,
                                                            T_SMi_SMj);
      LOG(INFO) << "debug: submap rp constraints between " << sid_i << " and "
                << sid_j << std::endl
                << "from: " << std::endl
                << T_M_SMi << "to: " << std::endl
                << T_M_SMj << std::endl
                << T_SMi_SMj;
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

  TransformationVector submap_poses;
  submap_collection_ptr_->getSubmapPoses(&submap_poses);
  LOG(INFO) << "before optimize";
  for (int i = 0; i < submap_poses.size(); i++) {
    LOG(INFO) << i << std::endl << submap_poses[i];
  }

  pose_graph_interface_.optimize(enable_registration);

  if (verbose_) evaluateResiduals();

  // Update the submap poses
  // TODO(mikexyl) deprecate this
  // pose_graph_interface_.updateSubmapCollectionPoses();

  submap_collection_ptr_->getPosesUpdateMutex()->unlock();

  updateCliMapRelativePose();

  // Publish Optimized Maps
  publishMaps();

  // Report successful completion
  return OptState::OK;
}

void CoxgraphServer::evaluateResiduals() {
  if (config_.enable_map_fusion_constraints) {
    LOG(INFO) << "Evaluating Residuals of Map Fusion Constraints";
    for (double residual : pose_graph_interface_.evaluateResiduals(
             PoseGraphInterface::ConstraintType::RelPose)) {
      std::cout << residual << " ";
    }
    std::cout << std::endl;
  }
  if (config_.enable_submap_relative_pose_constraints &&
      submap_collection_ptr_->size() > 2) {
    LOG(INFO) << "Evaluating Residuals of Submap RelPose Constraints";
    for (double residual : pose_graph_interface_.evaluateResiduals(
             PoseGraphInterface::ConstraintType::SubmapRelPose)) {
      std::cout << residual << " ";
    }
    std::cout << std::endl;
  } else {
    LOG(INFO) << "No Submap RelPose Constraints added yet";
  }
}

void CoxgraphServer::updateCliMapRelativePose() {
  std::lock_guard<std::mutex> pose_update_lock(
      *(tf_controller_->getPoseUpdateMutex()));
  PoseMap pose_map = pose_graph_interface_.getPoseMap();
  TransformationVector submap_poses;
  // TODO(mikexyl) assume this submap poses vector is sorted by sm id
  submap_collection_ptr_->getSubmapPoses(&submap_poses);
  for (int i = 0; i < config_.client_number; i++) {
    std::vector<SerSmId>* ser_sm_ids_a =
        submap_collection_ptr_->getSerSmIdsByCliId(i);
    if (ser_sm_ids_a == nullptr) continue;
    for (int j = i + 1; j < config_.client_number; j++) {
      std::vector<SerSmId>* ser_sm_ids_b =
          submap_collection_ptr_->getSerSmIdsByCliId(j);
      if (ser_sm_ids_b == nullptr) continue;

      // TODO(mikexyl) these traversing add too many constraints
      for (auto const& sm_id_a : *ser_sm_ids_a) {
        for (auto const& sm_id_b : *ser_sm_ids_b) {
          Transformation T_CA_SMA = submap_poses[sm_id_a];
          Transformation T_CB_SMB = submap_poses[sm_id_b];
          Transformation T_SMA_SMB =
              pose_map[sm_id_a].inverse() * pose_map[sm_id_b];
          Transformation T_CA_CB = T_CA_SMA * T_SMA_SMB * T_CB_SMB.inverse();
          tf_controller_->addCliMapRelativePose(i, j, T_CA_CB);
        }
      }
    }
  }
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
