#include "coxgraph/server/global_tf_controller.h"

#include <mutex>
#include <string>
#include <vector>

namespace coxgraph {
namespace server {

GlobalTfController::Config GlobalTfController::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<int>("global_tf_controller/init_cli_map_dist",
                        config.init_cli_map_dist, config.init_cli_map_dist);
  nh_private.param<std::string>("global_tf_controller/map_frame_prefix",
                                config.map_frame_prefix,
                                config.map_frame_prefix);
  return config;
}

void GlobalTfController::initCliMapPose() {
  int8_t tf_grid_dim = static_cast<int8_t>(floor(sqrt(client_number_)));
  for (int i = 0; i < client_number_; i++) {
    cli_mission_frames_.emplace_back(config_.map_frame_prefix + "_" +
                                     std::to_string(i));
    int8_t cli_row = static_cast<int>(i / tf_grid_dim);
    int8_t cli_col = static_cast<int>(i % tf_grid_dim);
    client_tf_optimizer_.addClient(i, Transformation());
    T_g_cli_mean_.emplace_back(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1),
                      tf::Vector3(cli_row * config_.init_cli_map_dist,
                                  cli_col * config_.init_cli_map_dist, 0)),
        ros::Time::now(), global_mission_frame_, cli_mission_frames_[i]));
  }

  tf_pub_timer_ =
      nh_private_.createTimer(ros::Duration(1 / kTfPubFreq),
                              &GlobalTfController::pubCliTfCallback, this);
}

void GlobalTfController::pubCliTfCallback(const ros::TimerEvent& event) {
  updateCliMapPose();
  tf_boardcaster_.sendTransform(T_g_cli_mean_);
}

void GlobalTfController::addCliMapRelativePose(const CliId& first_cid,
                                               const CliId& second_cid,
                                               const Transformation& T_C1_C2) {
  client_tf_optimizer_.addClientRelativePoseMeasurement(first_cid, second_cid,
                                                        T_C1_C2);
  pose_updated_ = true;
}

void GlobalTfController::updateCliMapPose() {
  if (pose_updated_) {
    std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex);
    computeOptCliMapPose();
    pose_updated_ = false;
  }
}

void GlobalTfController::computeOptCliMapPose() {
  client_tf_optimizer_.optimize();
}

void GlobalTfController::publishTfGloCli() {}

}  // namespace server
}  // namespace coxgraph
