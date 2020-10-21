#include "coxgraph/server/global_tf_controller.h"

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

void GlobalTfController::initClientTf() {
  int8_t tf_grid_dim = static_cast<int8_t>(floor(sqrt(client_number_)));
  for (int i = 0; i < client_number_; i++) {
    cli_mission_frames_.emplace_back(config_.map_frame_prefix + "_" +
                                     std::to_string(i));
    int8_t cli_row = static_cast<int>(i / tf_grid_dim);
    int8_t cli_col = static_cast<int>(i % tf_grid_dim);
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
  updateCliTf(T_g_cli_mean_);
  tf_boardcaster_.sendTransform(T_g_cli_mean_);
}

// TODO(mikexyl): add locks below
void GlobalTfController::updateCliTf(
    const std::vector<tf::StampedTransform>& new_T_g_cli,
    const ros::Time& time) {
  T_g_cli_mean_ = new_T_g_cli;
  for (auto& T_g_cli : T_g_cli_mean_) {
    T_g_cli.stamp_ = time;
  }
}

void GlobalTfController::updateCliTf(const CliId& cid,
                                     const tf::StampedTransform& new_T_g_cli,
                                     const ros::Time& time) {
  T_g_cli_mean_.at(cid) = new_T_g_cli;
  T_g_cli_mean_.at(cid).stamp_ = time;
}

void GlobalTfController::resetTfGloCli() {
  for (auto& T_g_cli : T_g_cli_map_) {
    T_g_cli.second.clear();
  }
}

void GlobalTfController::addTfGloCli(const CliId& cid,
                                     const Transformation& tf) {
  T_g_cli_map_[cid].emplace_back(tf);
}

void GlobalTfController::publishTfGloCli() {}

}  // namespace server
}  // namespace coxgraph
