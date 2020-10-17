#ifndef COXGRAPH_SERVER_GLOBAL_TF_CONTROLLER_H_
#define COXGRAPH_SERVER_GLOBAL_TF_CONTROLLER_H_

#include <tf/transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

#include "coxgraph/common.h"

namespace coxgraph {
namespace server {

class GlobalTfController {
 public:
  struct Config {
    Config() : init_cli_map_dist(10), map_frame_prefix("map") {}
    int32_t init_cli_map_dist;
    std::string map_frame_prefix;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Global Tf Controller using Config" << std::endl
        << "  Initial Client Mission Frame Distance: " << v.init_cli_map_dist
        << std::endl
        << "  Mission Frames Prefix: " << v.map_frame_prefix << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<GlobalTfController> Ptr;

  GlobalTfController(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private, int8_t client_number)
      : GlobalTfController(nh, nh_private, client_number,
                           getConfigFromRosParam(nh_private)) {}

  GlobalTfController(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private, int8_t client_number,
                     const Config& config)
      : nh_(nh),
        nh_private_(nh_private),
        client_number_(client_number),
        config_(config),
        global_mission_frame_(config.map_frame_prefix + "_g") {
    LOG(INFO) << config;
    initClientTf();
  }
  ~GlobalTfController() = default;

 private:
  void initClientTf();

  void pubCliTfCallback(const ros::TimerEvent& event);

  void updateCliTf(const std::vector<tf::StampedTransform>& new_T_g_cli,
                   const ros::Time& time = ros::Time::now());
  void updateCliTf(const CliId& cid, const tf::StampedTransform& new_T_g_cli,
                   const ros::Time& time = ros::Time::now());

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Config config_;

  const int8_t client_number_;
  const std::string global_mission_frame_;
  std::vector<std::string> cli_mission_frames_;

  ros::Timer tf_pub_timer_;
  tf::TransformBroadcaster tf_boardcaster_;
  std::vector<bool> cli_tf_fused_;
  std::vector<tf::StampedTransform> T_g_cli_;

  constexpr static float kTfPubFreq = 100;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_GLOBAL_TF_CONTROLLER_H_
