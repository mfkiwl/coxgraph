#ifndef COXGRAPH_SERVER_GLOBAL_TF_CONTROLLER_H_
#define COXGRAPH_SERVER_GLOBAL_TF_CONTROLLER_H_

#include <tf/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/client_tf_optimizer.h"
#include "coxgraph/server/pose_graph_interface.h"

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
  using PoseMap = ClientTfOptimizer::PoseMap;

  GlobalTfController(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private, int8_t client_number,
                     bool in_control, bool verbose)
      : GlobalTfController(nh, nh_private, client_number, in_control,
                           getConfigFromRosParam(nh_private), verbose) {}

  GlobalTfController(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private, int8_t client_number,
                     bool in_control, const Config& config, bool verbose)
      : verbose_(verbose),
        nh_(nh),
        nh_private_(nh_private),
        client_number_(client_number),
        in_control_(in_control),
        config_(config),
        global_mission_frame_(config.map_frame_prefix + "_g"),
        client_tf_optimizer_(nh_private, verbose),
        pose_updated_(false) {
    LOG(INFO) << config;
    initCliMapPose();
  }

  ~GlobalTfController() = default;

  const std::string& getGlobalMissionFrame() const {
    return global_mission_frame_;
  }

  void publishTfGloCli();

  void addCliMapRelativePose(const CliId& first_cid, const CliId& second_cid,
                             const Transformation& T_C1_C2);

  void resetCliMapRelativePoses() {
    client_tf_optimizer_.resetClientRelativePoseConstraints();
  }

  std::mutex* getPoseUpdateMutex() { return &pose_update_mutex; }

  inline bool inControl() const { return in_control_; }
  inline void setControl(bool in_control) { in_control_ = in_control; }

 private:
  void initCliMapPose();

  void pubCliTfCallback(const ros::TimerEvent& event);

  void updateCliMapPose();

  void computeOptCliMapPose();

  bool verbose_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Config config_;

  const int8_t client_number_;
  const std::string global_mission_frame_;
  std::vector<std::string> cli_mission_frames_;

  ros::Timer tf_pub_timer_;
  tf::TransformBroadcaster tf_boardcaster_;
  std::vector<bool> cli_tf_fused_;
  std::vector<tf::StampedTransform> T_G_CLI_opt_;

  ClientTfOptimizer client_tf_optimizer_;
  bool pose_updated_;

  std::mutex pose_update_mutex;

  bool in_control_;

  constexpr static float kTfPubFreq = 100;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_GLOBAL_TF_CONTROLLER_H_
