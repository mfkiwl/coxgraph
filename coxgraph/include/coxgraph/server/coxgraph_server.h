#ifndef COXGRAPH_SERVER_COXGRAPH_SERVER_H_
#define COXGRAPH_SERVER_COXGRAPH_SERVER_H_

#include <coxgraph_msgs/ControlTrigger.h>
#include <coxgraph_msgs/MapFusion.h>
#include <ros/ros.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_ros/ros_params.h>
#include <voxgraph/frontend/pose_graph_interface/pose_graph_interface.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>
#include <voxgraph/tools/ros_params.h>
#include <voxgraph/tools/threading_helper.h>
#include <voxgraph/tools/visualization/submap_visuals.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/client_handler.h"
#include "coxgraph/server/distribution/distribution_controller.h"
#include "coxgraph/server/global_tf_controller.h"
#include "coxgraph/server/pose_graph_interface.h"
#include "coxgraph/server/submap_collection.h"
#include "coxgraph/server/visualizer/server_visualizer.h"

namespace coxgraph {

class CoxgraphServer {
 public:
  struct Config {
    Config()
        : client_number(0),
          map_fusion_queue_size(10),
          refuse_interval(ros::Duration(2)),
          fixed_map_client_id(0),
          output_mission_frame("mission"),
          enable_registration_constraints(true),
          enable_map_fusion_constraints(true),
          publisher_queue_length(100),
          mesh_opacity(1.0),
          submap_mesh_color_mode("lambert_color"),
          combined_mesh_color_mode("normals"),
          enable_submap_relative_pose_constraints(true) {}
    int32_t client_number;
    int32_t map_fusion_queue_size;
    ros::Duration refuse_interval;
    int32_t fixed_map_client_id;
    std::string output_mission_frame;
    bool enable_registration_constraints;
    bool enable_map_fusion_constraints;
    bool enable_submap_relative_pose_constraints;
    int32_t publisher_queue_length;
    float mesh_opacity;
    std::string submap_mesh_color_mode;
    std::string combined_mesh_color_mode;
    bool enable_client_loop_clousure;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Coxgraph Server using Config:" << std::endl
        << "  Client Number: " << v.client_number << std::endl
        << "  Map Fusion Queue Size: " << v.map_fusion_queue_size << std::endl
        << "  Client Map Refusion Interval: " << v.refuse_interval << " s"
        << std::endl
        << "  Map Fixed for Client Id: " << v.fixed_map_client_id << std::endl
        << "  Output Mission Frame: " << v.output_mission_frame << std::endl
        << "  Registration Constraint: "
        << static_cast<std::string>(
               v.enable_registration_constraints ? "enabled" : "disabled")
        << std::endl
        << "  Map Fusion Constraint: "
        << static_cast<std::string>(
               v.enable_map_fusion_constraints ? "enabled" : "disabled")
        << std::endl
        << "  Submap Relative Pose Constraint: "
        << static_cast<std::string>(v.enable_submap_relative_pose_constraints
                                        ? "enabled"
                                        : "disabled")
        << std::endl
        << "  Client Loop Closure: "
        << static_cast<std::string>(v.enable_client_loop_clousure ? "enabled"
                                                                  : "disabled")
        << std::endl
        << "  Publisher Queue Length: " << v.publisher_queue_length << std::endl
        << "  Mesh Opacity: " << v.mesh_opacity << std::endl
        << "  Submap Mesh Color Mode: " << v.submap_mesh_color_mode << std::endl
        << "  Combined Mesh Color Mode: " << v.combined_mesh_color_mode
        << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  CoxgraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : CoxgraphServer(
            nh, nh_private, getConfigFromRosParam(nh_private),
            voxgraph::getVoxgraphSubmapConfigFromRosParams(nh_private),
            voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

  CoxgraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 const Config& config, const CliSmConfig& submap_config,
                 const MeshIntegratorConfig& mesh_config)
      : nh_(nh),
        nh_private_(nh_private),
        verbose_(false),
        config_(config),
        submap_config_(submap_config),
        submap_collection_ptr_(std::make_shared<SubmapCollection>(
            submap_config_, config.client_number)),
        pose_graph_interface_(nh_private, submap_collection_ptr_, mesh_config,
                              config.output_mission_frame, false),
        server_vis_(submap_config, mesh_config) {
    nh_private_.param<bool>("verbose", verbose_, verbose_);
    LOG(INFO) << "Verbose: " << verbose_;
    LOG(INFO) << config_;

    distrib_ctl_ptr_.reset(
        new DistributionController(nh_, nh_private_, submap_collection_ptr_));

    tf_controller_.reset(new GlobalTfController(
        nh_, nh_private_, config_.client_number, distrib_ctl_ptr_, verbose_));
    pose_graph_interface_.setVerbosity(verbose_);
    pose_graph_interface_.setMeasurementConfigFromRosParams(nh_private_);

    subscribeTopics();
    advertiseTopics();
    advertiseServices();
    initClientHandlers(nh_, nh_private_);

    LOG(INFO) << client_handlers_[0]->getConfig();

    CHECK_EQ(config_.fixed_map_client_id, 0)
        << "Fixed map client id has to be set 0 now, since pose graph "
           "optimization set pose of submap 0 as constant";

    server_vis_.setMeshOpacity(config_.mesh_opacity);
    server_vis_.setSubmapMeshColorMode(
        voxblox::getColorModeFromString(config_.submap_mesh_color_mode));
    server_vis_.setCombinedMeshColorMode(
        voxblox::getColorModeFromString(config_.combined_mesh_color_mode));

    // TODO(mikexyl): make a param for future mf proc timer
    // future_msg_proc_timer_ =
    //     nh_.createTimer(ros::Duration(kFutureMFProcInterval),
    //                     &CoxgraphServer::futureMFProcCallback, this);
  }

  ~CoxgraphServer() = default;

  void subscribeTopics();
  void advertiseTopics();
  void advertiseServices();

  void mapFusionMsgCallback(const coxgraph_msgs::MapFusion& map_fusion_msg);

  bool getFinalGlobalMeshCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT

  void futureMFProcCallback(const ros::TimerEvent& event);

 private:
  using ClientHandler = server::ClientHandler;
  using GlobalTfController = server::GlobalTfController;
  using ReqState = ClientHandler::ReqState;
  using SubmapCollection = server::SubmapCollection;
  using PoseGraphInterface = server::PoseGraphInterface;
  using ThreadingHelper = voxgraph::ThreadingHelper;
  using PoseMap = PoseGraphInterface::PoseMap;
  using ServerVisualizer = server::ServerVisualizer;
  using DistributionController = server::DistributionController;

  void initClientHandlers(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);

  void loopClosureCallback(const CliId& client_id,
                           const voxgraph_msgs::LoopClosure& loop_closure_msg);
  bool mapFusionCallback(const coxgraph_msgs::MapFusion& map_fusion_msg,
                         bool future = false);
  void addToMFFuture(const coxgraph_msgs::MapFusion& map_fusion_msg);
  void processMFFuture();
  void timeLineUpdateCallback() { processMFFuture(); }
  bool needRefuse(const CliId& cid_a, const ros::Time& time_a,
                  const CliId& cid_b, const ros::Time& time_b);
  bool updateNeedRefuse(const CliId& cid_a, const ros::Time& time_a,
                        const CliId& cid_b, const ros::Time& time_b);

  // TODO(mikexyl): make a pack struct for these vars
  bool fuseMap(const CliId& cid_a, const ros::Time& time_a,
               const CliSmId& cli_sm_id_a, const CliSm::Ptr& submap_a,
               const Transformation& T_submap_t_a, const CliId& cid_b,
               const ros::Time& time_b, const CliSmId& cli_sm_id_b,
               const CliSm::Ptr& submap_b, const Transformation& T_submap_t_b,
               const Transformation& T_t1_t2);

  void updateSubmapRPConstraints();

  enum OptState { FAILED = 0, OK, SKIPPED };
  OptState optimizePoseGraph(bool enable_registration);

  void evaluateResiduals();

  void updateCliMapRelativePose();

  inline bool isTimeFused(const CliId& cid, const ros::Time& time) {
    return fused_time_line_[cid].hasTime(time);
  }
  inline bool isTimeNeedRefuse(const CliId& cid, const ros::Time& time) {
    if (config_.refuse_interval == ros::Duration(0))
      return !isTimeFused(cid, time);
    if (isTimeFused(cid, time)) return false;
    if (time < fused_time_line_[cid].start) {
      return (fused_time_line_[cid].start - time) > config_.refuse_interval;
    } else if (time > fused_time_line_[cid].end) {
      return (time - fused_time_line_[cid].end) > config_.refuse_interval;
    }
    return false;
  }

  inline SerSmId addSubmap(const CliSm::Ptr& submap, const CliId& cid,
                           const CliSmId& cli_sm_id) {
    std::lock_guard<std::mutex> submap_add_lock(submap_add_mutex_);
    submap_collection_ptr_->addSubmap(submap, cid, cli_sm_id);
    pose_graph_interface_.addSubmap(submap->getID());
    return submap->getID();
  }

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber map_fusion_sub_;

  // Verbosity and debug mode
  bool verbose_;

  const Config config_;

  const CliSmConfig submap_config_;
  SubmapCollection::Ptr submap_collection_ptr_;
  PoseGraphInterface pose_graph_interface_;
  std::mutex submap_add_mutex_;

  std::vector<ClientHandler::Ptr> client_handlers_;

  // Map fusion msg process related
  std::deque<std::pair<coxgraph_msgs::MapFusion, int>> map_fusion_msgs_future_;
  std::vector<bool> force_fuse_;
  std::vector<TimeLine> fused_time_line_;
  std::map<SerSmId, SerSmId> fused_ser_sm_id_pair;
  std::mutex map_fuse_mutex_;
  std::mutex future_mf_queue_mutex_;
  ros::Timer future_msg_proc_timer_;

  // Asynchronous handle for the pose graph optimization thread
  std::future<OptState> optimization_async_handle_;

  GlobalTfController::Ptr tf_controller_;

  // Visualization
  ServerVisualizer server_vis_;
  ros::Publisher combined_mesh_pub_;
  ros::Publisher separated_mesh_pub_;
  ros::ServiceServer get_final_global_mesh_srv_;
  std::timed_mutex final_mesh_gen_mutex_;

  DistributionController::Ptr distrib_ctl_ptr_;
  inline bool inControl() const { return distrib_ctl_ptr_->inControl(); }

  constexpr static uint8_t kMaxClientNum = 2;
  constexpr static uint8_t kPoseUpdateWaitMs = 100;
  constexpr static float kFutureMFProcInterval = 1.0;
  constexpr static int kMaxFutureUncatchedN = 4;
};

}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_COXGRAPH_SERVER_H_
