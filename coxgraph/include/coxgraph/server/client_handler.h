#ifndef COXGRAPH_SERVER_CLIENT_HANDLER_H_
#define COXGRAPH_SERVER_CLIENT_HANDLER_H_

#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/ClientSubmapSrv.h>
#include <coxgraph_msgs/MapPoseUpdates.h>
#include <coxgraph_msgs/MapTransform.h>
#include <coxgraph_msgs/TimeLine.h>
#include <ros/ros.h>
#include <voxgraph_msgs/LoopClosure.h>
#include <Eigen/Dense>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "coxgraph/common.h"
#include "coxgraph/server/submap_collection.h"

namespace coxgraph {
namespace server {

class ClientHandler {
 public:
  struct Config {
    Config()
        : client_name_prefix("coxgraph_client_"),
          client_loop_closure_topic_suffix("loop_closure_in"),
          client_map_pose_update_topic_suffix("map_pose_update_in"),
          pub_queue_length(1) {}
    std::string client_name_prefix;
    std::string client_loop_closure_topic_suffix;
    std::string client_map_pose_update_topic_suffix;
    int32_t pub_queue_length;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Client Handler using Config:" << std::endl
        << "  Client Name Prefix: " << v.client_name_prefix << std::endl
        << "  Client Loop Closure Topic Suffix: "
        << v.client_loop_closure_topic_suffix << std::endl
        << "  Client Map Pose Update Topic Suffix: "
        << v.client_map_pose_update_topic_suffix << std::endl
        << "  Publisher Queue Length: " << v.pub_queue_length << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  typedef std::shared_ptr<ClientHandler> Ptr;

  ClientHandler() : client_id_(-1) {}
  ClientHandler(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const CliId& client_id, const CliSmConfig& submap_config,
                const SubmapCollection::Ptr& submap_collection_ptr)
      : ClientHandler(nh, nh_private, client_id, submap_config,
                      getConfigFromRosParam(nh_private),
                      submap_collection_ptr) {}
  ClientHandler(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const CliId& client_id, const CliSmConfig& submap_config,
                const Config& config,
                const SubmapCollection::Ptr& submap_collection_ptr)
      : client_id_(client_id),
        nh_(nh),
        nh_private_(nh_private),
        config_(config),
        submap_config_(submap_config),
        client_node_name_(config.client_name_prefix + "_" +
                          std::to_string(client_id_)),
        log_prefix_("CH " + std::to_string(static_cast<int>(client_id_)) +
                    ": "),
        submap_collection_ptr_(submap_collection_ptr) {
    subscribeToTopics();
    advertiseTopics();
    subscribeToServices();
  }
  virtual ~ClientHandler() = default;

  inline const Config& getConfig() const { return config_; }

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  enum ReqState { FAILED = 0, FUTURE, SUCCESS };
  ReqState requestSubmapByTime(const ros::Time& timestamp,
                               const SerSmId& ser_sm_id, CliSmId* cli_sm_id,
                               CliSm::Ptr* submap, Transformation* T_submap_t);

  bool requestAllSubmaps(std::vector<CliSmIdPack>* submap_packs,
                         SerSmId* start_ser_sm_id);

  inline bool hasTime(const ros::Time time) { return time_line_.hasTime(time); }
  inline bool isTimeLineUpdated() const { return time_line_updated_; }
  inline void resetTimeLineUpdated() { time_line_updated_ = false; }

  inline void pubLoopClosureMsg(
      const voxgraph_msgs::LoopClosure& loop_closure_msg) {
    loop_closure_pub_.publish(loop_closure_msg);
  }

  inline void pubMapPoseTfMsg(
      const coxgraph_msgs::MapTransform& map_pose_update_msg) {
    sm_pose_tf_pub_.publish(map_pose_update_msg);
  }

 private:
  void timeLineCallback(const coxgraph_msgs::TimeLine& time_line_msg);
  void submapPoseUpdatesCallback(
      const coxgraph_msgs::MapPoseUpdates& map_pose_updates_msg);

  inline bool updateTimeLine(const ros::Time& new_start,
                             const ros::Time& new_end) {
    time_line_updated_ = time_line_.update(new_start, new_end);
    LOG(INFO) << log_prefix_ << ": Updated new client time line from "
              << time_line_.start << " to " << time_line_.end << std::endl;
  }

  void subscribeToTopics();
  void advertiseTopics();
  void subscribeToServices();

  const CliId client_id_;
  const std::string client_node_name_;
  const std::string log_prefix_;

  std::string mission_frame_id_;

  Config config_;
  CliSmConfig submap_config_;

  TimeLine time_line_;
  bool time_line_updated_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher loop_closure_pub_;
  ros::Publisher sm_pose_tf_pub_;
  ros::Subscriber time_line_sub_;
  ros::Subscriber sm_pose_updates_sub_;
  ros::ServiceClient pub_client_submap_client_;
  ros::ServiceClient get_all_submaps_client_;

  SubmapCollection::Ptr submap_collection_ptr_;

  std::mutex submap_request_mutex_;

  constexpr static int8_t kSubQueueSize = 10;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_CLIENT_HANDLER_H_
