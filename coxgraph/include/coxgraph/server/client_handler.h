#ifndef COXGRAPH_SERVER_CLIENT_HANDLER_H_
#define COXGRAPH_SERVER_CLIENT_HANDLER_H_

#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/TimeLine.h>
#include <ros/ros.h>
#include <voxgraph_msgs/LoopClosure.h>
#include <Eigen/Dense>

#include <memory>
#include <string>

#include "coxgraph/common.h"

namespace coxgraph {
namespace server {

class ClientHandler {
 public:
  struct TimeLine {
    TimeLine() : start(0), end(0) {}
    ros::Time start;
    ros::Time end;
    bool hasTime(const ros::Time& time) {
      if (end.isZero()) return false;
      if (time > start && time < end) {
        return true;
      }
      return false;
    }
  };

  struct Config {
    Config()
        : client_name_prefix("coxgraph_client_"),
          client_loop_closure_topic("loop_closure_in"),
          pub_queue_length(1) {}
    std::string client_name_prefix;
    std::string client_loop_closure_topic;
    int32_t pub_queue_length;

    friend inline std::ostream& operator<<(std::ostream& s,
                                           const ClientHandler::Config& v) {
      s << std::endl
        << "Client Handler using Config:" << std::endl
        << "  Client Name Prefix: " << v.client_name_prefix << std::endl
        << "  Client Loop Closure Topic:" << v.client_loop_closure_topic
        << std::endl
        << "  Publisher Queue Length: " << v.pub_queue_length << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  typedef std::shared_ptr<ClientHandler> Ptr;

  ClientHandler() : client_id_(-1) {}
  ClientHandler(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const CliId& client_id, const CliSmConfig& submap_config)
      : ClientHandler(nh, nh_private, client_id, submap_config,
                      getConfigFromRosParam(nh_private)) {}
  ClientHandler(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const CliId& client_id, const CliSmConfig& submap_config,
                const Config& config)
      : client_id_(client_id),
        nh_(nh),
        nh_private_(nh_private),
        config_(config),
        submap_config_(submap_config),
        client_node_name_(config.client_name_prefix +
                          std::to_string(client_id_)) {
    subscribeToTopics();
    publishTopics();
    subscribeToServices();
  }
  virtual ~ClientHandler() = default;

  inline const Config& getConfig() const { return config_; }

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  bool sendLoopClosureMsg(const voxgraph_msgs::LoopClosure& loop_closure_msg);

  enum ReqState { FAILED = 0, FUTURE, SUCCESS };
  ReqState requestSubmapByTime(const ros::Time& timestamp, CliSmId* submap_id,
                               CliSm::Ptr* submap, Transformation* T_submap_t);

  inline bool hasTime(const ros::Time time) { return time_line_.hasTime(time); }
  inline bool isTimeLineUpdated() const { return time_line_updated_; }
  inline void resetTimeLineUpdated() { time_line_updated_ = false; }

  void timeLineCallback(const coxgraph_msgs::TimeLine& time_line_msg);

 private:
  void subscribeToTopics();
  void publishTopics();
  void subscribeToServices();

  const CliId client_id_;
  const std::string client_node_name_;

  Config config_;
  CliSmConfig submap_config_;

  TimeLine time_line_;
  bool time_line_updated_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher loop_closure_pub_;
  ros::Subscriber time_line_sub_;
  ros::ServiceClient pub_client_submap_client_;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_CLIENT_HANDLER_H_
