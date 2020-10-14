#ifndef COXGRAPH_SERVER_CLIENT_HANDLER_H_
#define COXGRAPH_SERVER_CLIENT_HANDLER_H_

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
  struct Config {
    Config() : client_name_prefix("coxgraph_client_"), pub_queue_length(1) {}
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
                const ClientId& client_id)
      : client_id_(client_id),
        nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)) {
    loop_closure_pub_ = nh_.advertise<voxgraph_msgs::LoopClosure>(
        config_.client_name_prefix + std::to_string(client_id_) + "/" +
            config_.client_loop_closure_topic,
        config_.pub_queue_length, true);
  }
  virtual ~ClientHandler() = default;

  inline const Config& getConfig() const { return config_; }

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  bool sendLoopClosureMsg(const voxgraph_msgs::LoopClosure& loop_closure_msg);

  bool requestSubmapByTime(const ros::Time& timestamp,
                           ClientSubmapId* submap_id,
                           const ClientSubmap::Ptr& submap_ptr);

 private:
  const ClientId client_id_;

  Config config_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher loop_closure_pub_;
};

}  // namespace server
}  // namespace coxgraph

#include "coxgraph/server/impl/client_handler_impl.h"

#endif  // COXGRAPH_SERVER_CLIENT_HANDLER_H_
