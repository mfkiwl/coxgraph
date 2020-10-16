#ifndef COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
#define COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_

#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/ClientSubmapSrv.h>
#include <coxgraph_msgs/TimeLine.h>
#include <voxgraph/frontend/voxgraph_mapper.h>
#include <voxgraph_msgs/LoopClosure.h>

#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {

class CoxgraphClient : public voxgraph::VoxgraphMapper {
 public:
  CoxgraphClient(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : VoxgraphMapper(nh, nh_private),
        frame_names_(FrameNames::fromRosParams(nh_private)) {
    int client_id;
    nh_private.param<int>("client_id", client_id, -1);
    client_id_ = static_cast<CliId>(client_id);
    subscribeClientTopics();
    advertiseClientTopics();
    advertiseClientServices();
    if (client_id_ < 0) {
      LOG(FATAL) << "Invalid Client Id " << client_id_;
    } else {
      LOG(INFO) << "Started Coxgraph Client " << client_id_;
    }
  }
  ~CoxgraphClient() = default;

  inline const CliId& getClientId() const { return client_id_; }

  void subscribeClientTopics();
  void advertiseClientTopics();
  void advertiseClientServices();

  bool publishClientSubmapCallback(
      coxgraph_msgs::ClientSubmapSrv::Request& request,     // NOLINT
      coxgraph_msgs::ClientSubmapSrv::Response& response);  // NOLINT

  void submapCallback(const voxblox_msgs::LayerWithTrajectory& submap_msg);

 private:
  using VoxgraphMapper = voxgraph::VoxgraphMapper;

  void publishTimeLine();

  CliId client_id_;

  FrameNames frame_names_;

  ros::Publisher time_line_pub_;
  ros::ServiceServer publish_client_submap_srv_;
};

}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
