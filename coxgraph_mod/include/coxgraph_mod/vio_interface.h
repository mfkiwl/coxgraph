#ifndef COXGRAPH_MOD_VIO_INTERFACE_H_
#define COXGRAPH_MOD_VIO_INTERFACE_H_

#include <Eigen/Dense>

#include "coxgraph_mod/common.h"
#include "coxgraph_mod/loop_closure_publisher.h"
#include "coxgraph_mod/tf_publisher.h"

namespace coxgraph {
namespace mod {

class VIOInterface {
 public:
  VIOInterface() {
    bool server_mode = false;
    nh_private_.param<bool>("is_server", server_mode, server_mode);
    VIOInterface(ros::NodeHandle(), ros::NodeHandle("~"), server_mode);
  }

  VIOInterface(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
               bool server_mode = false)
      : server_mode(server_mode) {
    if (!server_mode) tf_pub_.reset(new TfPublisher(nh_, nh_private_));
    loop_closure_pub_.reset(new LoopClosurePublisher(nh_, nh_private_));
  }
  ~VIOInterface() = default;

  void updatePose(Eigen::Matrix4d pose, double timestamp) {
    if (tf_pub_ == nullptr) return;
    tf_pub_->updatePose(pose, timestamp);
  }

  void publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp,
                          Eigen::Matrix4d T_A_B) {
    loop_closure_pub_->publishLoopClosure(from_client_id, from_timestamp,
                                          to_client_id, to_timestamp, T_A_B);
  }

  void publishLoopClosure(const double& from_timestamp,
                          const double& to_timestamp, Eigen::Matrix4d T_A_B) {
    loop_closure_pub_->publishLoopClosure(from_timestamp, to_timestamp, T_A_B);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool server_mode;

  TfPublisher::Ptr tf_pub_;
  LoopClosurePublisher::Ptr loop_closure_pub_;
};

}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_VIO_INTERFACE_H_
