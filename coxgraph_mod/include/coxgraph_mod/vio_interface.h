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
  VIOInterface(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
               int client_id = -1)
      : client_id_(client_id) {
    if (client_id >= 0)
      tf_pub_.reset(new TfPublisher(nh_, nh_private_, client_id));
    loop_closure_pub_.reset(new LoopClosurePublisher(nh_, nh_private_));
  }
  ~VIOInterface() = default;

  void updatePose(Eigen::Matrix4d pose, double timestamp) {
    if (tf_pub_ == nullptr) return;
    tf_pub_->updatePose(pose, timestamp);
  }

  void publishMapFusion(size_t from_client_id, double from_timestamp,
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

  int client_id_;

  TfPublisher::Ptr tf_pub_;
  LoopClosurePublisher::Ptr loop_closure_pub_;
};

}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_VIO_INTERFACE_H_
