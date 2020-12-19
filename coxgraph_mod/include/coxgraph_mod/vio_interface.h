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
  VIOInterface() : initialized_(false) {}

  ~VIOInterface() = default;

  void updatePose(Eigen::Matrix4d pose, double timestamp) {
    init();
    if (tf_pub_ == nullptr) return;
    tf_pub_->updatePose(pose, timestamp);
  }

  void publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp,
                          Eigen::Matrix4d T_A_B) {
    init();
    loop_closure_pub_->publishLoopClosure(from_client_id, from_timestamp,
                                          to_client_id, to_timestamp, T_A_B);
  }

  void publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp, cv::Mat R,
                          cv::Mat t) {
    init();
    loop_closure_pub_->publishLoopClosure(from_client_id, from_timestamp,
                                          to_client_id, to_timestamp, R, t);
  }

  void publishLoopClosure(const double& from_timestamp,
                          const double& to_timestamp, Eigen::Matrix4d T_A_B) {
    init();
    loop_closure_pub_->publishLoopClosure(from_timestamp, to_timestamp, T_A_B);
  }

  void publishLoopClosure(const double& from_timestamp,
                          const double& to_timestamp, cv::Mat R, cv::Mat t) {
    init();
    loop_closure_pub_->publishLoopClosure(from_timestamp, to_timestamp, R, t);
  }

 private:
  ros::NodeHandle* nh_;
  ros::NodeHandle* nh_private_;

  bool initialized_;

  void init() {
    if (initialized_) return;
    nh_ = new ros::NodeHandle();
    nh_private_ = new ros::NodeHandle("~");
    nh_private_->param<bool>("server_mode", server_mode_, false);

    if (!server_mode_) tf_pub_.reset(new TfPublisher(*nh_, *nh_private_));
    loop_closure_pub_.reset(
        new LoopClosurePublisher(*nh_, *nh_private_, server_mode_));

    initialized_ = true;
  }

  bool server_mode_;

  TfPublisher::Ptr tf_pub_;
  LoopClosurePublisher::Ptr loop_closure_pub_;
};

}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_VIO_INTERFACE_H_
