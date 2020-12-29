#ifndef COXGRAPH_MOD_VIO_INTERFACE_H_
#define COXGRAPH_MOD_VIO_INTERFACE_H_

#include <Eigen/Dense>

#include "coxgraph_mod/common.h"
#include "coxgraph_mod/loop_closure_publisher.h"
#include "coxgraph_mod/tf_publisher.h"

namespace coxgraph {
namespace mod {

void updatePose(Eigen::Matrix4d pose, double timestamp);
void updatePose(cv::Mat pose, double timestamp);
void publishLoopClosure(size_t from_client_id, double from_timestamp,
                        size_t to_client_id, double to_timestamp,
                        Eigen::Matrix4d T_A_B);
void publishLoopClosure(size_t from_client_id, double from_timestamp,
                        size_t to_client_id, double to_timestamp, cv::Mat R,
                        cv::Mat t);
void publishLoopClosure(const double& from_timestamp,
                        const double& to_timestamp, Eigen::Matrix4d T_A_B);
void publishLoopClosure(const double& from_timestamp,
                        const double& to_timestamp, cv::Mat R, cv::Mat t);
bool toggleMapping(bool b_mapping);
}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_VIO_INTERFACE_H_
