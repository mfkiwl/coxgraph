#ifndef COXGRAPH_MOD_LOOP_CLOSURE_PUBLISHER_H_
#define COXGRAPH_MOD_LOOP_CLOSURE_PUBLISHER_H_

#include <coxgraph_msgs/LoopClosure.h>
#include <coxgraph_msgs/MapFusion.h>
#include <eigen_conversions/eigen_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

#include <memory>

#include "coxgraph_mod/common.h"

namespace coxgraph {
namespace mod {
class LoopClosurePublisher {
 public:
  typedef std::shared_ptr<LoopClosurePublisher> Ptr;

  LoopClosurePublisher(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       bool server_mode = false)
      : nh_(nh), nh_private_(nh_private), server_mode_(server_mode) {
    if (server_mode_)
      loop_closure_pub_ = nh_private_.advertise<coxgraph_msgs::MapFusion>(
          "map_fusion", 10, true);
    else
      loop_closure_pub_ = nh_private_.advertise<coxgraph_msgs::LoopClosure>(
          "loop_closure", 10, true);
  }
  ~LoopClosurePublisher() = default;

  bool publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp,
                          Eigen::Matrix4d T_A_B) {
    tf2::Matrix3x3 tf2_rot(T_A_B(0, 0), T_A_B(0, 1), T_A_B(0, 2), T_A_B(1, 0),
                           T_A_B(1, 1), T_A_B(1, 2), T_A_B(2, 0), T_A_B(2, 1),
                           T_A_B(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);

    coxgraph_msgs::MapFusion map_fusion_msg;
    map_fusion_msg.from_client_id = from_client_id;
    map_fusion_msg.from_timestamp = ros::Time(from_timestamp);
    map_fusion_msg.to_client_id = to_client_id;
    map_fusion_msg.to_timestamp = ros::Time(to_timestamp);
    map_fusion_msg.transform.rotation = tf2::toMsg(tf2_quaternion);
    map_fusion_msg.transform.translation.x = T_A_B(0, 3);
    map_fusion_msg.transform.translation.y = T_A_B(1, 3);
    map_fusion_msg.transform.translation.z = T_A_B(2, 3);
    loop_closure_pub_.publish(map_fusion_msg);
    ROS_INFO(
        "Map Fusion Message Published, from client %d time %d, to client "
        "%d time %d ",
        static_cast<int>(from_client_id),
        static_cast<int>(map_fusion_msg.from_timestamp.toSec()),
        static_cast<int>(to_client_id),
        static_cast<int>(map_fusion_msg.to_timestamp.toSec()));

    return true;
  }

  bool publishLoopClosure(const double& from_timestamp,
                          const double& to_timestamp, Eigen::Matrix4d T_A_B) {
    tf2::Matrix3x3 tf2_rot(T_A_B(0, 0), T_A_B(0, 1), T_A_B(0, 2), T_A_B(1, 0),
                           T_A_B(1, 1), T_A_B(1, 2), T_A_B(2, 0), T_A_B(2, 1),
                           T_A_B(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);

    coxgraph_msgs::LoopClosure loop_closure_msg;
    loop_closure_msg.from_timestamp = ros::Time(from_timestamp);
    loop_closure_msg.to_timestamp = ros::Time(to_timestamp);
    loop_closure_msg.transform.rotation = tf2::toMsg(tf2_quaternion);
    loop_closure_msg.transform.translation.x = T_A_B(0, 3);
    loop_closure_msg.transform.translation.y = T_A_B(1, 3);
    loop_closure_msg.transform.translation.z = T_A_B(2, 3);
    loop_closure_pub_.publish(loop_closure_msg);
    ROS_INFO(
        "Loop Closure Message Published from time %d, to "
        "time %d",
        static_cast<int>(loop_closure_msg.from_timestamp.toSec()),
        static_cast<int>(loop_closure_msg.to_timestamp.toSec()));

    return true;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool server_mode_;

  ros::Publisher loop_closure_pub_;
};
}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_LOOP_CLOSURE_PUBLISHER_H_
