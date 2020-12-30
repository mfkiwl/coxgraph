#ifndef COXGRAPH_MOD_LOOP_CLOSURE_PUBLISHER_H_
#define COXGRAPH_MOD_LOOP_CLOSURE_PUBLISHER_H_

#include <coxgraph/common.h>
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
#include <opencv2/opencv.hpp>

#include <map>
#include <memory>
#include <string>

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
    loop_closure_pub_.emplace(
        -1, nh_private_.advertise<coxgraph_msgs::MapFusion>("map_fusion", 10,
                                                            true));
    loop_closure_topic_ = nh_private_.param<std::string>(
        "loop_closure_topic_prefix", loop_closure_topic_, "loop_closure_in_");
  }

  ~LoopClosurePublisher() = default;

  bool publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp,
                          geometry_msgs::Quaternion rotation,
                          geometry_msgs::Vector3 transform) {
    coxgraph_msgs::MapFusion map_fusion_msg;
    map_fusion_msg.from_client_id = from_client_id;
    map_fusion_msg.from_timestamp = ros::Time(from_timestamp);
    map_fusion_msg.to_client_id = to_client_id;
    map_fusion_msg.to_timestamp = ros::Time(to_timestamp);
    map_fusion_msg.transform.rotation = rotation;
    map_fusion_msg.transform.translation = transform;

    if (from_client_id != to_client_id) {
      ROS_FATAL(
          "Map Fusion Message Published, from client %d time %d, to client "
          "%d time %d ",
          static_cast<int>(from_client_id),
          static_cast<int>(map_fusion_msg.from_timestamp.toSec()),
          static_cast<int>(to_client_id),
          static_cast<int>(map_fusion_msg.to_timestamp.toSec()));
      loop_closure_pub_[-1].publish(map_fusion_msg);
    } else {
      ROS_INFO(
          "Loop Closure Message Published, from client %d time %d, to client "
          "%d time %d ",
          static_cast<int>(from_client_id),
          static_cast<int>(map_fusion_msg.from_timestamp.toSec()),
          static_cast<int>(to_client_id),
          static_cast<int>(map_fusion_msg.to_timestamp.toSec()));
      publishLoopClosure(from_client_id, from_timestamp, to_timestamp, rotation,
                         transform);
    }
    return true;
  }
  bool publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp,
                          Eigen::Matrix4d T_A_B) {
    return publishLoopClosure(from_client_id, from_timestamp, to_client_id,
                              to_timestamp, toGeoQuat(T_A_B), toGeoVec3(T_A_B));
  }

  bool publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp, cv::Mat R,
                          cv::Mat t) {
    return publishLoopClosure(from_client_id, from_timestamp, to_client_id,
                              to_timestamp, toGeoQuat(R), toGeoVec3(t));
  }

  bool publishLoopClosure(CliId cid, double from_timestamp, double to_timestamp,
                          geometry_msgs::Quaternion rotation,
                          geometry_msgs::Vector3 transform) {
    coxgraph_msgs::LoopClosure loop_closure_msg;
    loop_closure_msg.from_timestamp = ros::Time(from_timestamp);
    loop_closure_msg.to_timestamp = ros::Time(to_timestamp);
    loop_closure_msg.transform.rotation = rotation;
    loop_closure_msg.transform.translation = transform;
    if (!loop_closure_pub_.count(cid))
      loop_closure_pub_.emplace(
          cid, nh_private_.advertise<coxgraph_msgs::LoopClosure>(
                   loop_closure_topic_ + std::to_string(cid), 10, true));
    loop_closure_pub_[cid].publish(loop_closure_msg);
    return true;
  }

  bool publishLoopClosure(CliId cid, const double& from_timestamp,
                          const double& to_timestamp, Eigen::Matrix4d T_A_B) {
    return publishLoopClosure(cid, from_timestamp, to_timestamp,
                              toGeoQuat(T_A_B), toGeoVec3(T_A_B));
  }

  bool publishLoopClosure(CliId cid, const double& from_timestamp,
                          const double& to_timestamp, cv::Mat R, cv::Mat t) {
    return publishLoopClosure(cid, from_timestamp, to_timestamp, toGeoQuat(R),
                              toGeoVec3(t));
  }

 private:
  geometry_msgs::Quaternion toGeoQuat(cv::Mat R) {
    tf2::Matrix3x3 tf2_rot(
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);
    return tf2::toMsg(tf2_quaternion);
  }

  geometry_msgs::Quaternion toGeoQuat(Eigen::Matrix4d T_A_B) {
    tf2::Matrix3x3 tf2_rot(T_A_B(0, 0), T_A_B(0, 1), T_A_B(0, 2), T_A_B(1, 0),
                           T_A_B(1, 1), T_A_B(1, 2), T_A_B(2, 0), T_A_B(2, 1),
                           T_A_B(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);
    return tf2::toMsg(tf2_quaternion);
  }

  geometry_msgs::Vector3 toGeoVec3(Eigen::Matrix4d T_A_B) {
    geometry_msgs::Vector3 transform;
    transform.x = T_A_B(0, 3);
    transform.y = T_A_B(1, 3);
    transform.z = T_A_B(2, 3);
    return transform;
  }

  geometry_msgs::Vector3 toGeoVec3(cv::Mat t) {
    geometry_msgs::Vector3 transform;
    transform.x = t.at<float>(0);
    transform.y = t.at<float>(1);
    transform.z = t.at<float>(2);
    return transform;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool server_mode_;

  std::string loop_closure_topic_;
  std::map<int8_t, ros::Publisher> loop_closure_pub_;
};

}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_LOOP_CLOSURE_PUBLISHER_H_
