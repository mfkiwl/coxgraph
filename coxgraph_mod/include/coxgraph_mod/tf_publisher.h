#ifndef COXGRAPH_MOD_TF_PUBLISHER_H_
#define COXGRAPH_MOD_TF_PUBLISHER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

#include <memory>
#include <mutex>
#include <string>

#include "coxgraph_mod/common.h"

namespace coxgraph {
namespace mod {

class TfPublisher {
 public:
  typedef std::shared_ptr<TfPublisher> Ptr;

  TfPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
              int client_id)
      : nh_(nh), nh_private_(nh_private), current_time_(ros::Time::now()) {
    tf_timer_ =
        nh_.createTimer(ros::Duration(0.01),
                        &TfPublisher::PublishPositionAsTransformCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 10, true);
    nh_private_.param<std::string>(
        "map_frame", map_frame_,
        "map_" + static_cast<std::string>(std::to_string(client_id)));
    nh_private_.param<std::string>(
        "camera_frame", camera_frame_,
        "robot_base_" + static_cast<std::string>(std::to_string(client_id)));
  }
  ~TfPublisher() = default;

  void updatePose(Eigen::Matrix4d pose, double timestamp) {
    std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
    current_position_ = TransformFromMat(pose);
    current_time_.fromSec(timestamp);
  }

  void PublishPositionAsTransformCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        current_position_, current_time_, map_frame_, camera_frame_));
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = camera_frame_;
    odom_msg.pose.pose.position.x = current_position_.getOrigin().x();
    odom_msg.pose.pose.position.y = current_position_.getOrigin().y();
    odom_msg.pose.pose.position.z = current_position_.getOrigin().z();
    odom_msg.pose.pose.orientation.w = current_position_.getRotation().w();
    odom_msg.pose.pose.orientation.x = current_position_.getRotation().x();
    odom_msg.pose.pose.orientation.y = current_position_.getRotation().y();
    odom_msg.pose.pose.orientation.z = current_position_.getRotation().z();
    odom_pub_.publish(odom_msg);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Time current_time_;
  tf::Transform current_position_;
  std::string map_frame_;
  std::string camera_frame_;

  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_pub_;

  ros::Timer tf_timer_;

  std::mutex pose_update_mutex_;

  tf::Transform TransformFromMat(Eigen::Matrix4d position_mat) {
    tf::Matrix3x3 tf_camera_rotation(
        position_mat(0, 0), position_mat(0, 1), position_mat(0, 2),
        position_mat(1, 0), position_mat(1, 1), position_mat(1, 2),
        position_mat(2, 0), position_mat(2, 1), position_mat(2, 2));

    tf::Vector3 tf_camera_translation(position_mat(0, 3), position_mat(1, 3),
                                      position_mat(2, 3));

    // Coordinate transformation matrix from orb coordinate system to ros
    // coordinate system
    const tf::Matrix3x3 tf_orb_to_ros(0, 0, 1, -1, 0, 0, 0, -1, 0);

    // Transform from orb coordinate system to ros coordinate system on camera
    // coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map
    // coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf::Transform(tf_camera_rotation, tf_camera_translation);
  }
};

}  // namespace mod
}  // namespace coxgraph

#endif  // COXGRAPH_MOD_TF_PUBLISHER_H_
