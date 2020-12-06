#include "coxgraph_sim/odometry_transform_publisher.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include <string>

namespace coxgraph {

OdometryTransformPublisher::Config
OdometryTransformPublisher::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<std::string>("odom_frame", config.odom_frame,
                                config.odom_frame);
  nh_private.param<std::string>("base_link_frame", config.base_link_frame,
                                config.base_link_frame);
  nh_private.param<float>("tf_pub_frequency", config.tf_pub_frequency,
                          config.tf_pub_frequency);
  nh_private.param<float>("origin_x", config.origin_pos[0],
                          config.origin_pos[0]);
  nh_private.param<float>("origin_y", config.origin_pos[1],
                          config.origin_pos[1]);
  nh_private.param<float>("origin_z", config.origin_pos[2],
                          config.origin_pos[2]);
  nh_private.param<float>("origin_yaw", config.origin_yaw, config.origin_yaw);
  return config;
}

void OdometryTransformPublisher::setOrigin() {
  tf::Transform transform;
  tf::Vector3 position;
  position.setX(config_.origin_pos[0]);
  position.setY(config_.origin_pos[1]);
  position.setZ(config_.origin_pos[2]);
  transform.setOrigin(position);

  tf::Quaternion quaternion = tf::createQuaternionFromYaw(config_.origin_yaw);
  transform.setRotation(quaternion);

  tf::transformTFToKindr(transform, &T_G_O_);
}

void OdometryTransformPublisher::subscribeTopics() {
  gt_odom_sub_ = nh_private_.subscribe(
      "odom_groundtruth", 10, &OdometryTransformPublisher::odomCallback, this);
}

void OdometryTransformPublisher::advertiseTopics() {
  fk_odom_pub_ =
      nh_private_.advertise<nav_msgs::Odometry>("odom_repub", 10, true);
}

void OdometryTransformPublisher::advertiseTf() {
  if (config_.tf_pub_frequency > 0.0)
    tf_pub_timer_ = nh_private_.createTimer(
        ros::Duration(1.0 / config_.tf_pub_frequency),
        std::bind(&OdometryTransformPublisher::publishTfEvent, this));
}

void OdometryTransformPublisher::odomCallback(
    const nav_msgs::Odometry& odom_msg) {
  CHECK_EQ(odom_msg.child_frame_id, config_.base_link_frame);
  CHECK_EQ(odom_msg.header.frame_id, "world");

  std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
  TransformationD T_G_B;
  tf::poseMsgToKindr(odom_msg.pose.pose, &T_G_B);
  T_O_B_ = T_G_O_.inverse() * T_G_B;

  nav_msgs::Odometry odom_msg_o_b;
  odom_msg_o_b = odom_msg;
  tf::poseKindrToMsg(T_O_B_, &odom_msg_o_b.pose.pose);

  Eigen::Vector3d linear_velocity(odom_msg.twist.twist.linear.x,
                                  odom_msg.twist.twist.linear.y,
                                  odom_msg.twist.twist.linear.z);
  Eigen::Vector3d angular_velocity(odom_msg.twist.twist.angular.x,
                                   odom_msg.twist.twist.angular.y,
                                   odom_msg.twist.twist.angular.z);
  Eigen::Vector3d transformed_lin_vel(T_G_O_.getRotationMatrix().inverse() *
                                      linear_velocity);
  Eigen::Vector3d transformed_ang_vel(T_G_O_.getRotationMatrix().inverse() *
                                      angular_velocity);

  odom_msg_o_b.twist.twist.linear.x = transformed_lin_vel.x();
  odom_msg_o_b.twist.twist.linear.y = transformed_lin_vel.y();
  odom_msg_o_b.twist.twist.linear.z = transformed_lin_vel.z();

  odom_msg_o_b.twist.twist.angular.x = transformed_ang_vel.x();
  odom_msg_o_b.twist.twist.angular.y = transformed_ang_vel.y();
  odom_msg_o_b.twist.twist.angular.z = transformed_ang_vel.z();

  odom_msg_o_b.child_frame_id = config_.base_link_frame;
  odom_msg_o_b.header.frame_id = config_.odom_frame;

  odom_msg_o_b.header.stamp = ros::Time::now();
  fk_odom_pub_.publish(odom_msg_o_b);
}

void OdometryTransformPublisher::publishTfEvent() { publishTf(); }

void OdometryTransformPublisher::publishTf() {
  std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
  tf::Transform T_O_B_msg;
  tf::transformKindrToTF(T_O_B_, &T_O_B_msg);
  tf_pub_.sendTransform(tf::StampedTransform(T_O_B_msg, ros::Time::now(),
                                             config_.odom_frame,
                                             config_.base_link_frame));
}

}  // namespace coxgraph
