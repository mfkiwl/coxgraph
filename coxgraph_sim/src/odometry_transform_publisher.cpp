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
  return config;
}

void OdometryTransformPublisher::subscribeToTopics() {
  gt_odom_sub_ = nh_private_.subscribe(
      "odom_gt", 10, &OdometryTransformPublisher::gtOdomCallback, this);
  vio_img_sub_ = nh_private_.subscribe(
      "vio_img", 1, &OdometryTransformPublisher::vioImgCallback, this);
  vio_odom_sub_ = nh_private_.subscribe(
      "vio_odom", 1, &OdometryTransformPublisher::vioOdomCallback, this);
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

void OdometryTransformPublisher::gtOdomCallback(
    const nav_msgs::Odometry& odom_msg) {
  CHECK_EQ(odom_msg.child_frame_id, config_.base_link_frame);
  CHECK_EQ(odom_msg.header.frame_id, "world");

  std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
  TransformationD T_G_B;
  tf::poseMsgToKindr(odom_msg.pose.pose, &T_G_B);
  T_O_B_ = T_G_O_.inverse() * T_G_B;
  if (!initialized_) return;

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

void OdometryTransformPublisher::vioImgCallback(
    const sensor_msgs::ImageConstPtr& odom_msg) {
  initTGO();
}

void OdometryTransformPublisher::vioOdomCallback(
    const nav_msgs::Odometry& odom_msg) {
  initTGO();
}

void OdometryTransformPublisher::initTGO() {
  if (!initialized_) {
    CHECK(T_G_O_ == TransformationD());

    // Before T_G_O is initialized, T_O_B_ = T_G_B_;
    T_G_O_ = T_O_B_;
    T_O_B_ = T_G_O_.inverse() * T_O_B_;

    initialized_ = true;

    ROS_INFO_STREAM("Set T_G_O to " << T_G_O_);
  }
  vio_img_sub_.shutdown();
  vio_odom_sub_.shutdown();
}

void OdometryTransformPublisher::publishTfEvent() { publishTf(); }

void OdometryTransformPublisher::publishTf() {
  if (!initialized_) return;
  std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);

  tf::Transform T_O_B_msg;
  tf::transformKindrToTF(T_O_B_, &T_O_B_msg);
  tf_pub_.sendTransform(tf::StampedTransform(T_O_B_msg, ros::Time::now(),
                                             config_.odom_frame,
                                             config_.base_link_frame));
}

}  // namespace coxgraph
