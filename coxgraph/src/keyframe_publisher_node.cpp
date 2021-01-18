#include <glog/logging.h>
#include <ros/ros.h>

#include "coxgraph/client/keyframe_publisher.h"

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Register with ROS master
  ros::init(argc, argv, "tsdf_recover");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the mapper
  coxgraph::client::KeyframePublisher keyframe_publiser(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
