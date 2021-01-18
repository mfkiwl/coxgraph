#include <glog/logging.h>
#include <ros/ros.h>

#include "coxgraph/client/keyframe_tracker.h"

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
  coxgraph::client::KeyframeTracker keyframe_tracker(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
