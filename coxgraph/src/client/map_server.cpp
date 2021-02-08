#include "coxgraph/client/map_server.h"

#include <coxgraph_msgs/BoundingBox.h>
#include <coxgraph_msgs/MeshWithTrajectory.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxblox_msgs/MultiMesh.h>

#include <memory>
#include <string>

namespace coxgraph {
namespace client {

MapServer::Config MapServer::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  return config;
}

void MapServer::subscribeToTopics() {}

void MapServer::advertiseTopics() {}

void MapServer::subscribeToServices() {}

void MapServer::advertiseServices() {}

void MapServer::startTimers() {}

}  // namespace client
}  // namespace coxgraph
