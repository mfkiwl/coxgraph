#ifndef COXGRAPH_SERVER_DATA_SERVER_PROJECTED_MAP_SERVER_H_
#define COXGRAPH_SERVER_DATA_SERVER_PROJECTED_MAP_SERVER_H_

#include <voxblox_msgs/Layer.h>

#include <string>

#include "coxgraph/server/submap_collection.h"

namespace coxgraph {
namespace server {
class ProjectedMapServer {
 public:
  explicit ProjectedMapServer(ros::NodeHandle nh_private)
      : prev_submap_number_(0) {
    projected_tsdf_map_pub_ = nh_private.advertise<voxblox_msgs::Layer>(
        "projected_map_tsdf", 1, true);
  }
  ~ProjectedMapServer() = default;

  void publishProjectedMap(const SubmapCollection::Ptr& submap_collection,
                           const std::string& world_frame,
                           const ros::Time& timestamp) {
    // Only publish if there are subscribers
    publishProjectedMap(submap_collection, world_frame, timestamp,
                        projected_tsdf_map_pub_);
  }

  void publishProjectedMap(const SubmapCollection::Ptr& submap_collection,
                           const std::string& world_frame,
                           const ros::Time& timestamp,
                           const ros::Publisher& projected_map_publisher) {
    if (projected_tsdf_map_pub_.getNumSubscribers() > 0 &&
        submap_collection->getSubmapPtrs().size() +
                submap_collection->getNumberOfRecoveredSubmaps() >
            prev_submap_number_) {
      prev_submap_number_ = submap_collection->getSubmapPtrs().size() +
                            submap_collection->getNumberOfRecoveredSubmaps();
      LOG(INFO) << "Publishing projected map";

      voxblox_msgs::Layer projected_tsdf_layer_msg;

      voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
          submap_collection->getProjectedMap()->getTsdfLayer(), false,
          &projected_tsdf_layer_msg);
      projected_tsdf_layer_msg.action = voxblox_msgs::Layer::ACTION_RESET;
      projected_tsdf_layer_msg.frame_id = world_frame;

      projected_map_publisher.publish(projected_tsdf_layer_msg);
      LOG(INFO) << "Published projected map";
    }
  }

 private:
  ros::Publisher projected_tsdf_map_pub_;

  int8_t prev_submap_number_;
};
}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_DATA_SERVER_PROJECTED_MAP_SERVER_H_
