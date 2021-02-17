#ifndef COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
#define COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_

#include <Open3D/Visualization/Visualizer/Visualizer.h>
#include <coxgraph_msgs/ClientSubmap.h>
#include <coxgraph_msgs/ClientSubmapSrv.h>
#include <coxgraph_msgs/PoseHistorySrv.h>
#include <coxgraph_msgs/SubmapsSrv.h>
#include <coxgraph_msgs/TimeLine.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxgraph/frontend/voxgraph_mapper.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "coxgraph/client/map_server.h"
#include "coxgraph/common.h"
#include "coxgraph/utils/msg_converter.h"

namespace coxgraph {
class CoxgraphClient : public voxgraph::VoxgraphMapper {
 public:
  CoxgraphClient(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : VoxgraphMapper(nh, nh_private),
        recover_mode_(true),
        vis_combined_o3d_mesh_(false) {
    int client_id;
    nh_private.param<int>("client_id", client_id, -1);
    client_id_ = static_cast<CliId>(client_id);

    nh_private.param("recover_mode", recover_mode_, recover_mode_);
    nh_private.param("vis_combined_o3d_mesh", vis_combined_o3d_mesh_,
                     vis_combined_o3d_mesh_);
    if (vis_combined_o3d_mesh_) {
      o3d_vis_ = new open3d::visualization::Visualizer();
      o3d_vis_->CreateVisualizerWindow("client_" + std::to_string(client_id_));
    }
    subscribeToClientTopics();
    advertiseClientTopics();
    advertiseClientServices();
    if (client_id_ < 0) {
      LOG(FATAL) << "Invalid Client Id " << client_id_;
    } else {
      LOG(INFO) << "Started Coxgraph Client " << client_id_;
    }
    log_prefix_ = "Client " + std::to_string(client_id_) + ": ";

    map_server_.reset(new MapServer(nh_, nh_private_, client_id_,
                                    submap_config_, frame_names_,
                                    submap_collection_ptr_));
  }

  ~CoxgraphClient() = default;

  inline const CliId& getClientId() const { return client_id_; }

  void subscribeToClientTopics();
  void advertiseClientTopics();
  void advertiseClientServices();

  bool getClientSubmapCallback(
      coxgraph_msgs::ClientSubmapSrv::Request& request,     // NOLINT
      coxgraph_msgs::ClientSubmapSrv::Response& response);  // NOLINT

  bool getAllClientSubmapsCallback(
      coxgraph_msgs::SubmapsSrv::Request& request,     // NOLINT
      coxgraph_msgs::SubmapsSrv::Response& response);  // NOLINT

  bool getPoseHistory(
      coxgraph_msgs::PoseHistorySrv::Request& request,      // NOLINT
      coxgraph_msgs::PoseHistorySrv::Response& response) {  // NOLINT
    response.pose_history.pose_history =
        submap_collection_ptr_->getPoseHistory();
    boost::filesystem::path p(request.file_path);
    p.append("coxgraph_client_traj_" + std::to_string(client_id_) + ".txt");
    savePoseHistory(p.string());
    return true;
  }

  bool submapCallback(const voxblox_msgs::LayerWithTrajectory& submap_msg,
                      bool transform_layer) override;

 private:
  using VoxgraphMapper = voxgraph::VoxgraphMapper;
  using MapServer = client::MapServer;
  typedef std::map<CliSmId, Transformation> SmIdTfMap;

  void publishTimeLine();
  void publishMapPoseUpdates();
  void publishSubmapPoseTFs() override;

  void savePoseHistory(std::string file_path);

  CliId client_id_;
  std::string log_prefix_;

  ros::Publisher time_line_pub_;
  ros::Publisher map_pose_pub_;
  ros::Publisher submap_mesh_pub_;
  ros::ServiceServer get_client_submap_srv_;
  ros::ServiceServer get_all_client_submaps_srv_;
  ros::ServiceServer get_pose_history_srv_;

  SmIdTfMap ser_sm_id_pose_map_;

  std::timed_mutex submap_proc_mutex_;

  MapServer::Ptr map_server_;

  bool recover_mode_;
  typedef message_filters::sync_policies::ApproximateTime<
      voxblox_msgs::LayerWithTrajectory, sensor_msgs::PointCloud2>
      sync_pol;
  message_filters::Synchronizer<sync_pol>* synchronizer_;
  message_filters::Subscriber<voxblox_msgs::LayerWithTrajectory>*
      submap_sync_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>*
      mesh_pointcloud_sync_sub_;

  // T_Submap_Mesh
  std::map<CliSmId, std::shared_ptr<open3d::geometry::TriangleMesh>>
      mesh_collection_;
  bool new_submap_added_;
  open3d::visualization::Visualizer* o3d_vis_;
  void submapMeshCallback(
      const voxblox_msgs::LayerWithTrajectoryConstPtr& layer_msg,
      const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg) {
    if (submapCallback(*layer_msg, true)) {
      auto o3d_mesh = utils::o3dMeshFromMsg(*pointcloud_msg);
      if (o3d_mesh != nullptr && vis_combined_o3d_mesh_) {
        auto T_G_Sm = submap_collection_ptr_->getActiveSubmapPose();
        // o3d_mesh: T_G_Mesh
        o3d_mesh->Transform(
            T_G_Sm.cast<double>().inverse().getTransformationMatrix());
        mesh_collection_.emplace(submap_collection_ptr_->getActiveSubmapID(),
                                 o3d_mesh);
        o3d_vis_->AddGeometry(o3d_mesh);
        o3dMeshVisualize();
      }
    }
  }

  bool vis_combined_o3d_mesh_;
  ros::Timer o3d_mesh_timer_;
  void o3dMeshVisualizeEvent(const ros::TimerEvent& /*event*/) {
    o3dMeshVisualize();
  }

  void o3dMeshVisualize() {
    for (auto const& kv : mesh_collection_) {
      Transformation T_G_Sm;
      submap_collection_ptr_->getSubmapPose(kv.first, &T_G_Sm);
      // Transform to T_G_Mesh
      kv.second->Transform(T_G_Sm.cast<double>().getTransformationMatrix());
      kv.second->ComputeTriangleNormals();
      o3d_vis_->UpdateGeometry(kv.second);
    }
    o3d_vis_->PollEvents();
    o3d_vis_->UpdateRender();

    for (auto const& kv : mesh_collection_) {
      Transformation T_G_Sm;
      submap_collection_ptr_->getSubmapPose(kv.first, &T_G_Sm);
      // Transform mesh back to T_Submap_Mesh
      kv.second->Transform(
          T_G_Sm.cast<double>().inverse().getTransformationMatrix());
    }
  }
};

}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
