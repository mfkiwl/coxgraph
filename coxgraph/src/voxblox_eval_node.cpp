#include <gflags/gflags.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxgraph/tools/evaluation/map_evaluation.h>
#include <boost/filesystem.hpp>

#include <deque>
#include <memory>
#include <string>

#include "coxgraph/common.h"

// This binary evaluates a pre-built voxblox map against a provided ground
// truth dataset, provided as pointcloud, and outputs a variety of statistics.
namespace voxblox {

// Stolen from voxblox eval
class VoxbloxEvaluator {
 public:
  VoxbloxEvaluator(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  void evaluate();
  void visualize();
  bool shouldExit() const { return !bag_.isOpen(); }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Whether to do the visualizations (involves generating mesh of the TSDF
  // layer) and keep alive (for visualization) after finishing the eval.
  // Otherwise only outputs the evaluations statistics.
  bool visualize_;
  // Whether to recolor the voxels by error to the GT before generating a mesh.
  bool recolor_by_error_;
  // How to color the mesh.
  ColorMode color_mode_;
  // If visualizing, what TF frame to visualize in.
  std::string frame_id_;

  // Transformation between the ground truth dataset and the voxblox map.
  // The GT is transformed INTO the voxblox coordinate frame.
  Transformation T_V_G_;

  // Visualization publishers.
  ros::Publisher mesh_pub_;
  ros::Publisher occupancy_marker_pub_;
  ros::Publisher gt_occupancy_marker_pub_;

  // Core data to compare.
  std::shared_ptr<Layer<TsdfVoxel>> tsdf_layer_;

  // Interpolator to get the distance at the exact point in the GT.
  Interpolator<TsdfVoxel>::Ptr interpolator_;

  // Mesh visualization.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  rosbag::Bag bag_;
  std::string tsdf_topic_;
  std::string result_folder_path_;
  std::ofstream result_file_;

  voxgraph::MapEvaluation* map_evaluation_;
  ros::Publisher tsdf_pointcloud_pub_;
};

VoxbloxEvaluator::VoxbloxEvaluator(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      recolor_by_error_(false),
      frame_id_("world") {
  // Load parameters.
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("recolor_by_error", recolor_by_error_, recolor_by_error_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  nh_private_.param<std::string>("tsdf_topic", tsdf_topic_, "");

  nh_private_.param<std::string>("result_folder_path", result_folder_path_,
                                 ".");
  boost::filesystem::path p(result_folder_path_);
  p.append("voxblox_eval.txt");
  result_file_.open(p.string());
  if (!result_file_.is_open()) {
    LOG(ERROR) << "Failed to open result file " << p.string()
               << " , result will not be logged";
  } else {
    result_file_ << "evaluated_voxels,"
                    "overlapping_voxels,"
                    "non_overlapping_voxels,"
                    "ignored_voxels,"
                    "error_min,"
                    "error_max,"
                    "RMSE"
                 << std::endl;
  }

  // Load transformations.
  XmlRpc::XmlRpcValue T_V_G_xml;
  if (nh_private_.getParam("T_V_G", T_V_G_xml)) {
    kindr::minimal::xmlRpcToKindr(T_V_G_xml, &T_V_G_);
    bool invert_static_tranform = false;
    nh_private_.param("invert_T_V_G", invert_static_tranform,
                      invert_static_tranform);
    if (invert_static_tranform) {
      T_V_G_ = T_V_G_.inverse();
    }
  }

  // Load the actual map and GT.
  // Just exit if there's any issues here (this is just an evaluation node,
  // after all).
  std::string voxblox_bag_path, gt_file_path;
  CHECK(nh_private_.getParam("voxblox_bag_path", voxblox_bag_path))
      << "No file path provided for voxblox map! Set the \"voxblox_file_path\" "
         "param.";
  bag_.open(voxblox_bag_path);
  CHECK(bag_.isOpen());
  CHECK(nh_private_.getParam("gt_file_path", gt_file_path))
      << "No file path provided for ground truth pointcloud! Set the "
         "\"gt_file_path\" param.";

  map_evaluation_ = new voxgraph::MapEvaluation(nh_, gt_file_path);

  // Initialize the interpolator.
  interpolator_.reset(new Interpolator<TsdfVoxel>(tsdf_layer_.get()));

  // If doing visualizations, initialize the publishers.
  if (visualize_) {
    mesh_pub_ =
        nh_private_.advertise<visualization_msgs::MarkerArray>("mesh", 1, true);

    std::string color_mode("color");
    nh_private_.param("color_mode", color_mode, color_mode);
    if (color_mode == "color") {
      color_mode_ = ColorMode::kColor;
    } else if (color_mode == "height") {
      color_mode_ = ColorMode::kHeight;
    } else if (color_mode == "normals") {
      color_mode_ = ColorMode::kNormals;
    } else if (color_mode == "lambert") {
      color_mode_ = ColorMode::kLambert;
    } else {  // Default case is gray.
      color_mode_ = ColorMode::kGray;
    }
  }
  occupancy_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("occupied_nodes",
                                                             1, true);

  gt_occupancy_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "gt_occupied_nodes", 1, true);

  tsdf_pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_pointcloud", 1, true);

  std::cout << "Evaluating " << voxblox_bag_path << std::endl;
}

void VoxbloxEvaluator::evaluate() {
  // Display every n seconds, to make the reconstruction visible in rviz
  ros::Rate display_loop(ros::Duration(1.0));

  for (rosbag::MessageInstance const m : rosbag::View(bag_)) {
    if (tsdf_topic_.size() && m.getTopic() != tsdf_topic_) continue;

    voxblox_msgs::LayerConstPtr i = m.instantiate<voxblox_msgs::Layer>();
    if (i != nullptr) {
      display_loop.sleep();

      if (tsdf_layer_ == nullptr) {
        tsdf_layer_.reset(
            new Layer<TsdfVoxel>(i->voxel_size, i->voxels_per_side));
        interpolator_.reset(new Interpolator<TsdfVoxel>(tsdf_layer_.get()));
      }
      Layer<TsdfVoxel>::Ptr transformed_tsdf_layer(
          new Layer<TsdfVoxel>(i->voxel_size, i->voxels_per_side));
      deserializeMsgToLayer(*i, transformed_tsdf_layer.get());
      Transformation T_I_C(kindr::minimal::PositionTemplate<float>(0, 0, 0),
                           Rotation(0, 0, 0, 1));
      transformLayer(*transformed_tsdf_layer, T_I_C, tsdf_layer_.get());
      LOG(INFO) << "tsdf layer blocks: "
                << tsdf_layer_->getNumberOfAllocatedBlocks();

      auto result = map_evaluation_->evaluate(*tsdf_layer_);

      std::cout << result.reconstruction.toString();
      auto recon_rlt = result.reconstruction;

      if (result_file_.is_open()) {
        result_file_ << m.getTime() << "," << recon_rlt.num_evaluated_voxels
                     << "," << recon_rlt.num_overlapping_voxels << ","
                     << recon_rlt.num_non_overlapping_voxels << ","
                     << recon_rlt.num_ignored_voxels << ","
                     << recon_rlt.min_error << "," << recon_rlt.max_error << ","
                     << recon_rlt.rmse << std::endl;
      }

      if (visualize_) {
        visualize();
      }
    }
  }

  bag_.close();
}

void VoxbloxEvaluator::visualize() {
  // Generate the mesh.
  MeshIntegratorConfig mesh_config;
  mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_layer_.get(), mesh_layer_.get()));

  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  LOG(INFO) << "mesh size: " << mesh_layer_->getNumberOfAllocatedMeshes();

  // Publish mesh.
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0].header.frame_id = frame_id_;
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  mesh_pub_.publish(marker_array);

  visualization_msgs::MarkerArray occu_node_array;
  createOccupancyBlocksFromTsdfLayer(*tsdf_layer_, frame_id_, &occu_node_array);
  occupancy_marker_pub_.publish(occu_node_array);

  visualization_msgs::MarkerArray gt_occu_node_array;
  createOccupancyBlocksFromTsdfLayer(
      map_evaluation_->getGroundTruthMapPtr()->getTsdfMapPtr()->getTsdfLayer(),
      frame_id_, &gt_occu_node_array);
  occupancy_marker_pub_.publish(gt_occu_node_array);

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(*tsdf_layer_, &pointcloud);

  pointcloud.header.frame_id = frame_id_;
  tsdf_pointcloud_pub_.publish(pointcloud);

  std::cout << "Finished visualizing.\n";
}

}  // namespace voxblox
int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Register with ROS master
  ros::init(argc, argv, "voxblox_eval");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::VoxbloxEvaluator eval(nh, nh_private);
  eval.evaluate();

  if (!eval.shouldExit()) {
    ros::spin();
  }
  return 0;

  // Exit normally
  return 0;
}
