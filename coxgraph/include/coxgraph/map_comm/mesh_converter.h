#ifndef COXGRAPH_MAP_COMM_MESH_CONVERTER_H_
#define COXGRAPH_MAP_COMM_MESH_CONVERTER_H_

#include <coxgraph/common.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <voxblox/core/common.h>
#include <voxblox_msgs/Mesh.h>

#include <memory>
#include <vector>

namespace voxblox {
typedef std::shared_ptr<Pointcloud> PointcloudPtr;
class MeshConverter {
 public:
  struct Config {
    float voxel_size = 0.05;
    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Mesh Converter using Config:" << std::endl
        << "  voxel_size: " << v.voxel_size << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<MeshConverter> Ptr;
  typedef kindr::minimal::PositionTemplate<FloatingPoint> Position;

  explicit MeshConverter(const ros::NodeHandle nh_private)
      : config_(getConfigFromRosParam(nh_private)) {
    LOG(INFO) << config_;
  }

  ~MeshConverter() = default;

  inline void setMesh(const voxblox_msgs::Mesh& mesh) {
    LOG_IF(ERROR, mesh.trajectory.poses.empty())
        << "received a mesh with empty trajectory";
    mesh_ = mesh;
    setTrajectory(mesh.trajectory);
  }

  inline void setTrajectory(const nav_msgs::Path& path) {
    // Clear point clouds
    pointcloud_.clear();
    T_G_C_.clear();
    for (auto const& pose : path.poses) {
      coxgraph::TransformationD T_Sm_C;
      tf::poseMsgToKindr(pose.pose, &T_Sm_C);
      T_G_C_.emplace_back(T_Sm_C.cast<FloatingPoint>());
      pointcloud_.emplace_back(new Pointcloud());
    }
  }

  bool convertToPointCloud();

  bool getNextPointcloud(int* i, Transformation* T_G_C,
                         Pointcloud* pointcloud) {
    if (*i >= T_G_C_.size()) return false;
    CHECK(pointcloud);
    *T_G_C = T_G_C_[*i];
    // Mesh points are T_G_P
    Transformation T_Mesh_Sdf(Quaternion(0.5, -0.5, 0.5, -0.5).inverse(),
                              Transformation::Position());
    transformPointcloud(T_G_C->inverse(), *(pointcloud_[*i]), pointcloud);
    (*i)++;
    return true;
  }

  auto getCombinedPointcloud() {
    Pointcloud combined_pointcloud;
    for (auto const& pointcloud : pointcloud_) {
      combined_pointcloud.insert(combined_pointcloud.end(), pointcloud->begin(),
                                 pointcloud->end());
    }
    return combined_pointcloud;
  }

  Pointcloud interpolateTriangle(const Pointcloud& triangle) {
    timing::Timer interpolate_timer("interpolate_triangle");
    // TODO(mikexyl): a really stupid interpolation, but should do the work
    Pointcloud interp_pts_e01, interp_pts_e02, interp_pts_e12;
    Point p0 = triangle[0], p1 = triangle[1], p2 = triangle[2];

    Point t_p0_p1 = p1 - p0;
    Point t_p0_p2 = p2 - p0;
    Point t_p1_p2 = p2 - p1;

    for (float dist = config_.voxel_size; dist < t_p0_p1.norm();
         dist += config_.voxel_size) {
      interp_pts_e01.emplace_back(p0 + t_p0_p1 / t_p0_p1.norm() * dist);
    }
    for (float dist = config_.voxel_size; dist < t_p0_p1.norm();
         dist += config_.voxel_size) {
      interp_pts_e02.emplace_back(p0 + t_p0_p2 / t_p0_p2.norm() * dist);
    }
    for (float dist = config_.voxel_size; dist < t_p1_p2.norm();
         dist += config_.voxel_size) {
      interp_pts_e12.emplace_back(p1 + t_p1_p2 / t_p1_p2.norm() * dist);
    }

    //  for (auto const& pt : interp_pts_e01) {
    //    auto t = p2 - pt;
    //    for (float dist = config_.voxel_size; dist < t.norm();
    //         dist += config_.voxel_size) {
    //      interp_pts_e12.emplace_back(pt + t / t.norm() * dist);
    //    }
    //  }

    interp_pts_e01.emplace_back((p0 + p1 + p2) / 3);

    interp_pts_e01.insert(interp_pts_e01.end(), interp_pts_e02.begin(),
                          interp_pts_e02.end());
    interp_pts_e01.insert(interp_pts_e01.end(), interp_pts_e12.begin(),
                          interp_pts_e12.end());

    interpolate_timer.Stop();
    return interp_pts_e01;
  }

 private:
  Config config_;

  voxblox_msgs::Mesh mesh_;
  AlignedVector<Transformation> T_G_C_;

  std::vector<PointcloudPtr> pointcloud_;
};
}  // namespace voxblox

#endif  //  COXGRAPH_MAP_COMM_MESH_CONVERTER_H_
