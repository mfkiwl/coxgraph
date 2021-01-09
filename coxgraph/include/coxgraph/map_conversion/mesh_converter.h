#ifndef COXGRAPH_MAP_CONVERSION_MESH_CONVERTER_H_
#define COXGRAPH_MAP_CONVERSION_MESH_CONVERTER_H_

#include <coxgraph/common.h>
#include <coxgraph_msgs/MeshWithTrajectory.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <voxblox/core/common.h>

#include <memory>

namespace voxblox {
class MeshConverter {
 public:
  struct FOV {
    float max_ray_length_m;
    float min_ray_length_m;
    float vertical_degree;
    float horizontal_degree;

    static FOV getFOVFromRosParam(const ros::NodeHandle& nh_private) {
      FOV fov;
      nh_private.param<float>("max_ray_length_m", fov.max_ray_length_m, 90);
      nh_private.param<float>("min_ray_length_m", fov.min_ray_length_m, 90);
      nh_private.param<float>("fov_horizontal_degree", fov.horizontal_degree,
                              90);
      nh_private.param<float>("fov_vertical_degree", fov.vertical_degree, 60);
      return fov;
    }
  };

  struct Config {
    Config() : interpolate_ratio(0.0) {}
    float interpolate_ratio;
    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Mesh Converter using Config:" << std::endl
        << std::endl
        << "  Interpolate Ratio: " << v.interpolate_ratio << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<MeshConverter> Ptr;

  typedef kindr::minimal::PositionTemplate<FloatingPoint> Position;

  explicit MeshConverter(const ros::NodeHandle nh_private)
      : config_(getConfigFromRosParam(nh_private)),
        fov_(FOV::getFOVFromRosParam(nh_private)),
        pointcloud_(new Pointcloud()) {
    LOG(INFO) << config_;
    LOG(INFO) << "fov: " << fov_.horizontal_degree << " "
              << fov_.vertical_degree << " " << fov_.max_ray_length_m << " "
              << fov_.min_ray_length_m;
  }
  ~MeshConverter() = default;

  inline void setMesh(const voxblox_msgs::Mesh& mesh) { mesh_ = mesh; }

  inline void setTrajectory(const nav_msgs::Path& path) {
    for (auto const& pose : path.poses) {
      coxgraph::TransformationD T_Sm_C;
      tf::poseMsgToKindr(pose.pose, &T_Sm_C);
      T_Sm_C_.emplace(T_Sm_C.cast<FloatingPoint>());
    }
  }

  bool convertToPointCloud();

  bool getPointcloudInNextFOV(Transformation* T_Sm_C,
                              Pointcloud* pointcloud_in_fov_C) {
    if (T_Sm_C_.empty()) return false;

    timing::Timer get_pointcloud_in_fov_timer("get_pointcloud_in_fov");
    CHECK(pointcloud_in_fov_C != nullptr);
    pointcloud_in_fov_C->clear();
    *T_Sm_C = T_Sm_C_.front();
    T_Sm_C_.pop();

    // TODO(mikexyl): verify the frame transform
    for (auto const& point : *pointcloud_) {
      // Roughly filter out points outside fov box
      if (fabs(point[0] - T_Sm_C->getPosition().x()) > fov_.max_ray_length_m ||
          fabs(point[0] - T_Sm_C->getPosition().x()) < fov_.min_ray_length_m ||
          fabs(point[1] - T_Sm_C->getPosition().y()) > fov_.max_ray_length_m ||
          fabs(point[1] - T_Sm_C->getPosition().y()) < fov_.min_ray_length_m ||
          fabs(point[2] - T_Sm_C->getPosition().z()) > fov_.max_ray_length_m ||
          fabs(point[2] - T_Sm_C->getPosition().z()) < fov_.min_ray_length_m)
        continue;

      Transformation T_Sm_P(Position(point[0], point[1], point[2]),
                            Quaternion().setIdentity());
      Transformation T_C_P = T_Sm_C->inverse() * T_Sm_P;
      float x = T_C_P.getPosition().x();
      float y = T_C_P.getPosition().y();
      float z = T_C_P.getPosition().z();

      float dist = T_C_P.getPosition().norm();
      if (dist > fov_.max_ray_length_m || dist < fov_.min_ray_length_m) {
        continue;
      }

      if (fabs(atan2(x, z)) / 3.14159 * 180 > fov_.horizontal_degree) continue;
      if (fabs(atan2(y, z)) / 3.14159 * 180 > fov_.vertical_degree) continue;

      pointcloud_in_fov_C->emplace_back(x, y, z);
    }
    get_pointcloud_in_fov_timer.Stop();
    return true;
  }

  const std::shared_ptr<Pointcloud>& getPointcloud() const {
    return pointcloud_;
  }

 private:
  Config config_;
  FOV fov_;

  voxblox_msgs::Mesh mesh_;
  AlignedQueue<Transformation> T_Sm_C_;

  std::shared_ptr<Pointcloud> pointcloud_;
};
}  // namespace voxblox

#endif  //  COXGRAPH_MAP_CONVERSION_MESH_CONVERTER_H_
