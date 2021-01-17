#ifndef COXGRAPH_SIM_ODOMETRY_TRANSFORM_PUBLISHER_H_
#define COXGRAPH_SIM_ODOMETRY_TRANSFORM_PUBLISHER_H_

#include <coxgraph/common.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <mutex>
#include <string>
#include <vector>

namespace coxgraph {

class OdometryTransformPublisher {
 public:
  struct Config {
    Config()
        : odom_frame("odom"),
          base_link_frame("base_link"),
          tf_pub_frequency(0.0) {}
    std::string odom_frame;
    std::string base_link_frame;
    float tf_pub_frequency;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "OdometryTransformPublisher using Config:" << std::endl
        << "  Odom Frame: " << v.odom_frame << std::endl
        << "  Base Link Frame: " << v.base_link_frame << std::endl
        << "  Tf Pub Frequency: " << v.tf_pub_frequency << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  OdometryTransformPublisher(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
      : OdometryTransformPublisher(nh, nh_private,
                                   getConfigFromRosParam(nh_private)) {}

  OdometryTransformPublisher(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private,
                             const Config& config)
      : nh_(nh), nh_private_(nh_private), config_(config), initialized_(false) {
    LOG(INFO) << config_;

    subscribeToTopics();
    advertiseTopics();
    advertiseTf();
  }

  ~OdometryTransformPublisher() = default;

 private:
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseTf();
  void gtOdomCallback(const nav_msgs::Odometry& odom_msg);
  void vioImgCallback(const sensor_msgs::ImageConstPtr& odom_msg);
  void vioOdomCallback(const nav_msgs::Odometry& odom_msg);
  void initTGO();
  void publishTfEvent();
  void publishTf();

  Config config_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber gt_odom_sub_;
  ros::Subscriber vio_img_sub_;
  ros::Subscriber vio_odom_sub_;
  ros::Publisher fk_odom_pub_;
  ros::Timer tf_pub_timer_;
  tf::TransformBroadcaster tf_pub_;

  bool initialized_;
  TransformationD T_G_O_;
  TransformationD T_O_B_;

  std::mutex pose_update_mutex_;
};

}  // namespace coxgraph

#endif  // COXGRAPH_SIM_ODOMETRY_TRANSFORM_PUBLISHER_H_
