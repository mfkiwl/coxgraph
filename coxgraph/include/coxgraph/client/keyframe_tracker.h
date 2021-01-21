#ifndef COXGRAPH_CLIENT_KEYFRAME_TRACKER_H_
#define COXGRAPH_CLIENT_KEYFRAME_TRACKER_H_

#include <brisk/brisk.h>
#include <comm_msgs/keyframe.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_sequencer.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "DBoW2/FBRISK.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "coxgraph/client/keyframe.h"
#include "coxgraph/common.h"

namespace coxgraph {
namespace client {

class KeyframeTracker {
 public:
  struct Config {
    Config()
        : min_dist_m(0.1),
          min_yaw_rad(0.1),
          min_local_score(0.015),
          min_kf_n(10),
          voc_file(""),
          camera_k(),
          camera_d(),
          sensor(static_cast<SensorType>(0)),
          depth_factor(1.0),
          window_size(3),
          publish_landmarks(true),
          map_frame("map"),
          publish_point_tracking_image(true) {}
    std::vector<double> camera_k;
    std::vector<double> camera_d;
    float min_dist_m;
    float min_yaw_rad;
    // TODO(mikexyl): should rename it to max_local_score
    float min_local_score;
    int min_kf_n;
    std::string voc_file;
    int sensor;
    float depth_factor;
    int window_size;
    bool publish_landmarks;
    std::string map_frame;
    bool publish_point_tracking_image;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Keyframe Publisher using Config:" << std::endl
        << "  Min KF Dist: " << v.min_dist_m << " m" << std::endl
        << "  Min Yaw Diff: " << v.min_yaw_rad << " rad" << std::endl
        << "  Min KF Every: " << v.min_kf_n << " frames" << std::endl
        << "  Min Local Score: " << v.min_local_score << std::endl
        << "  Sensor Type: " << v.sensor << std::endl
        << "  Depth Factor: " << v.depth_factor << std::endl
        << "  Window Size: " << v.window_size << std::endl
        << "  Publsih Landmark Pointcloud: " << v.publish_landmarks << std::endl
        << "  Map Frame: " << v.map_frame << std::endl
        << "  Publish Point Tracking Image: " << v.publish_point_tracking_image
        << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private) {
    Config config;
    nh_private.param<float>("min_dist_m", config.min_dist_m, config.min_dist_m);
    nh_private.param<float>("min_yaw_rad", config.min_yaw_rad,
                            config.min_yaw_rad);
    nh_private.param<int>("min_kf_n", config.min_kf_n, config.min_kf_n);
    nh_private.param<std::string>("voc_file", config.voc_file, config.voc_file);
    nh_private.param<float>("min_local_score", config.min_local_score,
                            config.min_local_score);
    nh_private.param<std::vector<double>>("camera_k", config.camera_k,
                                          config.camera_k);
    nh_private.param<std::vector<double>>("camera_d", config.camera_d,
                                          config.camera_d);
    nh_private.param<int>("sensor", config.sensor, config.sensor);
    nh_private.param<float>("depth_factor", config.depth_factor,
                            config.depth_factor);
    nh_private.param<int>("window_size", config.window_size,
                          config.window_size);
    nh_private.param<bool>("publish_landmarks", config.publish_landmarks,
                           config.publish_landmarks);
    nh_private.param<std::string>("map_frame", config.map_frame,
                                  config.map_frame);
    nh_private.param<bool>("publish_point_tracking_image",
                           config.publish_point_tracking_image,
                           config.publish_point_tracking_image);
    return config;
  }

  KeyframeTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        config_(getConfigFromRosParam(nh_private)),
        kf_index_(0),
        n_since_last_kf_(0),
        tracking_config_(getTrackingConfigFromRosParam(
            ros::NodeHandle(nh_private, "tracking"))),
        last_kf_(nullptr),
        kf_pose_map_(new std::map<int, TransformationD>()) {
    LOG(INFO) << config_;
    LOG(INFO) << tracking_config_;
    nh_private_.param<int>("client_id", cid_, cid_);
    if (config_.voc_file.empty()) {
      LOG(FATAL) << "No voc file path given";
    }
    voc_.reset(new BRISKVocabulary(config_.voc_file));
    CHECK(!voc_->empty()) << "voc file invalid";
    brisk_extractor_.reset(new brisk::BriskDescriptorExtractor(
        true, false, brisk::BriskDescriptorExtractor::briskV2));

    if (config_.camera_k.size() && config_.camera_d.size())
      setCameraInfo(config_.camera_k, config_.camera_d);

    landmark_pc_.header.frame_id = config_.map_frame;

    subscribeToTopics();
    advertiseTopics();

    CHECK_EQ(config_.window_size, 3) << "Only window size 3 supported";

    LOG(INFO) << "Keyframe Tracker started!";
  }

  ~KeyframeTracker() = default;

  void subscribeToTopics() {
    rgb_subscriber_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        nh_private_, "rgb/image_raw", 1));
    odom_subscriber_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh_private_, "odometry", 1));

    if (config_.sensor == SensorType::RGBD) {
      depth_subscriber_.reset(
          new message_filters::Subscriber<sensor_msgs::Image>(
              nh_private_, "depth/image_raw", 1));
      rgbd_sync_.reset(new message_filters::Synchronizer<rgbd_odom_sync_pol_>(
          rgbd_odom_sync_pol_(10), *rgb_subscriber_, *depth_subscriber_,
          *odom_subscriber_));
      rgbd_sync_->registerCallback(
          boost::bind(&KeyframeTracker::rgbdImageCallback, this, _1, _2, _3));
    } else if (config_.sensor == SensorType::MONOCULAR) {
      rgb_sync_.reset(new message_filters::Synchronizer<rgb_odom_sync_pol_>(
          rgb_odom_sync_pol_(10), *rgb_subscriber_, *odom_subscriber_));
      rgb_sync_->registerCallback(
          boost::bind(&KeyframeTracker::rgbImageCallback, this, _1, _2));
    }

    if (config_.camera_k.empty() || config_.camera_d.empty())
      camera_info_sub_ = nh_private_.subscribe(
          "camera_info", 10, &KeyframeTracker::cameraInfoCallback, this);
  }

  void advertiseTopics() {
    kf_pub_ = nh_private_.advertise<comm_msgs::keyframe>("keyframe", 10, true);
    landmark_pc_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
        "landmarks", 10, true);
    point_tracking_image_pub_ = nh_private_.advertise<sensor_msgs::Image>(
        "point_tracking_image", 10, true);
  }

 private:
  int cid_;
  enum SensorType { MONOCULAR = 0, STEREO, RGBD };
  Config config_;
  TrackingConfig tracking_config_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>
      rgbd_odom_sync_pol_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          nav_msgs::Odometry>
      rgb_odom_sync_pol_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
      rgb_subscriber_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
      depth_subscriber_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>
      odom_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<rgbd_odom_sync_pol_>>
      rgbd_sync_;
  std::shared_ptr<message_filters::Synchronizer<rgb_odom_sync_pol_>> rgb_sync_;

  std::shared_ptr<BRISKVocabulary> voc_;
  std::shared_ptr<brisk::BriskDescriptorExtractor> brisk_extractor_;
  std::vector<cv::Point2f> prev_pts_, cur_pts_, forw_pts_;

  void process(const ros::Time& timestamp, const TransformationD& T_G_C,
               const cv::Mat& rgb_image, const cv::Mat& depth_image) {
    TransformationD T_lastKF_C = T_G_lastKF_.inverse() * T_G_C;

    // TODO(mikexyl): forgot how to convert kindr quaternion to rpy
    if (T_lastKF_C.getPosition().norm() > config_.min_dist_m ||
        T_lastKF_C.getRotation().norm() > config_.min_yaw_rad) {
      Keyframe::Ptr new_kf(new Keyframe(timestamp, kf_index_++, cid_, rgb_image,
                                        T_G_C, kf_pose_map_, last_kf_,
                                        brisk_extractor_, k_, dist_coef_,
                                        tracking_config_, depth_image));
      kf_pose_map_->emplace(new_kf->getID(), T_G_C);
      last_kf_ = new_kf;

      pcl::PointCloud<pcl::PointXYZ> landmark_pc;

      if (n_since_last_kf_++ > config_.min_kf_n &&
          voc_->score(new_kf->getBowVec(), bow_vec_lastKF_) <
              config_.min_local_score) {
        new_kf->trackKeypointsRef(true, &landmark_pc);

        if (new_kf->goodToPublish()) kf_pub_.publish(new_kf->asKeyframeMsg());

        // TODO(mikexyl): no bow vec computed for now
        bow_vec_lastKF_ = new_kf->getBowVec();
        n_since_last_kf_ = 0;
        T_G_lastKF_ = T_G_C;
      } else {
        new_kf->trackKeypointsRef(true, &landmark_pc);
      }

      if (config_.publish_landmarks &&
          landmark_pc_pub_.getNumSubscribers() > 0) {
        landmark_pc_.insert(landmark_pc_.end(), landmark_pc.begin(),
                            landmark_pc.end());
        landmark_pc_pub_.publish(landmark_pc_);
      }
    }

    ROS_INFO_STREAM_ONCE("Got first image and odom message");
  }

  void rgbImageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                        const nav_msgs::OdometryConstPtr& odom_msg) {
    TransformationD T_G_C;
    tf::poseMsgToKindr(odom_msg->pose.pose, &T_G_C);

    cv_bridge::CvImageConstPtr rgb_image_ptr;
    try {
      rgb_image_ptr = cv_bridge::toCvShare(rgb_msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    process(rgb_msg->header.stamp, T_G_C, rgb_image_ptr->image, cv::Mat());
  }

  void rgbdImageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                         const sensor_msgs::ImageConstPtr& depth_msg,
                         const nav_msgs::OdometryConstPtr& odom_msg) {
    TransformationD T_G_C;
    tf::poseMsgToKindr(odom_msg->pose.pose, &T_G_C);

    cv_bridge::CvImageConstPtr rgb_image_ptr;
    try {
      rgb_image_ptr = cv_bridge::toCvShare(rgb_msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImageConstPtr depth_image_ptr;
    try {
      depth_image_ptr = cv_bridge::toCvShare(rgb_msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth_image = depth_image_ptr->image;
    if ((fabs(config_.depth_factor - 1.0f) > 1e-5) ||
        depth_image.type() != CV_32F)
      depth_image.convertTo(depth_image, CV_32F, config_.depth_factor);
    process(rgb_msg->header.stamp, T_G_C, rgb_image_ptr->image, depth_image);
  }

  ros::Subscriber camera_info_sub_;
  cv::Mat k_, dist_coef_;
  void cameraInfoCallback(const sensor_msgs::CameraInfo& camera_info) {
    setCameraInfo(
        std::vector<double>(camera_info.K.begin(), camera_info.K.end()),
        std::vector<double>(camera_info.D.begin(), camera_info.D.end()));
  }

  void setCameraInfo(const std::vector<double>& k_vec,
                     const std::vector<double>& d_vec) {
    cv::Mat k(3, 3, CV_64F), dist_coef(1, d_vec.size(), CV_64F);
    k.setTo(0);
    k.at<double>(0, 0) = k_vec[0];
    k.at<double>(1, 1) = k_vec[4];
    k.at<double>(0, 2) = k_vec[2];
    k.at<double>(1, 2) = k_vec[5];
    k.at<double>(2, 2) = 1.0;
    k.copyTo(k_);
    LOG(INFO) << "Set Camera Matrix to:" << std::endl << k_;

    dist_coef.setTo(0);
    for (int i = 0; i < d_vec.size(); i++) {
      dist_coef.at<double>(i) = d_vec[i];
    }
    dist_coef.copyTo(dist_coef_);
    LOG(INFO) << "Set Distortion Parameters to:" << std::endl << dist_coef_;
  }

  enum TrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
  };
  TrackingState state_;

  int kf_index_;
  int n_since_last_kf_;
  TransformationD T_G_lastKF_;
  DBoW2::BowVector bow_vec_lastKF_;
  Keyframe::Ptr last_kf_;
  std::shared_ptr<KfPoseMap> kf_pose_map_;
  ros::Publisher kf_pub_;
  ros::Publisher landmark_pc_pub_;
  ros::Publisher point_tracking_image_pub_;
  pcl::PointCloud<pcl::PointXYZ> landmark_pc_;

  constexpr static size_t kOdomQueueSize = 10;
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_KEYFRAME_TRACKER_H_
