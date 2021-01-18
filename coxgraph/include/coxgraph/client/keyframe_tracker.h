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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <deque>
#include <vector>

#include "DBoW2/FBRISK.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "coxgraph/common.h"

namespace coxgraph {
namespace client {

// Borrowed from vins_client_server
class Keyframe {
 public:
  Keyframe(const ros::Time& timestamp, int index, CliId cid,
           const cv::Mat& rgb_image, const cv::Mat& depth_image,
           const TransformationD T_G_C,
           const brisk::BriskDescriptorExtractor& brisk_extractor)
      : timestamp_(timestamp),
        index_(index),
        cid_(cid),
        rgb_image_(rgb_image),
        depth_image_(depth_image),
        T_G_C_(T_G_C) {
    computeBRIEFPoint(brisk_extractor);
  }

  virtual ~Keyframe() = default;

  void computeBRIEFPoint(
      const brisk::BriskDescriptorExtractor& brisk_extractor) {
    //  brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>
    //      brisk_detector(0, 10.0, 300.0, 200);
    keypoints_.clear();
    vector<cv::Point2f> tmp_pts;
    cv::goodFeaturesToTrack(rgb_image_, tmp_pts, 500, 0.01, 10);
    for (size_t i = 0; i < tmp_pts.size(); ++i) {
      cv::KeyPoint key;
      key.pt = tmp_pts[i];
      key.octave = 0;
      keypoints_.push_back(key);
    }

    brisk_extractor.compute(rgb_image_, keypoints_, descriptors_);
  }

  const cv::Mat& getDescriptors() const { return descriptors_; }

  comm_msgs::keyframe toKeyframeMsg(int num_odom_connections) {
    comm_msgs::keyframe kf_msg;
    kf_msg.header.stamp = timestamp_;
    kf_msg.frameId = index_;
    kf_msg.agentId = cid_;

    kf_msg.header.stamp = timestamp_;
    tf::poseKindrToMsg(T_G_C_, &kf_msg.odometry.pose.pose);

    cv::Mat combined_descriptor(window_descriptors_.rows + descriptors_.rows,
                                window_descriptors_.cols, descriptors_.type());
    if (window_descriptors_.rows > 0) {
      window_descriptors_.copyTo(
          combined_descriptor.rowRange(0, window_descriptors_.rows));
    }
    if (descriptors_.rows > 0) {
      descriptors_.copyTo(combined_descriptor.rowRange(
          window_descriptors_.rows,
          window_descriptors_.rows + descriptors_.rows));
    }
    sensor_msgs::fillImage(
        kf_msg.keyPtsDescriptors, sensor_msgs::image_encodings::MONO8,
        combined_descriptor.rows, combined_descriptor.cols,
        combined_descriptor.step.buf[0], combined_descriptor.data);
    for (int i = 1; i < num_odom_connections; i++) {
      kf_msg.connections.emplace_back(index_ - i);
    }
    const size_t num_total_keypoints = combined_descriptor.rows;

    kf_msg.numKeyPts = num_total_keypoints;

    // TODO(mikexyl): unfinished

    return kf_msg;
  }

 private:
  ros::Time timestamp_;
  int index_;
  CliId cid_;
  TransformationD T_G_C_;
  cv::Mat rgb_image_;
  cv::Mat depth_image_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<cv::KeyPoint> window_keypoints_;
  cv::Mat window_descriptors_;
  cv::Mat descriptors_;
};

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
          sensor(static_cast<SensorType>(0)) {}
    std::vector<double> camera_k;
    std::vector<double> camera_d;
    float min_dist_m;
    float min_yaw_rad;
    float min_local_score;
    int min_kf_n;
    std::string voc_file;
    int sensor;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Keyframe Publisher using Config:" << std::endl
        << "  Min KF Dist: " << v.min_dist_m << " m" << std::endl
        << "  Min Yaw Diff: " << v.min_yaw_rad << " rad" << std::endl
        << "  Min KF Every: " << v.min_kf_n << " frames" << std::endl
        << "  Min Local Score: " << v.min_local_score << std::endl
        << "  Sensor Type: " << v.sensor << std::endl
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
    return config;
  }

  typedef DBoW2::TemplatedVocabulary<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>
      BRISKVocabulary;

  KeyframeTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : config_(getConfigFromRosParam(nh_private)),
        kf_index(0),
        n_since_last_kf(0) {
    nh_private_.param<CliId>("client_id", cid_, cid_);
    if (config_.voc_file.empty()) {
      LOG(FATAL) << "No voc file path given";
    }
    voc_.reset(new BRISKVocabulary(config_.voc_file));
    CHECK(voc_->empty()) << "voc file invalid";
    brisk_extractor_.reset(new brisk::BriskDescriptorExtractor(
        true, false, brisk::BriskDescriptorExtractor::briskV2));

    if (config_.camera_k.size() && config_.camera_d.size())
      setCameraInfo(config_.camera_k, config_.camera_d);

    subscribeToTopics();
    advertiseTopics();
  }

  ~KeyframeTracker() = default;

  void subscribeToTopics() {
    rgb_subscriber_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        nh_private_, "rgb/image_raw", 1));
    depth_subscriber_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        nh_private_, "depth/image_raw", 1));
    odom_subscriber_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh_private_, "odometry", 1));
    sync_.reset(new message_filters::Synchronizer<sync_pol>(
        sync_pol(10), *rgb_subscriber_, *depth_subscriber_, *odom_subscriber_));
    sync_->registerCallback(
        boost::bind(&KeyframeTracker::imageRGBDCallback, this, _1, _2, _3));

    if (config_.camera_k.empty() || config_.camera_d.empty())
      camera_info_sub_ = nh_private_.subscribe(
          "camera_info", 10, &KeyframeTracker::cameraInfoCallback, this);
  }

  void advertiseTopics() {
    kf_pub_ = nh_private_.advertise<comm_msgs::keyframe>("keyframe", 10, true);
  }

 private:
  CliId cid_;
  enum SensorType { MONOCULAR = 0, STEREO, RGBD };
  Config config_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
      rgb_subscriber_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
      depth_subscriber_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>
      odom_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;

  std::shared_ptr<BRISKVocabulary> voc_;
  std::shared_ptr<brisk::BriskDescriptorExtractor> brisk_extractor_;
  void imageRGBDCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const sensor_msgs::ImageConstPtr& depth_msg,
                     const nav_msgs::Odometry& odom_msg) {
    odomCallback(odom_msg);

    TransformationD T_G_C = T_G_C_queue_.back().second;
    TransformationD T_lastKF_C = T_G_lastKF.inverse() * T_G_C;

    // TODO(mikexyl): forgot how to convert kindr quaternion to rpy
    if (n_since_last_kf++ > config_.min_kf_n &&
        (T_lastKF_C.getPosition().norm() > config_.min_dist_m ||
         T_lastKF_C.getRotation().norm() > config_.min_yaw_rad)) {
      cv_bridge::CvImageConstPtr rgb_image_ptr, depth_image_ptr;

      try {
        rgb_image_ptr = cv_bridge::toCvShare(rgb_msg);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      try {
        depth_image_ptr = cv_bridge::toCvShare(depth_msg);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      Keyframe new_kf(rgb_msg->header.stamp, kf_index, cid_,
                      rgb_image_ptr->image, depth_image_ptr->image, T_G_C,
                      *brisk_extractor_);
      DBoW2::BowVector new_bow_vec;
      voc_->transform(new_kf.getDescriptors(), new_bow_vec);
      if (voc_->score(new_bow_vec, bow_vec_lastKF) < config_.min_local_score) {
        kf_index++;
        n_since_last_kf = 0;
        T_G_lastKF = T_G_C;
      }
    }

    ROS_INFO_STREAM_ONCE("Got first image and odom message");
  }

  ros::Subscriber odom_sub_;
  void odomCallback(const nav_msgs::Odometry& odom_msg) {
    TransformationD T_G_C;
    tf::poseMsgToKindr(odom_msg.pose.pose, &T_G_C);
    if (T_G_C_queue_.size() == kOdomQueueSize) T_G_C_queue_.pop_front();
    T_G_C_queue_.emplace_back(odom_msg.header.stamp, T_G_C);
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
    cv::Mat k(3, 3, CV_32F), dist_coef(d_vec.size(), 1, CV_32F);
    k.at<float>(0, 0) = k_vec[0];
    k.at<float>(1, 1) = k_vec[4];
    k.at<float>(2, 0) = k_vec[2];
    k.at<float>(2, 1) = k_vec[5];
    k.at<float>(2, 2) = 1.0;
    k.copyTo(k_);

    for (int i = 0; i < d_vec.size(); i++) {
      dist_coef.at<float>(i) = d_vec[i];
    }
    dist_coef.copyTo(dist_coef_);
  }

  enum TrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
  };
  TrackingState state_;

  int kf_index;
  int n_since_last_kf;
  TransformationD T_G_lastKF;
  DBoW2::BowVector bow_vec_lastKF;
  std::deque<std::pair<ros::Time, TransformationD>> T_G_C_queue_;

  ros::Publisher kf_pub_;

  constexpr static size_t kOdomQueueSize = 10;
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_KEYFRAME_TRACKER_H_
