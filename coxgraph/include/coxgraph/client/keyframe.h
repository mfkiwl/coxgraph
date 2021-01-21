#ifndef COXGRAPH_CLIENT_KEYFRAME_H_
#define COXGRAPH_CLIENT_KEYFRAME_H_

#include <brisk/brisk.h>
#include <comm_msgs/keyframe.h>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "DBoW2/FBRISK.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "comm_msgs/landmark.h"
#include "coxgraph/common.h"

namespace coxgraph {
namespace client {

typedef DBoW2::TemplatedVocabulary<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>
    BRISKVocabulary;
typedef std::map<int, TransformationD> KfPoseMap;

struct TrackingConfig {
  TrackingConfig()
      : min_features_num(150), odom_connection_num(3), min_tracked_num(5) {}
  int min_features_num;
  int min_tracked_num;
  int odom_connection_num;

  friend inline std::ostream& operator<<(std::ostream& s,
                                         const TrackingConfig& v) {
    s << std::endl
      << "Keyframe Tracking Config:" << std::endl
      << "  Min Features: " << v.min_features_num << std::endl
      << "  Min Tracked Num: " << v.min_tracked_num << std::endl
      << "  Odom Connection Num: " << v.odom_connection_num << std::endl
      << "-------------------------------------------" << std::endl;
    return (s);
  }
};

static TrackingConfig getTrackingConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TrackingConfig config;
  nh_private.param<int>("min_features_num", config.min_features_num,
                        config.min_features_num);
  nh_private.param<int>("odom_connection_num", config.odom_connection_num,
                        config.odom_connection_num);
  nh_private.param<int>("min_trakced_num", config.odom_connection_num,
                        config.odom_connection_num);
  return config;
}
// Borrowed from vins_client_server
class Keyframe {
 public:
  typedef std::shared_ptr<Keyframe> Ptr;
  typedef Eigen::Matrix<double, 3, 4> Matrix34d;

  Keyframe(
      const ros::Time& timestamp, int index, CliId cid,
      const cv::Mat& rgb_image, const TransformationD T_G_C,
      const std::shared_ptr<KfPoseMap>& kf_pose_map,
      const Keyframe::Ptr& ref_kf,
      const std::shared_ptr<brisk::BriskDescriptorExtractor>& brisk_extractor,
      const cv::Mat& k, const cv::Mat& dist_coef,
      TrackingConfig tracking_config, const cv::Mat& depth_image = cv::Mat())
      : timestamp_(timestamp),
        index_(index),
        cid_(cid),
        rgb_image_(rgb_image),
        depth_image_(depth_image),
        T_G_C_(T_G_C),
        kf_pose_map_(kf_pose_map),
        ref_kf_(ref_kf),
        brisk_extractor_(brisk_extractor),
        k_(k),
        dist_coef_(dist_coef),
        tracking_config_(tracking_config) {}

  virtual ~Keyframe() = default;

  void setRefKeyframe(const Keyframe::Ptr& ref_kf) { ref_kf_ = ref_kf; }

  void addKeypointsFromDepth() {}

  auto computeBRIEFDescriptors(const std::vector<cv::Point2f>& keypoints_in,
                               std::vector<cv::Point2f>* keypoints_out) {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> tmp_keypoints;
    for (size_t i = 0; i < keypoints_in.size(); ++i) {
      cv::KeyPoint key;
      key.pt = keypoints_in[i];
      key.octave = 0;
      tmp_keypoints.push_back(key);
    }

    brisk_extractor_->compute(rgb_image_, tmp_keypoints, descriptors);

    if (keypoints_out != nullptr) {
      keypoints_out->clear();
      for (auto pt : tmp_keypoints) keypoints_out->emplace_back(pt.pt);
    }
    return descriptors;
  }

  void getProjectionMap(TransformationD T_1_2, Matrix34d* P1, Matrix34d* P2) {
    Eigen::Matrix3d k_eigen;
    cv::cv2eigen(k_, k_eigen);
    Matrix34d t_eigen;
    t_eigen.block(0, 0, 3, 3).setIdentity();
    t_eigen.col(3).setZero();
    if (P1 != nullptr) *P1 = k_eigen * t_eigen;

    t_eigen.block(0, 0, 3, 3) = T_1_2.getRotationMatrix();
    t_eigen.col(3) = T_1_2.getPosition();
    if (P2 != nullptr) *P2 = k_eigen * t_eigen;
  }

  int getID() const { return index_; }

  auto undistortKeyPoint(const cv::Point2f& pt) {
    std::vector<cv::Point2f> pts_un;
    undistortKeyPoints({pt}, &pts_un);
    return pts_un[0];
  }

  void undistortKeyPoints(const std::vector<cv::Point2f>& pts,
                          std::vector<cv::Point2f>* pts_un) {
    if (dist_coef_.at<float>(0) == 0.0) {
      return;
    }

    // Fill matrix with points
    size_t N = pts.size();
    cv::Mat mat(N, 2, CV_32F);
    for (int i = 0; i < N; i++) {
      mat.at<float>(i, 0) = pts.at(i).x;
      mat.at<float>(i, 1) = pts.at(i).y;
    }

    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, k_, dist_coef_, cv::Mat(), k_);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    pts_un->resize(N);
    for (int i = 0; i < N; i++) {
      cv::Point2f kp = pts[i];
      kp.x = mat.at<float>(i, 0);
      kp.y = mat.at<float>(i, 1);
      pts_un->at(i) = kp;
    }
  }

  const DBoW2::BowVector& getBowVec() const { return bow_vec_; }

  bool goodToPublish() {
    if (descriptors_to_pub_.empty()) return false;
    return true;
  }

  comm_msgs::keyframe asKeyframeMsg() {
    CHECK_EQ(descriptors_to_pub_.size(), keypoints_to_pub_.size());

    comm_msgs::keyframe kf_msg;
    kf_msg.header.stamp = timestamp_;
    kf_msg.frameId = index_;
    kf_msg.agentId = cid_;
    kf_msg.header.stamp = timestamp_;
    tf::poseKindrToMsg(T_G_C_, &kf_msg.odometry.pose.pose);

    cv::Mat combined_descriptors(descriptors_to_pub_.size(),
                                 descriptors_to_pub_[0].cols,
                                 descriptors_to_pub_[0].type());

    for (int i = 0; i < descriptors_to_pub_.size(); i++) {
      auto descriptor = descriptors_to_pub_[i];
      descriptor.copyTo(combined_descriptors.rowRange(i, i + 1));
    }

    sensor_msgs::fillImage(
        kf_msg.keyPtsDescriptors, sensor_msgs::image_encodings::MONO8,
        combined_descriptors.rows, combined_descriptors.cols,
        combined_descriptors.step.buf[0], combined_descriptors.data);

    for (int i = 1; i < tracking_config_.odom_connection_num; i++) {
      kf_msg.connections.emplace_back(index_ - i);
    }

    const size_t num_total_keypoints = combined_descriptors.rows;

    kf_msg.numKeyPts = num_total_keypoints;

    for (int i = 0; i < num_total_keypoints; i++) {
      comm_msgs::keypoint kp;
      kp.x = keypoints_to_pub_[i].x;
      kp.y = keypoints_to_pub_[i].y;
      kf_msg.keyPts.emplace_back(kp);
      if (i < combined_descriptors.rows) {
        comm_msgs::landmark lm;
        lm.index = landmarks_[i].second;
        lm.x = landmarks_[i].first[0];
        lm.y = landmarks_[i].first[1];
        lm.z = landmarks_[i].first[2];
        kf_msg.landmarks.emplace_back(lm);
      }
    }
    return kf_msg;
  }

  const Keyframe::Ptr& getRefKfPtr() const { return ref_kf_; }
  const cv::Mat& getRgbImage() const { return rgb_image_; }
  const TransformationD& getTGC() const { return T_G_C_; }

  auto const& getTrackPts() const { return track_pts_; }
  std::vector<int> good_ids_;
  const std::vector<int>& getGoodIds() const { return good_ids_; }
  void getGoodPtsToTrack(std::vector<cv::Point2f>* good_pts,
                         std::vector<int>* good_ids) {
    for (int i = 0; i < track_pts_.size(); i++) {
      if (i >= track_pts_status_.size() || track_pts_status_[i]) {
        good_pts->emplace_back(track_pts_[i]);
        good_ids_.emplace_back(i);
      }
    }
    *good_ids = good_ids_;
  }

  std::vector<cv::Point2f> getPointTrackHistory(int id) {
    auto pt_it = pts_track_history_.find(id);
    if (pt_it != pts_track_history_.end())
      return pt_it->second;
    else
      return std::vector<cv::Point2f>();
  }

  void addPointTrackHistory(int id, cv::Point2f new_point,
                            const std::vector<cv::Point2f>& prev_history) {
    CHECK(!pts_track_history_.count(id));
    pts_track_history_.emplace(id, prev_history);
    pts_track_history_[id].emplace_back(new_point);
    // LOG(INFO) << pts_track_history_[id].size();
  }

  void trackKeypointsRef(bool prepare_publish_data,
                         pcl::PointCloud<pcl::PointXYZ>* landmarks) {
    landmarks->clear();
    if (ref_kf_ != nullptr) {
      std::vector<cv::Point2f> prev_pts;
      std::vector<int> prev_ids;
      ref_kf_->getGoodPtsToTrack(&prev_pts, &prev_ids);

      // optical flow tracking
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(ref_kf_->getRgbImage(), rgb_image_, prev_pts,
                               track_pts_, track_pts_status_, err,
                               cv::Size(21, 21), 3);

      for (int i = 0; i < track_pts_.size(); i++)
        if (track_pts_status_[i] && !inBorder(track_pts_[i]))
          track_pts_status_[i] = 0;

      for (int i = 0; i < track_pts_.size(); i++) {
        if (track_pts_status_[i]) {
          addPointTrackHistory(i, track_pts_[i],
                               ref_kf_->getPointTrackHistory(prev_ids[i]));
        } else {
          auto history = ref_kf_->getPointTrackHistory(prev_ids[i]);
          size_t n_history = history.size();
          LOG(INFO) << "dead " << n_history;
          if (prepare_publish_data &&
              history.size() > tracking_config_.min_tracked_num) {
            std::vector<cv::Point2f> points_un;
            undistortKeyPoints(history, &points_un);

            std::vector<Matrix34d> proj_mats;
            Matrix34d proj_0;
            getProjectionMap(TransformationD(), &proj_0, nullptr);
            proj_mats.emplace_back(proj_0);

            TransformationD T_G_0 = (*kf_pose_map_)[index_ - n_history];

            for (int i = 1; i < history.size(); i--) {
              TransformationD T_G_N = (*kf_pose_map_)[index_ - i];
              TransformationD T_0_N = T_G_0.inverse() * T_G_N;
              Matrix34d proj_n;
              getProjectionMap(T_0_N, nullptr, &proj_n);
              proj_mats.emplace_back(proj_n);
            }

            Eigen::Vector3f point3d_eigen;
            if (!triangulatePoints(proj_mats, points_un, &point3d_eigen)) {
              LOG(INFO) << "triangulatePoints failed";
              continue;
            }

            pcl::PointXYZ point3d_pcl(point3d_eigen[0], point3d_eigen[1],
                                      point3d_eigen[2]);
            point3d_pcl = pcl::transformPoint(
                point3d_pcl, Eigen::Translation3d(T_G_0.getPosition()) *
                                 T_G_0.getRotationMatrix());
            landmarks->push_back(point3d_pcl);

            //          keypoints_to_pub_.emplace_back(point_c_un);
            // TODO(mikexyl): point not in current frame, this is only for debug
            // counting
            landmarks_.emplace_back(point3d_eigen,
                                    keypoints_to_pub_.size() - 1);
            //          descriptors_to_pub_.emplace_back(descriptor);
          }
        }
      }

      LOG_IF(INFO, prepare_publish_data)
          << "landmark size: " << landmarks_.size();
    }

    if (track_pts_.size() < tracking_config_.min_features_num) {
      std::vector<cv::Point2f> new_pts;
      cv::goodFeaturesToTrack(
          rgb_image_, new_pts,
          tracking_config_.min_features_num - track_pts_.size(), 0.01, 10);
      track_pts_.insert(track_pts_.end(), new_pts.begin(), new_pts.end());
    }
  }

  void extractExtraKeypoints() {}

  bool pointWasTracked(int i) { return pts_track_history_.count(i); }

  std::vector<uchar> track_pts_status_;
  const std::vector<uchar>& getTrackPtsStatus() const {
    return track_pts_status_;
  }

  static bool triangulatePoints(const std::vector<Matrix34d>& proj_mats,
                                const std::vector<cv::Point2f>& points,
                                Eigen::Vector3f* point3d) {
    CHECK(point3d != nullptr);

    Eigen::MatrixXd A(points.size() * 2, 4);
    for (size_t i = 0; i < points.size(); ++i) {
      A.row(i * 2 + 0) =
          points[i].x * proj_mats[i].row(2) - proj_mats[i].row(0);
      A.row(i * 2 + 1) =
          points[i].y * proj_mats[i].row(2) - proj_mats[i].row(1);
    }
    auto ev = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
    if (ev[3] / ev[2] > 1e-2) return false;
    *point3d = Eigen::Vector3f(ev[0] / ev[3], ev[1] / ev[3], ev[2] / ev[3]);
    return true;
  }

  bool inBorder(const cv::Point2f& pt) {
    constexpr int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < rgb_image_.cols - BORDER_SIZE &&
           BORDER_SIZE <= img_y && img_y < rgb_image_.rows - BORDER_SIZE;
  }

  template <typename T>
  static void reduceVector(std::vector<T>* v, std::vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < v->size(); i++)
      if (status[i] || status.size() <= i) (*v)[j++] = (*v)[i];
    v->resize(j);
  }

  auto const& getLandmarks() const { return landmarks_; }
  auto getLandmarksAsPcl() const {
    pcl::PointCloud<pcl::PointXYZ> landmark_pc;
    landmark_pc.header.frame_id = "world";
    for (auto const& lm : landmarks_)
      landmark_pc.push_back(
          pcl::PointXYZ(lm.first[0], lm.first[1], lm.first[2]));
    pcl::transformPointCloud(landmark_pc, landmark_pc,
                             T_G_C_.getTransformationMatrix());
    return landmark_pc;
  }

 private:
  TrackingConfig tracking_config_;
  ros::Time timestamp_;
  int index_;
  Keyframe::Ptr ref_kf_;
  CliId cid_;
  TransformationD T_G_C_;
  cv::Mat rgb_image_;
  cv::Mat depth_image_;
  cv::Mat k_;
  cv::Mat dist_coef_;
  std::vector<cv::Point2f> keypoints_to_pub_;
  std::vector<std::pair<Eigen::Vector3f, int>> landmarks_;
  std::vector<cv::Point2f> track_pts_;
  std::map<int, std::vector<cv::Point2f>> pts_track_history_;
  std::map<cv::Point2f, int> n_tracked_;
  std::vector<float> z_from_depth_;
  std::vector<cv::Mat> descriptors_to_pub_;
  DBoW2::BowVector bow_vec_;
  std::shared_ptr<brisk::BriskDescriptorExtractor> brisk_extractor_;
  std::shared_ptr<KfPoseMap> kf_pose_map_;
};  // namespace client

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_KEYFRAME_H_
