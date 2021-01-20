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

struct TrackingConfig {
  TrackingConfig() : min_features(150), odom_connection_num(3) {}
  int min_features;
  int odom_connection_num;
};

static TrackingConfig getTrackingConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TrackingConfig config;
  nh_private.param<int>("tracking/min_freatures", config.min_features,
                        config.min_features);
  nh_private.param<int>("odom_connection_num", config.odom_connection_num,
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
      const std::shared_ptr<brisk::BriskDescriptorExtractor>& brisk_extractor,
      const cv::Mat& k, const cv::Mat& dist_coef,
      TrackingConfig tracking_config, const cv::Mat& depth_image = cv::Mat())
      : timestamp_(timestamp),
        index_(index),
        cid_(cid),
        rgb_image_(rgb_image),
        depth_image_(depth_image),
        T_G_C_(T_G_C),
        brisk_extractor_(brisk_extractor),
        k_(k),
        dist_coef_(dist_coef),
        tracking_config_(tracking_config) {}

  virtual ~Keyframe() = default;

  void addRefKeyframe(const Keyframe::Ptr& ref_kf) {
    ref_kfs_.emplace_back(ref_kf);
  }

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

  void getProjectionMap(TransformationD T_1_2, cv::Mat* R1, cv::Mat* R2,
                        cv::Mat* P1, cv::Mat* P2) {
    cv::Mat rot_mat;
    cv::eigen2cv(T_1_2.getRotationMatrix(), rot_mat);
    cv::Vec3d t;
    cv::eigen2cv(T_1_2.getPosition(), t);

    cv::Mat Q;
    cv::stereoRectify(k_, dist_coef_, k_, dist_coef_, rgb_image_.size(),
                      rot_mat, t, *R1, *R2, *P1, *P2, Q);
  }

  void getProjectionMap(TransformationD T_1_2, Matrix34d* P1, Matrix34d* P2) {
    Eigen::Matrix3d k_eigen;
    cv::cv2eigen(k_, k_eigen);
    Matrix34d t_eigen;
    t_eigen.block(0, 0, 3, 3).setIdentity();
    t_eigen.col(3).setZero();
    *P1 = k_eigen * t_eigen;

    t_eigen.block(0, 0, 3, 3) = T_1_2.getRotationMatrix();
    t_eigen.col(3) = T_1_2.getPosition();
    *P2 = k_eigen * t_eigen;
  }

  auto undistortKeyPoint(const cv::Point2f& pt) {
    std::vector<cv::Point2f> pts_un;
    undistortKeyPoints(k_, dist_coef_, {pt}, &pts_un);
    return pts_un[0];
  }

  static void undistortKeyPoints(const cv::Mat& k, const cv::Mat& dist_coef,
                                 const std::vector<cv::Point2f>& pts,
                                 std::vector<cv::Point2f>* pts_un) {
    if (dist_coef.at<float>(0) == 0.0) {
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
    cv::undistortPoints(mat, mat, k, dist_coef, cv::Mat(), k);
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
        lm.x = landmarks_[i].first.x;
        lm.y = landmarks_[i].first.y;
        lm.z = landmarks_[i].first.z;
        kf_msg.landmarks.emplace_back(lm);
      }
    }
    return kf_msg;
  }

  const std::vector<Keyframe::Ptr>& getRefKfsPtr() const { return ref_kfs_; }
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

  void trackKeypointsRef(bool prepare_publish_data) {
    if (ref_kfs_.size()) {
      std::vector<cv::Point2f> prev_pts_0;
      std::vector<int> prev_good_ids;
      ref_kfs_[0]->getGoodPtsToTrack(&prev_pts_0, &prev_good_ids);

      // optical flow tracking
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(ref_kfs_[0]->getRgbImage(), rgb_image_,
                               prev_pts_0, track_pts_, track_pts_status_, err,
                               cv::Size(21, 21), 3);

      LOG(INFO) << "tracked pts: " << track_pts_.size();

      for (int i = 0; i < track_pts_.size(); i++)
        if (track_pts_status_[i] && !inBorder(track_pts_[i]))
          track_pts_status_[i] = 0;

      for (int i = 0; i < track_pts_.size(); i++) {
        if (track_pts_status_[i]) {
          pointTracked(i, prev_pts_0[i]);

          if (prepare_publish_data && ref_kfs_.size() == 2) {
            // this logic only support tracked 2 times
            cv::Point2f prev_pt_1;
            if (ref_kfs_[0]->getPointHistory(prev_good_ids[i], &prev_pt_1)) {
              // If can't get descriptor, skip it
              cv::Mat descriptor;
              descriptor = computeBRIEFDescriptors({track_pts_[i]}, nullptr);
              if (descriptor.rows == 0) continue;

              auto point_c_un = undistortKeyPoint(track_pts_[i]);
              auto point_0_un = undistortKeyPoint(prev_pts_0[i]);
              auto point_1_un = undistortKeyPoint(prev_pt_1);

              TransformationD T_C_0, T_C_1, T_G_0, T_G_1;
              T_G_0 = ref_kfs_[0]->getTGC();
              T_G_1 = ref_kfs_[1]->getTGC();
              T_C_0 = T_G_C_.inverse() * T_G_0;
              T_C_1 = T_G_C_.inverse() * T_G_1;
              Matrix34d PC_0, PC_1, P0, P1;
              getProjectionMap(T_C_0, &PC_0, &P0);
              getProjectionMap(T_C_1, &PC_1, &P1);

              cv::Point3f point3d;
              if (!triangulatePoints({PC_0, P0, P1},
                                     {point_c_un, point_0_un, point_1_un},
                                     &point3d))
                continue;

              keypoints_to_pub_.emplace_back(point_c_un);
              landmarks_.emplace_back(point3d, keypoints_to_pub_.size() - 1);
              descriptors_to_pub_.emplace_back(descriptor);
            }
          }
        }
      }

      LOG_IF(INFO, prepare_publish_data)
          << "landmark size: " << landmarks_.size();
    }

    if (track_pts_.size() < tracking_config_.min_features) {
      std::vector<cv::Point2f> new_pts;
      cv::goodFeaturesToTrack(rgb_image_, new_pts,
                              tracking_config_.min_features - track_pts_.size(),
                              0.01, 10);
      track_pts_.insert(track_pts_.end(), new_pts.begin(), new_pts.end());
    }
  }

  void extractExtraKeypoints() {}

  void pointTracked(int i, cv::Point2f prev_pt) {
    auto pt_it = pts_track_history_.find(i);
    if (pt_it == pts_track_history_.end())
      pts_track_history_.emplace(i, prev_pt);
    else
      pt_it->second = prev_pt;
  }

  bool getPointHistory(int i, cv::Point2f* prev_pt) {
    if (!pts_track_history_.count(i)) {
      return false;
    } else {
      *prev_pt = pts_track_history_[i];
      return true;
    }
  }

  bool pointWasTracked(int i) { return pts_track_history_.count(i); }

  std::vector<uchar> track_pts_status_;
  const std::vector<uchar>& getTrackPtsStatus() const {
    return track_pts_status_;
  }
  const std::vector<int>& getTrackPtsId() const { return track_pts_id_; }

  static bool triangulatePoints(const std::vector<Matrix34d>& proj_mats,
                                const std::vector<cv::Point2f>& points,
                                cv::Point3f* point3d) {
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
    *point3d = cv::Point3f(ev[0] / ev[3], ev[1] / ev[3], ev[2] / ev[3]);
    return true;
  }

  void triangulateKeypointRef(const std::vector<cv::Point2f>& cur_pts,
                              const std::vector<cv::Point2f>& prev_pts,
                              const TransformationD& T_G_prev,
                              const TransformationD& T_G_cur, cv::Mat* pts4d) {
    CHECK(pts4d != nullptr);
    CHECK_EQ(cur_pts.size(), prev_pts.size());
    std::vector<cv::Point2f> cur_pts_un;
    std::vector<cv::Point2f> prev_pts_un;
    undistortKeyPoints(k_, dist_coef_, cur_pts, &cur_pts_un);
    undistortKeyPoints(k_, dist_coef_, prev_pts, &prev_pts_un);

    TransformationD T_prev_cur = T_G_prev.inverse() * T_G_cur;
    cv::Mat R(3, 3, CV_32F, T_prev_cur.getRotationMatrix().data());
    cv::Vec3f T(T_prev_cur.cast<float>().getPosition().data());
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(k_, dist_coef_, k_, dist_coef_, rgb_image_.size(), R, T,
                      R1, R2, P1, P2, Q);

    cv::triangulatePoints(P1, P2, prev_pts_un, cur_pts_un, *pts4d);
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
      landmark_pc.push_back(pcl::PointXYZ(lm.first.x, lm.first.y, lm.first.z));
    pcl::transformPointCloud(landmark_pc, landmark_pc,
                             T_G_C_.getTransformationMatrix());
    return landmark_pc;
  }

 private:
  TrackingConfig tracking_config_;
  ros::Time timestamp_;
  int index_;
  std::vector<Keyframe::Ptr> ref_kfs_;
  CliId cid_;
  TransformationD T_G_C_;
  cv::Mat rgb_image_;
  cv::Mat depth_image_;
  cv::Mat k_;
  cv::Mat dist_coef_;
  std::vector<cv::Point2f> keypoints_to_pub_;
  std::vector<std::pair<cv::Point3f, int>> landmarks_;
  std::vector<cv::Point2f> track_pts_;
  std::vector<int> track_pts_id_;
  std::map<int, cv::Point2f> pts_track_history_;
  std::map<cv::Point2f, int> n_tracked_;
  std::vector<float> z_from_depth_;
  std::vector<cv::Mat> descriptors_to_pub_;
  DBoW2::BowVector bow_vec_;
  std::shared_ptr<brisk::BriskDescriptorExtractor> brisk_extractor_;
};  // namespace client

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_KEYFRAME_H_
