#ifndef COXGRAPH_CLIENT_KEYFRAME_H_
#define COXGRAPH_CLIENT_KEYFRAME_H_

#include <brisk/brisk.h>
#include <comm_msgs/keyframe.h>
#include <minkindr_conversions/kindr_msg.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>

#include <map>
#include <memory>
#include <vector>

#include "DBoW2/FBRISK.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "coxgraph/common.h"

namespace coxgraph {
namespace client {

struct TrackingConfig {
  TrackingConfig() : min_features(150) {}
  int min_features;
};

static TrackingConfig getTrackingConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TrackingConfig config;
  nh_private.param<int>("tracking/min_freatures", config.min_features,
                        config.min_features);
  return config;
}
// Borrowed from vins_client_server
class Keyframe {
 public:
  typedef std::shared_ptr<Keyframe> Ptr;

  Keyframe(const ros::Time& timestamp, int index, CliId cid,
           const cv::Mat& rgb_image, const cv::Mat& depth_image,
           const TransformationD T_G_C,
           const brisk::BriskDescriptorExtractor& brisk_extractor,
           const BRISKVocabulary& voc, const cv::Mat& k,
           const cv::Mat& dist_coef, TrackingConfig tracking_config)
      : timestamp_(timestamp),
        index_(index),
        cid_(cid),
        rgb_image_(rgb_image),
        depth_image_(depth_image),
        T_G_C_(T_G_C),
        k_(k),
        dist_coef_(dist_coef),
        tracking_config_(tracking_config) {}

  virtual ~Keyframe() = default;

  void addRefKeyframe(const Keyframe::Ptr& ref_kf) {
    ref_kfs_.emplace_back(ref_kf);
  }

  void addKeypointsFromDepth() {
    // remove bad keypoints if depth map gradient is too big
    for (int i = 0; i < keypoints_.size(); i++) {
    }
  }

  void computeBRIEFPoint(
      const brisk::BriskDescriptorExtractor& brisk_extractor) {
    //  brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>
    //      brisk_detector(0, 10.0, 300.0, 200);
    keypoints_.clear();
    cv::goodFeaturesToTrack(rgb_image_, keypoints_, 500, 0.01, 10);
    std::vector<cv::KeyPoint> tmp_keypoints;
    for (size_t i = 0; i < keypoints_.size(); ++i) {
      cv::KeyPoint key;
      key.pt = keypoints_[i];
      key.octave = 0;
      tmp_keypoints.push_back(key);
    }
    addKeypointsFromDepth();

    brisk_extractor.compute(rgb_image_, tmp_keypoints, descriptors_);
  }

  void getProjectionMap(TransformationD T_1_2, cv::Mat* P1, cv::Mat* P2) {
    cv::Mat rot_mat(3, 3, CV_32F, T_1_2.getRotationMatrix().data());
    cv::Vec3f t(T_1_2.cast<float>().getPosition().data());
    getProjectionMap(rot_mat, t, P1, P2);
  }

  void getProjectionMap(const cv::Mat& R, const cv::Vec3f& T, cv::Mat* P1,
                        cv::Mat* P2) {
    cv::Mat R1, R2, Q;
    cv::stereoRectify(k_, dist_coef_, k_, dist_coef_, rgb_image_.size(), R, T,
                      R1, R2, *P1, *P2, Q);
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

  void computeKeypointZFromDepth() {
    z_from_depth_.resize(keypoints_.size());
    for (int i = 0; i < keypoints_.size(); i++) {
      auto keypoint = keypoints_[i];
      z_from_depth_[i] = depth_image_.at<float>(keypoint);
    }
  }

  const cv::Mat& getDescriptors() const { return descriptors_; }
  const DBoW2::BowVector& getBowVec() const { return bow_vec_; }
  const std::vector<cv::Point2f>& getKeyPoint() { return keypoints_; }
  std::vector<cv::Point2f>* getKeyPointPtr() { return &keypoints_; }
  const std::vector<cv::Point2f>& getKeyPointUn() { return keypoints_un_; }
  std::vector<cv::Point2f>* getKeyPointUnPtr() { return &keypoints_; }

  comm_msgs::keyframe toKeyframeMsg(int num_odom_connections) {
    comm_msgs::keyframe kf_msg;
    kf_msg.header.stamp = timestamp_;
    kf_msg.frameId = index_;
    kf_msg.agentId = cid_;

    kf_msg.header.stamp = timestamp_;
    tf::poseKindrToMsg(T_G_C_, &kf_msg.odometry.pose.pose);

    sensor_msgs::fillImage(kf_msg.keyPtsDescriptors,
                           sensor_msgs::image_encodings::MONO8,
                           descriptors_.rows, descriptors_.cols,
                           descriptors_.step.buf[0], descriptors_.data);
    for (int i = 1; i < num_odom_connections; i++) {
      kf_msg.connections.emplace_back(index_ - i);
    }
    const size_t num_total_keypoints = descriptors_.rows;

    kf_msg.numKeyPts = num_total_keypoints;

    // TODO(mikexyl): unfinished

    return kf_msg;
  }

  const std::vector<Keyframe::Ptr>& getRefKfsPtr() const { return ref_kfs_; }
  const cv::Mat& getRgbImage() const { return rgb_image_; }
  const TransformationD& getTGC() const { return T_G_C_; }

  auto const& getTrackPts() const { return track_pts_; }
  void getGoodPtsToTrack(std::vector<cv::Point2f>* good_pts,
                         std::vector<int>* good_ids) const {
    for (int i = 0; i < track_pts_.size(); i++) {
      if (track_pts_status_[i] || i >= track_pts_status_.size()) {
        good_pts->emplace_back(track_pts_[i]);
        good_ids->emplace_back(i);
      }
    }
  }

  void trackKeypointsRef() {
    std::vector<cv::Point2f> prev_good_pts;
    std::vector<int> prev_good_ids;
    ref_kfs_[0]->getGoodPtsToTrack(&prev_good_pts, &prev_good_ids);

    // optical flow tracking
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(ref_kfs_[0]->getRgbImage(), rgb_image_,
                             prev_good_pts, track_pts_, track_pts_status_, err,
                             cv::Size(21, 21), 3);

    for (int i = 0; i < track_pts_.size(); i++)
      if (track_pts_status_[i] && !inBorder(track_pts_[i]))
        track_pts_status_[i] = 0;

    for (int i = 0; i < track_pts_.size(); i++) {
      if (track_pts_status_[i]) {
        pointTracked(i);
        ref_kfs_[0]->pointTracked(prev_good_ids[i]);
        // this logic only support tracked 2 times
        if (ref_kfs_[0]->getPointTrackedNum(prev_good_ids[i]) == 2) {
          auto point_c = track_pts_[i];
          auto point_0 = ref_kfs_[0]->getTrackPts()[prev_good_ids[i]];
          auto point_1 = ref_kfs_[1]->getTrackPts()[prev_good_ids[i]];

          TransformationD T_C_0, T_C_1, T_G_0, T_G_1;
          T_G_0 = ref_kfs_[0]->getTGC();
          T_G_1 = ref_kfs_[1]->getTGC();
          T_C_0 = T_G_C_.inverse() * T_G_0;
          T_C_1 = T_G_C_.inverse() * T_G_1;
          cv::Mat PC_0(3, 4, CV_32F), PC_1(3, 4, CV_32F), P0(3, 4, CV_32F),
              P1(3, 4, CV_32F);
          getProjectionMap(T_C_0, &PC_0, &P0);
          getProjectionMap(T_C_1, &PC_1, &P1);
          CHECK(std::equal(PC_0.begin<float>(), PC_0.end<float>(),
                           PC_1.begin<float>()));

          auto point4d =
              triangulatePoints({PC_0, P0, P1}, {point_c, point_0, point_1});
        }
      }
    }

    if (track_pts_.size() < tracking_config_.min_features) {
      std::vector<cv::Point2f> new_pts;
      cv::goodFeaturesToTrack(rgb_image_, new_pts,
                              tracking_config_.min_features - track_pts_.size(),
                              0.01, 10);
      track_pts_.insert(track_pts_.end(), new_pts.begin(), new_pts.end());
    }
  }

  void pointTracked(int i) {
    auto pt_it = pts_tracked_.find(i);
    if (pt_it == pts_tracked_.end())
      pts_tracked_.emplace(i, 1);
    else
      pt_it->second++;
  }

  int getPointTrackedNum(int i) {
    CHECK(pts_tracked_.count(i));
    return pts_tracked_[i];
  }

  std::vector<uchar> track_pts_status_;
  const std::vector<uchar>& getTrackPtsStatus() const {
    return track_pts_status_;
  }
  const std::vector<int>& getTrackPtsId() const { return track_pts_id_; }

  // std::map<int, std::pair<int, uchar>> track_pts_status_;
  // void updateTrackPointsStatus(const std::vector<uchar>& status) {
  //   CHECK_EQ(track_pts_.size(), status.size());

  //   for (int i = 0; i < status.size(); i++) {
  //     auto pt_status_it = track_pts_status_.find(i);
  //     if (pt_status_it == track_pts_status_.end())
  //       pt_status_it.second.second = status[i];
  //   }
  // }

  static cv::Vec4d triangulatePoints(const std::vector<cv::Mat>& projMats,
                                     const std::vector<cv::Point2f>& points) {
    std::vector<Eigen::Matrix<float, 3, 4>> projMatsEigen;
    for (auto const& projMat : projMats) {
      projMatsEigen.emplace_back(projMat.ptr<float>());
    }
    Eigen::MatrixXf A(points.size() * 2, 4);
    for (size_t i = 0; i < points.size(); ++i) {
      A.row(i * 2 + 0) =
          points[i].x * projMatsEigen[i].row(2) - projMatsEigen[i].row(0);
      A.row(i * 2 + 1) =
          points[i].y * projMatsEigen[i].row(2) - projMatsEigen[i].row(1);
    }
    auto point_eigen = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
    return cv::Vec4d(point_eigen[0], point_eigen[1], point_eigen[2],
                     point_eigen[3]);
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
  std::vector<cv::Point2f> keypoints_;
  std::vector<cv::Point2f> keypoints_un_;
  std::vector<cv::Point2f> track_pts_;
  std::vector<int> track_pts_id_;
  std::map<int, int> pts_tracked_;
  std::map<cv::Point2f, int> n_tracked_;
  std::vector<float> z_from_depth_;
  cv::Mat descriptors_;
  DBoW2::BowVector bow_vec_;
};  // namespace client

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_KEYFRAME_H_
