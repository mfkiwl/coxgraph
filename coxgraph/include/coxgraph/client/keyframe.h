#ifndef COXGRAPH_CLIENT_KEYFRAME_H_
#define COXGRAPH_CLIENT_KEYFRAME_H_

#include <brisk/brisk.h>
#include <comm_msgs/keyframe.h>
#include <minkindr_conversions/kindr_msg.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>

#include <memory>
#include <vector>

#include "DBoW2/FBRISK.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "coxgraph/common.h"

namespace coxgraph {
namespace client {

// Borrowed from vins_client_server
class Keyframe {
 public:
  typedef std::shared_ptr<Keyframe> Ptr;

  Keyframe(const ros::Time& timestamp, int index, CliId cid,
           const cv::Mat& rgb_image, const cv::Mat& depth_image,
           const TransformationD T_G_C,
           const brisk::BriskDescriptorExtractor& brisk_extractor,
           const BRISKVocabulary& voc, const cv::Mat& k,
           const cv::Mat& dist_coef)
      : timestamp_(timestamp),
        index_(index),
        cid_(cid),
        rgb_image_(rgb_image),
        depth_image_(depth_image),
        T_G_C_(T_G_C),
        k_(k),
        dist_coef_(dist_coef) {
    computeBRIEFPoint(brisk_extractor);
    voc.transform(descriptors_, bow_vec_);
  }

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
    vector<cv::Point2f> tmp_pts;
    cv::goodFeaturesToTrack(rgb_image_, tmp_pts, 500, 0.01, 10);
    for (size_t i = 0; i < tmp_pts.size(); ++i) {
      cv::KeyPoint key;
      key.pt = tmp_pts[i];
      key.octave = 0;
      keypoints_.push_back(key);
    }
    addKeypointsFromDepth();

    brisk_extractor.compute(rgb_image_, keypoints_, descriptors_);
  }

  // Stolen from orbslam
  void undistortKeyPoints(const cv::Mat& k, const cv::Mat& dist_coef) {
    if (dist_coef.at<float>(0) == 0.0) {
      return;
    }

    // Fill matrix with points
    size_t N = keypoints_.size();
    cv::Mat mat(N, 2, CV_32F);
    for (int i = 0; i < N; i++) {
      mat.at<float>(i, 0) = keypoints_[i].pt.x;
      mat.at<float>(i, 1) = keypoints_[i].pt.y;
    }

    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, k, dist_coef, cv::Mat(), k);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    keypoints_un_.resize(N);
    for (int i = 0; i < N; i++) {
      cv::KeyPoint kp = keypoints_[i];
      kp.pt.x = mat.at<float>(i, 0);
      kp.pt.y = mat.at<float>(i, 1);
      keypoints_un_[i] = kp;
    }
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
      z_from_depth_[i] = depth_image_.at<float>(keypoint.pt);
    }
  }

  const cv::Mat& getDescriptors() const { return descriptors_; }
  const DBoW2::BowVector& getBowVec() const { return bow_vec_; }
  const std::vector<cv::KeyPoint>& getKeyPoint() { return keypoints_; }
  std::vector<cv::Point2f> getKeyPointAsPoint() {
    std::vector<cv::Point2f> keypoints;
    for (auto const& keypoint : keypoints_) {
      keypoints.emplace_back(keypoint.pt);
    }
    return keypoints;
  }
  std::vector<cv::KeyPoint>* getKeyPointPtr() { return &keypoints_; }
  const std::vector<cv::KeyPoint>& getKeyPointUn() { return keypoints_un_; }
  std::vector<cv::KeyPoint>* getKeyPointUnPtr() { return &keypoints_; }

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

  void trackKeypointsRef() {
    // optical flow tracking
    vector<uchar> status;
    vector<float> err;
    std::vector<cv::Point2f> prev_pts = ref_kfs_[0]->getKeyPointAsPoint();
    cv::calcOpticalFlowPyrLK(ref_kfs_[0]->getRgbImage(), rgb_image_,
                             ref_kfs_[0]->getKeyPointAsPoint(), cur_pts_,
                             status, err, cv::Size(21, 21), 3);

    for (int i = 0; i < cur_pts_.size(); i++)
      if (status[i] && !inBorder(cur_pts_[i])) status[i] = 0;
    reduceVector(&prev_pts, status);
    reduceVector(&cur_pts_, status);
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

  static void reduceVector(vector<cv::Point2f>* v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < v->size(); i++)
      if (status[i]) (*v)[j++] = (*v)[i];
    v->resize(j);
  }

 private:
  ros::Time timestamp_;
  int index_;
  std::vector<Keyframe::Ptr> ref_kfs_;
  CliId cid_;
  TransformationD T_G_C_;
  cv::Mat rgb_image_;
  cv::Mat depth_image_;
  cv::Mat k_;
  cv::Mat dist_coef_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<cv::KeyPoint> keypoints_un_;
  std::vector<cv::Point2f> cur_pts_;
  std::vector<float> z_from_depth_;
  cv::Mat descriptors_;
  DBoW2::BowVector bow_vec_;
};

}  // namespace client
}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_KEYFRAME_H_
