#include "coxgraph_mod/vio_interface.h"

#include <std_srvs/SetBool.h>

#include <map>
#include <string>

namespace coxgraph {
namespace mod {

class VIOInterface {
 public:
  VIOInterface() {
    initialized_.emplace(InitModule::nh, false);
    initialized_.emplace(InitModule::tf, false);
    initialized_.emplace(InitModule::lc, false);
    initialized_.emplace(InitModule::mapping, false);
  }

  ~VIOInterface() = default;

  void updatePose(Eigen::Matrix4d pose, double timestamp) {
    init(InitModule::tf);
    if (tf_pub_ == nullptr) return;
    tf_pub_->updatePose(pose, timestamp);
  }

  void updatePose(cv::Mat pose, double timestamp) {
    init(InitModule::tf);
    if (tf_pub_ == nullptr) return;
    tf_pub_->updatePose(pose, timestamp);
  }

  void publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp,
                          Eigen::Matrix4d T_A_B) {
    init(InitModule::lc);
    loop_closure_pub_->publishLoopClosure(from_client_id, from_timestamp,
                                          to_client_id, to_timestamp, T_A_B);
  }

  void publishLoopClosure(size_t from_client_id, double from_timestamp,
                          size_t to_client_id, double to_timestamp, cv::Mat R,
                          cv::Mat t) {
    init(InitModule::lc);
    loop_closure_pub_->publishLoopClosure(from_client_id, from_timestamp,
                                          to_client_id, to_timestamp, R, t);
  }

  void publishLoopClosure(const double& from_timestamp,
                          const double& to_timestamp, Eigen::Matrix4d T_A_B) {
    init(InitModule::lc);
    loop_closure_pub_->publishLoopClosure(from_timestamp, to_timestamp, T_A_B);
  }

  void publishLoopClosure(const double& from_timestamp,
                          const double& to_timestamp, cv::Mat R, cv::Mat t) {
    init(InitModule::lc);
    loop_closure_pub_->publishLoopClosure(from_timestamp, to_timestamp, R, t);
  }

  bool toggleMapping(bool b_mapping) {
    init(InitModule::mapping);
    std_srvs::SetBool toggle_mapping_msg;
    toggle_mapping_msg.request.data = b_mapping;
    if (toggle_mapping_srv_.call(toggle_mapping_msg)) {
      return true;
    } else {
      ROS_ERROR_STREAM(
          "Toggle Mapping to "
          << static_cast<std::string>(b_mapping ? "enabled" : "disabled")
          << " Failed");
      return false;
    }
  }

 private:
  ros::NodeHandle* nh_;
  ros::NodeHandle* nh_private_;

  enum InitModule { nh = 0, tf, lc, mapping };

  std::map<InitModule, bool> initialized_;

  void init(InitModule init_module) {
    if (initialized_[init_module]) return;
    switch (init_module) {
      case InitModule::nh:
        nh_ = new ros::NodeHandle();
        nh_private_ = new ros::NodeHandle("~");
        break;
      case InitModule::tf:
        init(InitModule::nh);
        tf_pub_.reset(new TfPublisher(*nh_, *nh_private_));
        break;
      case InitModule::lc:
        init(InitModule::nh);
        loop_closure_pub_.reset(new LoopClosurePublisher(*nh_, *nh_private_));
      case InitModule::mapping:
        init(InitModule::nh);
        std::string toggle_mapping_srv_name;
        nh_private_->param<std::string>("toggle_mapping_srv_name",
                                        toggle_mapping_srv_name,
                                        toggle_mapping_srv_name);
        if (toggle_mapping_srv_name.size()) {
          toggle_mapping_srv_ =
              nh_->serviceClient<std_srvs::SetBool>(toggle_mapping_srv_name);
        }
    }
    initialized_[init_module] = true;
  }

  TfPublisher::Ptr tf_pub_;
  LoopClosurePublisher::Ptr loop_closure_pub_;

  ros::ServiceClient toggle_mapping_srv_;
};

static VIOInterface vio_interface;

void updatePose(Eigen::Matrix4d pose, double timestamp) {
  vio_interface.updatePose(pose, timestamp);
}

void updatePose(cv::Mat pose, double timestamp) {
  vio_interface.updatePose(pose, timestamp);
}

void publishLoopClosure(size_t from_client_id, double from_timestamp,
                        size_t to_client_id, double to_timestamp,
                        Eigen::Matrix4d T_A_B) {
  vio_interface.publishLoopClosure(from_client_id, from_timestamp, to_client_id,
                                   to_timestamp, T_A_B);
}
void publishLoopClosure(size_t from_client_id, double from_timestamp,
                        size_t to_client_id, double to_timestamp, cv::Mat R,
                        cv::Mat t) {
  vio_interface.publishLoopClosure(from_client_id, from_timestamp, to_client_id,
                                   to_timestamp, R, t);
}
void publishLoopClosure(const double& from_timestamp,
                        const double& to_timestamp, Eigen::Matrix4d T_A_B) {
  vio_interface.publishLoopClosure(from_timestamp, to_timestamp, T_A_B);
}
void publishLoopClosure(const double& from_timestamp,
                        const double& to_timestamp, cv::Mat R, cv::Mat t) {
  vio_interface.publishLoopClosure(from_timestamp, to_timestamp, R, t);
}
bool toggleMapping(bool b_mapping) {
  return vio_interface.toggleMapping(b_mapping);
}

}  // namespace mod
}  // namespace coxgraph
