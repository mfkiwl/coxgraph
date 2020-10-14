#ifndef COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
#define COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_

#include <voxgraph/frontend/voxgraph_mapper.h>

#include "coxgraph/common.h"

namespace coxgraph {

using VoxgraphMapper = voxgraph::VoxgraphMapper;
class CoxgraphClient : public VoxgraphMapper {
 public:
  CoxgraphClient(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : VoxgraphMapper(nh, nh_private) {
    int client_id;
    nh_private.param<int>("client_id", client_id, -1);
    client_id_ = static_cast<ClientId>(client_id);
    if (client_id_ < 0) {
      LOG(FATAL) << "Invalid Client Id " << client_id_;
    } else {
      LOG(INFO) << "Started Coxgraph Client " << client_id_;
    }
  }
  ~CoxgraphClient() = default;

  inline const ClientId& getClientId() const { return client_id_; }

 private:
  ClientId client_id_;
};

}  // namespace coxgraph

#endif  // COXGRAPH_CLIENT_COXGRAPH_CLIENT_H_
