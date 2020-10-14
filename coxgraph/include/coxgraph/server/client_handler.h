#ifndef COXGRAPH_SERVER_CLIENT_HANDLER_H_
#define COXGRAPH_SERVER_CLIENT_HANDLER_H_

#include <memory>

#include "coxgraph/common.h"

namespace coxgraph {
namespace server {

class ClientHandler {
 public:
  typedef std::shared_ptr<ClientHandler> Ptr;

  explicit ClientHandler(const ClientId& client_id) : client_id_(client_id) {}
  virtual ~ClientHandler() = default;

 private:
  const ClientId client_id_;
};

}  // namespace server
}  // namespace coxgraph

#endif  // COXGRAPH_SERVER_CLIENT_HANDLER_H_
