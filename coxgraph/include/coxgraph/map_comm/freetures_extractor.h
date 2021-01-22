#ifndef COXGRAPH_MAP_COMM_FREETURES_EXTRACTOR_H_
#define COXGRAPH_MAP_COMM_FREETURES_EXTRACTOR_H_

#include "coxgraph/common.h"

namespace coxgraph {
namespace comm {

class FreeturesExtractor {
 public:
  explicit FreeturesExtractor(const voxblox::TsdfMap::Ptr& tsdf_map)
      : tsdf_map_(tsdf_map) {}
  virtual ~FreeturesExtractor() = default;

 private:
  voxblox::TsdfMap::Ptr tsdf_map_;
};

}  // namespace comm
}  // namespace coxgraph

#endif  // COXGRAPH_MAP_COMM_FREETURES_EXTRACTOR_H_
