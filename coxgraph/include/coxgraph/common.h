#ifndef COXGRAPH_COMMON_H_
#define COXGRAPH_COMMON_H_

#include <voxblox/core/common.h>
#include <voxgraph/common.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace coxgraph {

typedef int8_t ClientId;

using ClientSubmapId = voxgraph::SubmapID;
using ClientSubmapConfig = voxgraph::VoxgraphSubmap::Config;
using ClientSubmap = voxgraph::VoxgraphSubmap;

using Transformation = voxgraph::Transformation;

}  // namespace coxgraph

#endif  // COXGRAPH_COMMON_H_
