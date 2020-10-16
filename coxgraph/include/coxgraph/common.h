#ifndef COXGRAPH_COMMON_H_
#define COXGRAPH_COMMON_H_

#include <voxblox/core/common.h>
#include <voxgraph/common.h>
#include <voxgraph/frontend/frame_names.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace coxgraph {

typedef int8_t CliId;

using CliSm = voxgraph::VoxgraphSubmap;
using CliSmId = voxgraph::SubmapID;
using CliSmConfig = voxgraph::VoxgraphSubmap::Config;

using Transformation = voxgraph::Transformation;

using FrameNames = voxgraph::FrameNames;
}  // namespace coxgraph

#endif  // COXGRAPH_COMMON_H_
