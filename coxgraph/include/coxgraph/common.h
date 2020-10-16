#ifndef COXGRAPH_COMMON_H_
#define COXGRAPH_COMMON_H_

#include <voxblox/core/common.h>
#include <voxgraph/common.h>
#include <voxgraph/frontend/frame_names.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace coxgraph {

typedef int8_t CliId;

using CliSm = voxgraph::VoxgraphSubmap;
using SerSmId = voxgraph::SubmapID;
using CliSmId = voxgraph::SubmapID;
using CliSmConfig = voxgraph::VoxgraphSubmap::Config;

using Transformation = voxgraph::Transformation;

using FrameNames = voxgraph::FrameNames;

struct TimeLine {
  TimeLine() : start(0), end(0) {}
  ros::Time start;
  ros::Time end;
  bool hasTime(const ros::Time& time) {
    if (end.isZero()) return false;
    if (time > start && time < end) {
      return true;
    }
    return false;
  }
  bool update(const ros::Time& new_start, const ros::Time& new_end) {
    if (start != new_start || end != new_end) {
      start = new_start;
      end = new_end;
      return true;
    }
    return false;
  }
};
}  // namespace coxgraph

#endif  // COXGRAPH_COMMON_H_
