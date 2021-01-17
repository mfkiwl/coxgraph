#! /usr/bin/env sh

rosservice call /coxgraph/tsdf_client_0/toggle_mapping "data: True"
rosservice call /coxgraph/tsdf_client_1/toggle_mapping "data: True"

rosservice call /planner_0/planner_node/toggle_running "data: True"
rosservice call /planner_1/planner_node/toggle_running "data: True"