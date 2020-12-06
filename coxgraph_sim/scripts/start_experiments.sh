#! /usr/bin/env sh

rosservice call /tsdf_client/label_tsdf_client_0/toggle_mapping "data: True"
rosservice call /tsdf_client/label_tsdf_client_1/toggle_mapping "data: True"

sleep 3

rosservice call /planner_0/planner_node/toggle_running "data: True"
rosservice call /planner_1/planner_node/toggle_running "data: True"