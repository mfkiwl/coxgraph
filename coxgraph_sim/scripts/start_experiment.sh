#! /usr/bin/env sh

rosservice call /planner_0/planner_node/toggle_running "data: True"
rosservice call /planner_1/planner_node/toggle_running "data: True"