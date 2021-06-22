#! /usr/bin/env sh

rosservice call /coxgraph/coxgraph_client_0/finish_map
rosservice call /coxgraph/coxgraph_client_1/finish_map
rosservice call /coxgraph/coxgraph_server_node/get_final_global_mesh "file_path: '/workspaces/voxgraph_melodic_ws/eval/node_evaluator/coxgraph_sever_mesh.ply'"
rosservice call /coxgraph/coxgraph_server_node/get_pose_history "file_path: '/workspaces/voxgraph_melodic_ws/eval/node_evaluator/coxgraph_traj_all.txt'"
