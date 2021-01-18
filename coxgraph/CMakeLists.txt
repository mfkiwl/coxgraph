cmake_minimum_required(VERSION 2.8.3)
project(coxgraph)

add_definitions(-std=c++14 -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories("thirdparty/DBoW2")

# Thirdparty files
set(thrdprtysrc
  thirdparty/DBoW2/DBoW2/BowVector.cpp
  thirdparty/DBoW2/DBoW2/FBRISK.cpp
  thirdparty/DBoW2/DBoW2/FeatureVector.cpp
  thirdparty/DBoW2/DBoW2/FORB.cpp
  thirdparty/DBoW2/DBoW2/ScoringObject.cpp
  thirdparty/DBoW2/DUtils/Random.cpp
  thirdparty/DBoW2/DUtils/Timestamp.cpp
)

cs_add_library(${PROJECT_NAME}
    src/client/coxgraph_client.cpp
    src/client/map_server.cpp
    src/server/coxgraph_server.cpp
    src/server/pose_graph_interface.cpp
    src/server/global_tf_controller.cpp
    src/server/client_tf_optimizer.cpp
    src/server/visualizer/server_visualizer.cpp
    src/map_comm/submap_collection.cpp
    src/map_comm/client_handler.cpp
    src/map_comm/mesh_converter.cpp
    src/map_comm/tsdf_recover.cpp
    ${thrdprtysrc})

cs_add_executable(coxgraph_client_node
    src/coxgraph_client_node.cpp)
target_link_libraries(coxgraph_client_node ${PROJECT_NAME})

cs_add_executable(coxgraph_server_node
    src/coxgraph_server_node.cpp)
target_link_libraries(coxgraph_server_node ${PROJECT_NAME})

cs_add_executable(tsdf_recover_node
    src/tsdf_recover_node.cpp)
target_link_libraries(tsdf_recover_node ${PROJECT_NAME})

cs_add_executable(voxblox_eval_node
    src/voxblox_eval_node.cpp)
target_link_libraries(voxblox_eval_node ${PROJECT_NAME})

cs_add_executable(keyframe_publisher_node
    src/keyframe_publisher_node.cpp)
target_link_libraries(keyframe_publisher_node ${PROJECT_NAME})

cs_export()
