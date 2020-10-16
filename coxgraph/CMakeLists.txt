cmake_minimum_required(VERSION 2.8.3)
project(coxgraph)

add_definitions(-std=c++14 -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
    src/client/coxgraph_client.cpp
    src/server/coxgraph_server.cpp
    src/server/client_handler.cpp
    src/server/pose_graph_interface.cpp)

cs_add_executable(coxgraph_client_node
    src/coxgraph_client_node.cpp)
target_link_libraries(coxgraph_client_node ${PROJECT_NAME})

cs_add_executable(coxgraph_server_node
    src/coxgraph_server_node.cpp)
target_link_libraries(coxgraph_server_node ${PROJECT_NAME})

cs_export()