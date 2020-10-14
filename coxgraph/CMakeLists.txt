cmake_minimum_required(VERSION 2.8.3)
project(coxgraph)

add_definitions(-std=c++14 -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
    src/client/coxgraph_client.cpp)

cs_add_executable(coxgraph_client_node
    src/coxgraph_client_node.cpp)

cs_add_executable(coxgraph_server_node
    src/coxgraph_server_node.cpp)

cs_export()