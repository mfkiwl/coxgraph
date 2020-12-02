cmake_minimum_required(VERSION 2.8.3)
project(coxgraph_sim)

add_definitions(-std=c++14 -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(odometry_transform_publisher
  src/odometry_transform_publisher.cpp)
target_link_libraries(odometry_transform_publisher
    ${CATKIN_LIBRARIES}
)

cs_add_executable(odometry_transform_publisher_node
  src/odometry_transform_publisher_node.cpp)
target_link_libraries(odometry_transform_publisher_node odometry_transform_publisher)

##########
# EXPORT #
##########

cs_install()
cs_export()
