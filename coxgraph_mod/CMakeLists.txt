cmake_minimum_required(VERSION 2.8.3)
project(coxgraph_mod)

add_definitions(-std=c++14 -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(vio_interface
  src/vio_interface.cpp)
target_link_libraries(vio_interface
    ${CATKIN_LIBRARIES}
)

cs_add_library(tsdf_recover
  src/mesh_converter.cpp
  src/tsdf_recover.cpp)
target_link_libraries(tsdf_recover
    ${CATKIN_LIBRARIES}
)

cs_add_executable(tsdf_recover_node
  src/tsdf_recover_node.cpp)
target_link_libraries(tsdf_recover_node tsdf_recover) 

##########
# EXPORT #
##########

cs_install()
cs_export()
