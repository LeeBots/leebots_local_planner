cmake_minimum_required(VERSION 2.8.3)
project(leebots_local_planner)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    control_toolbox
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    )

  find_package(Eigen3)
  if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# generate_dynamic_reconfigure_options(cfg/EBandPlanner.cfg)

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros    
    dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES leebots_local_planner
)

# build the library
add_library(leebots_local_planner
  src/leebots_local_planner_ros.cpp
)
add_dependencies(leebots_local_planner ${catkin_EXPORTED_TARGETS})

target_link_libraries(leebots_local_planner ${catkin_LIBRARIES})

install(
  DIRECTORY include/leebots_local_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS leebots_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES lbp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
