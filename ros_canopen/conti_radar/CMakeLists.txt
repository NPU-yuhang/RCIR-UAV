cmake_minimum_required(VERSION 2.8.3)
project(conti_radar)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS #roscpp rospy std_msgs
  can_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(conti_radar_lib
        src/object_extended_info_60d.cpp
        src/object_quality_info_60c.cpp
        src/object_general_info_60b.cpp
        src/object_list_status_60a.cpp
        src/radar_state_201.cpp
        src/clusters_quality_info_702.cpp
        src/clusters_general_info_701.cpp
        src/clusters_list_status_600.cpp
)

install(TARGETS conti_radar_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
