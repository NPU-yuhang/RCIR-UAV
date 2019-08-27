cmake_minimum_required(VERSION 2.8.3)
PROJECt(my_radar)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  cv_bridge
  image_transport
  can_msgs
  sensor_msgs
  std_msgs
  camera_info_manager
  message_generation
  socketcan_interface
  socketcan_bridge
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_message_files(
  DIRECTORY msg
  FILES
  object.msg
  objects.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  can_msgs
)

catkin_package(
#  DEPENDS system_lib
)

include_directories(
  include
  ../conti_radar/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(objects_radar_node src/objects_radar_node.cpp)
target_link_libraries(objects_radar_node conti_radar_lib ${catkin_LIBRARIES})

add_executable(clusters_radar_node src/clusters_radar_node.cpp)
target_link_libraries(clusters_radar_node conti_radar_lib ${catkin_LIBRARIES})

add_executable(send_socketcan src/send_socketcan.cpp)
target_link_libraries(send_socketcan ${catkin_LIBRARIES})
