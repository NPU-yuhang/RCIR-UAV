cmake_minimum_required(VERSION 2.8.3)
PROJECt(cam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  camera_info_manager
  message_generation
)

find_package(OpenCV 3.3.0 QUIET)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    find_package(OpenCVContribImgProc QUIET)
    find_package(OpenCVViz QUIET)
else()
    find_package(OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, depth percpetion demo requires 3.3.0+ for better experience")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()

find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_message_files(
  DIRECTORY msg
  FILES
  vision_msg.msg
  vision_msgs.msg
  point.msg
  mono_radar_fusion.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub_cam_node src/pub_cam_node.cpp)
target_link_libraries(pub_cam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pub_cam_node cam_genetate_messages_cpp)

add_executable(pub_mono_cam src/pub_mono_cam.cpp)
target_link_libraries(pub_mono_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pub_mono_cam cam_genetate_messages_cpp)

add_executable(pub_left_cam src/pub_left_cam.cpp)
target_link_libraries(pub_left_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pub_right_cam src/pub_right_cam.cpp)
target_link_libraries(pub_right_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(vision_land src/vision_land.cpp)
target_link_libraries(vision_land ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(vision_land2 src/vision_land2.cpp)
target_link_libraries(vision_land2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(obstacle_detection src/obstacle_detection.cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mono_lidar_fusion src/mono_lidar_fusion.cpp)
target_link_libraries(mono_lidar_fusion ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(calib_cam_radar src/calib_cam_radar.cpp)
target_link_libraries(calib_cam_radar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
