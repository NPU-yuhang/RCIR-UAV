cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_demo)

# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        dji_sdk
        image_transport # for message_filter
        cv_bridge
        )

find_package(DJIOSDK REQUIRED)
find_package(Pangolin REQUIRED)

#find_package(darknet_ros QUIET)
#if(darknet_ros_FOUND)
#    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
#    add_definitions(-DUSE_DARKNET_ROS)
#endif()

# Try to see if user has OpenCV installed
# If there's version 3.3.0+, depth perception samples will be compiled
# If it's not version 3.3.0+, the sample will not be compiled
find_package(OpenCV 3.3.0 QUIET)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED TRUE)

    find_package(OpenCVContribImgProc QUIET)

    find_package(OpenCVViz QUIET)
else()
    find_package(OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, depth percpetion demo requires 3.3.0+ for better experience")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        add_definitions(-DOPEN_CV_INSTALLED)
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()

find_package(AdvancedSensing QUIET)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_executable(demo_mission
        src/demo_mission.cpp)
target_link_libraries(demo_mission
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_mission dji_sdk_generate_messages_cpp)

add_executable(demo_camera_gimbal
        src/demo_camera_gimbal.cpp )
target_link_libraries(demo_camera_gimbal
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_camera_gimbal dji_sdk_generate_messages_cpp)

add_executable(demo_mfio
        src/demo_mfio.cpp)
target_link_libraries(demo_mfio
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_mfio dji_sdk_generate_messages_cpp)

add_executable(demo_flight_control
        src/demo_flight_control.cpp)

target_link_libraries(demo_flight_control
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_flight_control dji_sdk_generate_messages_cpp)

add_executable(demo_mobile_comm
        src/demo_mobile_comm.cpp)

target_link_libraries(demo_mobile_comm
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_mobile_comm dji_sdk_generate_messages_cpp)

add_executable(demo_local_position_control
        src/demo_local_position_control.cpp)
target_link_libraries(demo_local_position_control
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_local_position_control dji_sdk_generate_messages_cpp)

add_executable(caitu_lena
               src/mymission/caitu_lena.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(caitu_lena
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(caitu_new_cam
               src/mymission/caitu_new_cam.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(caitu_new_cam
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

FILE(GLOB MISSION_PLANNING_FILE
        src/mymission/track_planning.cpp
        src/mymission/flydata.cpp
        src/mymission/Mission.cpp
        src/mymission/artifical_potencial_field.cpp
        )
add_executable(track
               src/mymission/track.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(track
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(track dji_sdk_generate_messages_cpp)

add_executable(auto_landing_task
               src/mymission/auto_landing_task.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(auto_landing_task
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(auto_landing_task dji_sdk_generate_messages_cpp)

add_executable(obstacle_avoidance_task
               src/mymission/obstacle_avoidance_task.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(obstacle_avoidance_task
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(obstacle_avoidance_task dji_sdk_generate_messages_cpp)

add_executable(path_planning
               src/mymission/path_planning.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(path_planning
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )
add_dependencies(path_planning dji_sdk_generate_messages_cpp)

add_executable(avoid_obsta_from_mono_radar
               src/mymission/avoid_obsta_from_mono_radar.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(avoid_obsta_from_mono_radar
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )
add_dependencies(avoid_obsta_from_mono_radar dji_sdk_generate_messages_cpp)

add_executable(kalibr_bag
               src/mymission/kalibr_bag.cpp
               ${MISSION_PLANNING_FILE}
               )
target_link_libraries(kalibr_bag
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )
add_dependencies(kalibr_bag dji_sdk_generate_messages_cpp)

#add_executable(fly_with_vision src/mymission/fly_with_vision.cpp)
#target_link_libraries(fly_with_vision
#        ${catkin_LIBRARIES}
#        ${DJIOSDK_LIBRARIES}
#        )
#add_dependencies(fly_with_vision dji_sdk_generate_messages_cpp)

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ AND FOUND_ADVANCED_SENSING)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, and advanced sensing module, depth percpetion demo will be compiled")

    FILE(GLOB STEREO_UTILITY_SRC_FILE
            src/stereo_utility/camera_param.cpp
            src/stereo_utility/stereo_frame.cpp
            src/stereo_utility/config.cpp
            src/stereo_utility/point_cloud_viewer.cpp)

    add_executable(demo_stereo_depth_perception
            src/demo_stereo_depth_perception.cpp
            ${STEREO_UTILITY_SRC_FILE} )
    target_link_libraries(demo_stereo_depth_perception
            ${catkin_LIBRARIES}
            ${DJIOSDK_LIBRARIES}
            ${OpenCV_LIBS}
            )
    add_dependencies(demo_stereo_depth_perception dji_sdk_generate_messages_cpp)
endif()

#if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ AND darknet_ros_FOUND AND FOUND_ADVANCED_SENSING)
#    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, darknet_ros, and advanced sensing module, depth percpetion demo will be compiled")

#    add_executable(demo_stereo_object_depth_perception
#            src/demo_stereo_object_depth_perception.cpp
#            ${STEREO_UTILITY_SRC_FILE} )
#    target_link_libraries(demo_stereo_object_depth_perception
#            ${catkin_LIBRARIES}
#            ${DJIOSDK_LIBRARIES}
#            ${OpenCV_LIBS}
#            )
#    add_dependencies(demo_stereo_object_depth_perception dji_sdk_generate_messages_cpp)
#endif()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
