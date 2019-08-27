#include "chrono"
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>

#include "stereo_utility/stereo_frame.hpp"

#include "darknet_ros_msgs/BoundingBoxes.h"

#include <cv_bridge/cv_bridge.h>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

void objectDetectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg, M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void displayObjectPtCloudCallback(const sensor_msgs::ImageConstPtr& img_left, const sensor_msgs::ImageConstPtr& img_right, const darknet_ros_msgs::BoundingBoxesConstPtr& b_box, M210_STEREO::StereoFrame::Ptr& stereo_frame_ptr);

void visualizeRectImgHelper(M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void visualizeDisparityMapHelper(M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void shutDownHandler(int s);

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
