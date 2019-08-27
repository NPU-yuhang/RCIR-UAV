#ifndef DEMO_ADVANCED_SENSING_DEPTH_PERCEPTION_H
#define DEMO_ADVANCED_SENSING_DEPTH_PERCEPTION_H

// System includes
#include "chrono"
#include <signal.h>

// DJI SDK includes
//#include "dji_sdk/StereoVGASubscription.h"
//#include "dji_sdk/Stereo240pSubscription.h"
//#include "dji_sdk/StereoDepthSubscription.h"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/PointCloud2.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
// Utility includes

#include <iostream>
#include <fstream>
#include <string>
#include <tf/tf.h>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

void StereoImgCallback(const sensor_msgs::ImageConstPtr &img_left);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &gps);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &atti);

void pos_callback(const geometry_msgs::PointStamped::ConstPtr &pos);

void velo_callback(const geometry_msgs::Vector3Stamped::ConstPtr &velo);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
#endif //DEMO_ADVANCED_SENSING_DEPTH_PERCEPTION_H
