#ifndef MONO_LIDAR_FUSION_H
#define MONO_LIDAR_FUSION_H
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
//system
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include "cam/mono_lidar_config.h"
#include <cam/vision_msg.h>
#include <cam/vision_msgs.h>
#include <cam/point.h>
#include <cam/mono_radar_fusion.h>
#include "cam/waveradar2image.h"

#include <my_radar/object.h>
#include <my_radar/objects.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180*M_PI)
float h = 0.07;
param fusion_param = param();
CameraParam camParam;
waveradar2image radar2image = waveradar2image();

ros::Publisher obstacle_info_pub;

void FusionCallback(const sensor_msgs::ImageConstPtr &img, const my_radar::objectsConstPtr &radar, const cam::vision_msgsConstPtr &obstacle);
void setCameraParam(param param_camera);

cv::Mat intrinsic;//相机内参数
cv::Mat distortion_coeff;//相机畸变参数

#endif // MONO_LIDAR_FUSION_H
