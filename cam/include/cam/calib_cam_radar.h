#ifndef CALIB_CAM_RADAR_H
#define CALIB_CAM_RADAR_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <my_radar/object.h>
#include <my_radar/objects.h>
#include "cam/waveradar2image.h"

int nWaitTime = 0;
cv::Mat radar_data;
cv::Mat img_src;
void calibCallback(const sensor_msgs::ImageConstPtr &img, const my_radar::objectsConstPtr &radar);
void onChange(int value, void* param);
#endif // CALIB_CAM_RADAR_H
