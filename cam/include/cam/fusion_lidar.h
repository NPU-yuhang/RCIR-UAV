#ifndef FUSION_LIDAR_H
#define FUSION_LIDAR_H
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
//system
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "cam/mono_lidar_config.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180*M_PI)

class fusion_lidar
{
public:
    fusion_lidar();
    ~fusion_lidar();
    void FusionCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::LaserScanConstPtr &scan);
    void setCameraParam(param param_camera);

public:
    float h = 0.03;
    param fusion_param = param();

}

#endif // FUSION_LIDAR_H
