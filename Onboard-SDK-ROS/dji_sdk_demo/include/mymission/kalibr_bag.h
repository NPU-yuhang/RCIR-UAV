#ifndef KALIBR_BAG_H
#define KALIBR_BAG_H
#include "mymission/Mission.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <cam/vision_msg.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void imu_Callback(const sensor_msgs::ImuConstPtr &imu_msg);
void img_Callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right);

#endif // KALIBR_BAG_H
