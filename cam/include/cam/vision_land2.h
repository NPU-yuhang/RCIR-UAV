#ifndef VISION_LAND2_H
#define VISION_LAND2_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <cam/vision_msg.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

IplImage* src = 0;
IplImage* src_ptr = 0;
IplImage* tmp = 0;
IplImage* tmp1 = 0;
IplImage* org = 0;
Mat src_mat;
void image_callback(const sensor_msgs::ImageConstPtr &img);
int row, col;
cam::vision_msg vision_location();
void Threshold_Demo(int);

ros::Publisher vision_pub;
#endif // VISION_LAND2_H
