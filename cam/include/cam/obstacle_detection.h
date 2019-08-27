#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <cam/vision_msg.h>
#include <cam/vision_msgs.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace std;

int row, col;
int threshold_value = 40;
const string T_value = "threshold_value";
const string window_name = "yuchulihou";
Mat dstImage, edge, threshold_output;
int g_nKernelSize=20;//核大小
cam::vision_msgs obstacle;
ros::Publisher obstacle_pub;
image_transport::Publisher obstacle_image;
cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();

void image_callback(const sensor_msgs::ImageConstPtr &img);
void detect(Mat img);
void Threshold_Demo( int, void* );
