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
using namespace cv;
using namespace std;

void image_callback(const sensor_msgs::ImageConstPtr &img);

cam::vision_msg vision_location();
void Threshold_Demo( int, void* );

ros::Publisher vision_pub;

Mat src_gray, src;
Mat threshold_output;

RNG rng(12345);
//Scalar colorful = CV_RGB(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));

Point2f Center_cal(vector<vector<Point> > contours, int i)//找到所提取轮廓的中心点
{
    int centerx = 0, centery = 0, n = contours[i].size();
    //在提取的小正方形的边界上每隔周长个像素提取一个点的坐标，求所提取四个点的平均坐标（即为小正方形的大致中心）
    centerx = (contours[i][n / 4].x + contours[i][n * 2 / 4].x + contours[i][3 * n / 4].x + contours[i][n - 1].x) / 4;
    centery = (contours[i][n / 4].y + contours[i][n * 2 / 4].y + contours[i][3 * n / 4].y + contours[i][n - 1].y) / 4;
    Point2f point1 = Point(centerx, centery);
    return point1;
}
