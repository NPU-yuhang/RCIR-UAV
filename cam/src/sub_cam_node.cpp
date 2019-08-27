#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

cv::VideoWriter writer;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
//sensor_msgs::Image ROS中image传递的消息形式
  try  
  {  
    cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);

    writer<<cv_bridge::toCvShare(msg, "mono8")->image;

    cv::waitKey(30);  
  }  
  catch (cv_bridge::Exception& e)  
  {  
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }  
}  

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_sub_node");  
  ros::NodeHandle nh;  
  cv::namedWindow("view");  
  cv::startWindowThread();  
  cv::Size size = cv::Size(640, 360);
  writer.open("/media/nvidia/SD/flyvideo/a2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, size, false);
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe("/my_stereo/left/image_raw", 1, imageCallback);
  ros::spin();  
  cv::destroyWindow("view");  //窗口
  writer.release();
}
