#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>

#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

#include "darknet_ros_msgs/BoundingBoxes.h"

#include "stereo_utility/stereo_frame.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
  std::cout<<"yuh"<<std::endl;
//sensor_msgs::Image ROS中image传递的消息形式
  try  
  {  
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);  
    cv::waitKey(30);  
  }  
  catch (cv_bridge::Exception& e)  
  {  
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }  
}

void myCallback(const sensor_msgs::ImageConstPtr& image_left, const sensor_msgs::ImageConstPtr& image_right, const darknet_ros_msgs::BoundingBoxesConstPtr& b_box) 
{
  std::cout<<"yuh"<<std::endl;
  //sensor_msgs::Image ROS中image传递的消息形式
  try  
  {  
    cv::imshow("view", cv_bridge::toCvShare(image_left, "bgr8")->image);  
    cv::waitKey(30);  
  }  
  catch (cv_bridge::Exception& e)  
  {  
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_left->encoding.c_str());  
  } 
}

void bboxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& b_box)
{
  std::cout<<"yuh"<<std::endl;
  ROS_INFO("Got %d detections", (int)b_box->bounding_boxes.size());
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "get_boundingboxes");
  ros::NodeHandle n;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> topic_synchronizer;

  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>* bbox_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_left_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_right_sub;
  message_filters::Synchronizer<topic_synchronizer>* sync;

  bbox_sub = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(n, "/darknet_ros/bounding_boxes", 1);
  img_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(n,"camera_L",1);
  img_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(n,"camera_R",1);

  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(20), *img_left_sub, *img_right_sub, *bbox_sub);

  sync->registerCallback(boost::bind(&myCallback,_1,_2,_3));
  //bbox_sub.registerCallback(bboxCallback);
  //img_right_sub.registerCallback(imageCallback);
  ros::spin();
  return 0;
}
