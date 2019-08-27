#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "pub_left_cam" );
  ros::NodeHandle n;

  // Open camera with CAMERA_INDEX (webcam is typically #0).
  const int CAMERA_INDEX = 0;
  const int CAMERA_INDEX2 = 1;
  ros::Rate loop_rate(20);
  cv::VideoCapture capture( CAMERA_INDEX ); //摄像头视频的读操作
  cv::VideoCapture capture2( CAMERA_INDEX2 );
  if( not capture.isOpened() )
  {
    ROS_ERROR_STREAM(
      "Failed to open camera with index " << "/media/nvidia/SD/flyvideo/2222.avi" << "!"
    );
    ros::shutdown();
  }

  else{
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    capture2.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture2.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  }

  //1 捕获视频

  //2 创建ROS中图像的发布者
  image_transport::ImageTransport it( n );
  image_transport::Publisher pub_image_left_BGR = it.advertise( "camera_left_BGR", 1 );
  image_transport::Publisher pub_image_right_BGR = it.advertise("camera_right_BGR", 1);

  //cv_bridge功能包提供了ROS图像和OpenCV图像转换的接口，建立了一座桥梁
  cv_bridge::CvImagePtr frame_BGR_left = boost::make_shared< cv_bridge::CvImage >();
  frame_BGR_left->encoding = sensor_msgs::image_encodings::BGR8;

  cv_bridge::CvImagePtr frame_BGR_right = boost::make_shared< cv_bridge::CvImage >();
  frame_BGR_right->encoding = sensor_msgs::image_encodings::BGR8;

  while( ros::ok() ) {
    capture >> frame_BGR_left->image; //流的转换
    capture2 >>frame_BGR_right->image;

    if( frame_BGR_left->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame!" );
      ros::shutdown();
    }
    //打成ROS数据包
    ros::Time t = ros::Time::now();
    frame_BGR_left->header.stamp = t;
    pub_image_left_BGR.publish(frame_BGR_left->toImageMsg());
    frame_BGR_right->header.stamp = t;
    pub_image_right_BGR.publish(frame_BGR_right->toImageMsg());

    cv::waitKey( 1 );//opencv刷新图像 3ms
    ros::spinOnce();
    loop_rate.sleep();
  }

  capture.release();  //释放流
  return EXIT_SUCCESS;
}
