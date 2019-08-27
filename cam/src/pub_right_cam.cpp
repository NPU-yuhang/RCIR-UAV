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
  ros::init( argc, argv, "pub_right_cam" );
  ros::NodeHandle n;

  // Open camera with CAMERA_INDEX (webcam is typically #0).
  const int CAMERA_INDEX = 1;
  ros::Rate loop_rate(30);
  cv::VideoCapture capture( CAMERA_INDEX ); //摄像头视频的读操作
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
  }

  //1 捕获视频

  //2 创建ROS中图像的发布者
  image_transport::ImageTransport it( n );
  image_transport::Publisher pub_image = it.advertise( "camera_right", 1 );

  image_transport::ImageTransport it_BGR( n );
  image_transport::Publisher pub_image_BGR = it_BGR.advertise( "camera_right_BGR", 1 );

  //cv_bridge功能包提供了ROS图像和OpenCV图像转换的接口，建立了一座桥梁
  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
  cv_bridge::CvImagePtr frame_BGR = boost::make_shared< cv_bridge::CvImage >();
  frame->encoding = sensor_msgs::image_encodings::MONO8;
  frame_BGR->encoding = sensor_msgs::image_encodings::BGR8;

  while( ros::ok() ) {
    capture >> frame_BGR->image; //流的转换

    if( frame_BGR->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame!" );
      ros::shutdown();
    }
    cv::cvtColor(frame_BGR->image,frame->image,CV_BGR2GRAY);
    //打成ROS数据包
    frame_BGR->header.stamp = ros::Time::now();
    pub_image_BGR.publish( frame_BGR->toImageMsg() );
    frame->header.stamp = ros::Time::now();
    //pub_image.publish( frame->toImageMsg() );

    cv::waitKey( 1 );//opencv刷新图像 3ms
    ros::spinOnce();
    //loop_rate.sleep();
  }

  capture.release();  //释放流
  return EXIT_SUCCESS;
}
