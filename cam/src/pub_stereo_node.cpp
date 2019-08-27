#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace camera_info_manager;

int main( int argc, char **argv )
{
  ros::init( argc, argv, "pub_stereo_node" );
  ros::NodeHandle n;
  ros::NodeHandle nhleft("/pubimage/left");
  ros::NodeHandle nhright("/pubimage/right");

  // Open camera with CAMERA_INDEX (webcam is typically #0).
  const int CAMERA_INDEX = 1;
  cv::VideoCapture capture( CAMERA_INDEX ); //摄像头视频的读操作
  if( not capture.isOpened() )
  {
    ROS_ERROR_STREAM(
      "Failed to open camera with index " << CAMERA_INDEX << "!"
    );
    ros::shutdown();
  }

  else{
    capture.set(CV_CAP_PROP_FRAME_WIDTH,2560);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  }

  camera_info_manager::CameraInfoManager *LInfo;
  camera_info_manager::CameraInfoManager *RInfo;

  LInfo = new camera_info_manager::CameraInfoManager (nhleft, "pgr_camera_13306261", "file:///home/nvidia/catkin_ws1/src/publish_images/calibration/cam0_params.yaml");
  RInfo = new camera_info_manager::CameraInfoManager (nhright, "pgr_camera_13330289", "file:///home/jay/catkin_ws/src/publish_images/calibration/cam1_params.yaml");

  //1 捕获视频

  //2 创建ROS中图像的发布者
  image_transport::ImageTransport it( n ); 
  image_transport::Publisher pub_image = it.advertise( "camera", 1 );

  image_transport::ImageTransport it_L( n ); 
  image_transport::Publisher pub_image_L = it_L.advertise( "/my_stereo/left/image_raw", 1 );

  image_transport::ImageTransport it_R( n ); 
  image_transport::Publisher pub_image_R = it_R.advertise( "/my_stereo/right/image_raw", 1 );

  //cv_bridge功能包提供了ROS图像和OpenCV图像转换的接口，建立了一座桥梁
  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
  cv_bridge::CvImagePtr frame_L = boost::make_shared< cv_bridge::CvImage >();
  cv_bridge::CvImagePtr frame_R = boost::make_shared< cv_bridge::CvImage >();
  frame->encoding = sensor_msgs::image_encodings::BGR8;
  frame_L->encoding = sensor_msgs::image_encodings::BGR8;
  frame_R->encoding = sensor_msgs::image_encodings::BGR8;

  capture >> frame->image; //流的转换
  double fScale = 0.5;  
  cv::Size dsize = cv::Size(frame->image.cols*fScale, frame->image.rows*fScale);
  cv::Mat imagedst = cv::Mat(dsize, CV_32S);
  resize(frame->image, imagedst, dsize);

  while( ros::ok() ) {
    capture >> frame->image; //流的转换
    resize(frame->image, imagedst, dsize);
    frame_L->image = imagedst(cv::Rect(0, 0, 640, 360));
    frame_R->image = imagedst(cv::Rect(640, 0, 640, 360));
    

    if( frame->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame!" );
      ros::shutdown();
    }
    //打成ROS数据包
    frame->header.stamp = ros::Time::now();
    pub_image.publish( frame->toImageMsg() );

    if( frame_L->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame_L!" );
      ros::shutdown();
    }
    //打成ROS数据包
    frame_L->header.stamp = ros::Time::now();
    pub_image_L.publish( frame_L->toImageMsg() );

    if( frame_R->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame_R!" );
      ros::shutdown();
    }
    //打成ROS数据包
    frame_R->header.stamp = ros::Time::now();
    pub_image_R.publish( frame_R->toImageMsg() );

    cv::waitKey( 30 );//opencv刷新图像 3ms
    ros::spinOnce();
  }

  capture.release();  //释放流
  return EXIT_SUCCESS;
}
