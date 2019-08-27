/*
Author: Monroe Kennedy III
Description:
This node simply takes in the syncronized images from the ELP camera, 
provided the width of the images splits the image into messages for left/right camera (still synchronized).
*/
#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>

//class SplitImage
//{
//  public:
//    SplitImage();
//    const int CAMERA_INDEX = 1;
//  private:
//    ros::NodeHandle nh_;
//    /** image transport interfaces */
//    std::shared_ptr<image_transport::ImageTransport> it_;
////    image_transport::Subscriber sync_image_sub_;
//    std::shared_ptr<image_transport::CameraPublisher> stereo_image_pub_;
//    std::shared_ptr<image_transport::CameraPublisher> left_image_pub_;
//    std::shared_ptr<image_transport::CameraPublisher> right_image_pub_;
//    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_stereo_;
//    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_left_;
//    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_right_;
//    std::string camera_ns; //Camera ns
//    //Path to calibration files
//    std::string calib_path_;
//};

//SplitImage::SplitImage() //: nh_(ros::NodeHandle("~")), it_(std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_))), left_image_pub_(it_->advertise("left/image_raw",1)), right_image_pub_ (it_->advertise("right/image_raw",1))
//{
//  nh_ = ros::NodeHandle("~");
//  it_.reset(new image_transport::ImageTransport(nh_)); //connect image transport to node
//  //sync_image_sub_  = nh_.subscribe("/elp/sync/image_raw", 1, &SplitImage::SyncImageCallback, this);
//  nh_.param<std::string>("camera_ns", camera_ns, "elp"); //Set camera ns
//  left_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_->advertiseCamera("left/image_raw",1));
//  right_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_->advertiseCamera("right/image_raw",1));

//  cv::VideoCapture capture( CAMERA_INDEX ); //摄像头视频的读操作
//  if( not capture.isOpened() )
//  {
//    ROS_ERROR_STREAM(
//      "Failed to open camera with index " << CAMERA_INDEX << "!"
//    );
//    ros::shutdown();
//  }

//  else{
//    capture.set(CV_CAP_PROP_FRAME_WIDTH,2560);
//    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
//  }
//  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
//  cv_bridge::CvImagePtr cv_ptr_left = boost::make_shared< cv_bridge::CvImage >();
//  cv_bridge::CvImagePtr cv_ptr_right = boost::make_shared< cv_bridge::CvImage >();
//  std::string stereo_frame = camera_ns + "_stereo_optical_frame";
//  std::string left_frame = camera_ns + "_left_optical_frame";
//  std::string right_frame = camera_ns + "_right_optical_frame";

//  try
//  {
//    frame->encoding = sensor_msgs::image_encodings::BGR8;
//    cv_ptr_left->encoding = sensor_msgs::image_encodings::BGR8;
//    cv_ptr_right->encoding = sensor_msgs::image_encodings::BGR8;
//  }
//  catch (cv_bridge::Exception& e)
//  {
//    ROS_ERROR("cv_bridge exception: %s", e.what());
//    return;
//  }

//  capture >> frame->image; //流的转换
//  double fScale = 0.5;
//  cv::Size dsize = cv::Size(frame->image.cols*fScale, frame->image.rows*fScale);
//  cv::Mat imagedst = cv::Mat(dsize, CV_32S);
//  resize(frame->image, imagedst, dsize);

//  int combined_rows = frame->image.rows*fScale;
//  int combined_cols = frame->image.cols*fScale;
//  int image_cols = combined_cols/2;
//  int image_rows = combined_rows;
//  //2. Given the width/height of the combined image, divide into two for the left/right images
////    cv::Rect leftROI(0,0,image_cols,image_rows);
////    cv::Rect rightROI(image_cols,0,image_cols,image_rows);
////    cv::Mat leftcrop  = cv_ptr_left->image(leftROI);
////    cv::Mat rightcrop  = cv_ptr_right->image(rightROI);
//  capture >> frame->image; //流的转换
//  resize(frame->image, imagedst, dsize);
//  cv_ptr_left->image = imagedst(cv::Rect(0, 0, image_cols, image_rows));
//  cv_ptr_right->image = imagedst(cv::Rect(image_cols, 0, image_cols, image_rows));

//  if( cv_ptr_left->image.empty() || cv_ptr_right->image.empty())
//  {
//  ROS_ERROR_STREAM( "Failed to capture frame_L!" );
//  ros::shutdown();
//  }
//  //打成ROS数据包
//  frame->header.frame_id = stereo_frame;
//  cv_ptr_left->header.frame_id = left_frame;
//  cv_ptr_right->header.frame_id = right_frame;
//  //Get camera infos
//  sensor_msgs::CameraInfoPtr ci_stereo_(new sensor_msgs::CameraInfo(cinfo_stereo_->getCameraInfo()));
//  sensor_msgs::CameraInfoPtr ci_left_(new sensor_msgs::CameraInfo(cinfo_left_->getCameraInfo()));
//  sensor_msgs::CameraInfoPtr ci_right_(new sensor_msgs::CameraInfo(cinfo_right_->getCameraInfo()));
//  ci_stereo_->header = frame->header;
//  ci_left_->header = cv_ptr_left->header;
//  ci_right_->header = cv_ptr_right->header;
//  //3. Publish the left and right images
//  frame->header.stamp = ros::Time::now();
//  cv_ptr_left->header.stamp = ros::Time::now();
//  cv_ptr_right->header.stamp = ros::Time::now();
//  stereo_image_pub_->publish(frame->toImageMsg(),ci_stereo_);
//  left_image_pub_->publish(cv_ptr_left->toImageMsg(),ci_left_);
//  right_image_pub_->publish(cv_ptr_right->toImageMsg(),ci_right_);

//  cv::waitKey( 30 );//opencv刷新图像 3ms
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "split_sync_images");
  ROS_INFO("Split syncronized elp image node running");

  const int CAMERA_INDEX = 1;
  ros::NodeHandle nh_;
  /** image transport interfaces */
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::ImageTransport> it_L;
  std::shared_ptr<image_transport::ImageTransport> it_R;
//    image_transport::Subscriber sync_image_sub_;
  std::shared_ptr<image_transport::CameraPublisher> stereo_image_pub_;
  std::shared_ptr<image_transport::CameraPublisher> left_image_pub_;
  std::shared_ptr<image_transport::CameraPublisher> right_image_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_stereo_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_left_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_right_;
  std::string camera_ns; //Camera ns
  //Path to calibration files

  nh_ = ros::NodeHandle("~");
  it_.reset(new image_transport::ImageTransport(nh_)); //connect image transport to node
  it_L.reset(new image_transport::ImageTransport(nh_));
  it_R.reset(new image_transport::ImageTransport(nh_));
  //sync_image_sub_  = nh_.subscribe("/elp/sync/image_raw", 1, &SplitImage::SyncImageCallback, this);
  nh_.param<std::string>("camera_ns", camera_ns, "elp"); //Set camera ns
  stereo_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_->advertiseCamera("stereo/image_raw",1));
  left_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_L->advertiseCamera("left/image_raw",1));
  right_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_R->advertiseCamera("right/image_raw",1));

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
  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
  cv_bridge::CvImagePtr cv_ptr_left = boost::make_shared< cv_bridge::CvImage >();
  cv_bridge::CvImagePtr cv_ptr_right = boost::make_shared< cv_bridge::CvImage >();
  std::string stereo_frame = camera_ns + "_stereo_optical_frame";
  std::string left_frame = camera_ns + "_left_optical_frame";
  std::string right_frame = camera_ns + "_right_optical_frame";

  try
  {
    frame->encoding = sensor_msgs::image_encodings::BGR8;
    cv_ptr_left->encoding = sensor_msgs::image_encodings::BGR8;
    cv_ptr_right->encoding = sensor_msgs::image_encodings::BGR8;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  capture >> frame->image; //流的转换
  double fScale = 0.5;
  cv::Size dsize = cv::Size(frame->image.cols*fScale, frame->image.rows*fScale);
  cv::Mat imagedst = cv::Mat(dsize, CV_32S);
  resize(frame->image, imagedst, dsize);

  int combined_rows = frame->image.rows*fScale;
  int combined_cols = frame->image.cols*fScale;
  int image_cols = combined_cols/2;
  int image_rows = combined_rows;
  //2. Given the width/height of the combined image, divide into two for the left/right images
//    cv::Rect leftROI(0,0,image_cols,image_rows);
//    cv::Rect rightROI(image_cols,0,image_cols,image_rows);
//    cv::Mat leftcrop  = cv_ptr_left->image(leftROI);
//    cv::Mat rightcrop  = cv_ptr_right->image(rightROI);
  while(ros::ok())
  {
    capture >> frame->image; //流的转换
    resize(frame->image, imagedst, dsize);
    cv_ptr_left->image = imagedst(cv::Rect(0, 0, image_cols, image_rows));
    cv_ptr_right->image = imagedst(cv::Rect(image_cols, 0, image_cols, image_rows));

    if( cv_ptr_left->image.empty() || cv_ptr_right->image.empty())
    {
    ROS_ERROR_STREAM( "Failed to capture frame_L!" );
    ros::shutdown();
    }
    //打成ROS数据包
    frame->header.frame_id = stereo_frame;
    cv_ptr_left->header.frame_id = left_frame;
    cv_ptr_right->header.frame_id = right_frame;
    //Get camera infos
    sensor_msgs::CameraInfoPtr ci_stereo_(new sensor_msgs::CameraInfo(cinfo_stereo_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr ci_left_(new sensor_msgs::CameraInfo(cinfo_left_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr ci_right_(new sensor_msgs::CameraInfo(cinfo_right_->getCameraInfo()));
    ci_stereo_->header = frame->header;
    ci_left_->header = cv_ptr_left->header;
    ci_right_->header = cv_ptr_right->header;
    //3. Publish the left and right images
    frame->header.stamp = ros::Time::now();
    cv_ptr_left->header.stamp = ros::Time::now();
    cv_ptr_right->header.stamp = ros::Time::now();
    stereo_image_pub_->publish(frame->toImageMsg(),ci_stereo_);
    left_image_pub_->publish(cv_ptr_left->toImageMsg(),ci_left_);
    right_image_pub_->publish(cv_ptr_right->toImageMsg(),ci_right_);

    cv::waitKey( 30 );//opencv刷新图像 3ms
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}
