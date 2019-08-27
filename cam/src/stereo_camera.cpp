#include "cam/stereo_camera.hpp"

using namespace M210_STEREO;

bool is_disp_filterd;

//dji_sdk::StereoVGASubscription subscription;
ros::Publisher rect_img_left_publisher;
ros::Publisher rect_img_right_publisher;
ros::Publisher left_disparity_publisher;
ros::Publisher point_cloud_publisher;
ros::Publisher object_info_pub;
visualization_msgs::MarkerArray marker_array;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_camera");
  ros::NodeHandle nh;

//  if(argc >= 2){
//    ROS_INFO("Input yaml file: %s", argv[1]);
//  } else{
//    ROS_ERROR("Please specify a yaml file with camera parameters");
//    ROS_ERROR("rosrun dji_sdk_demo demo_stereo_object_depth_perception m210_stereo_param.yaml");
//    return -1;
//  }

//  std::string yaml_file_path = argv[1];
//  std::string yaml_file_path = "/home/nvidia/catkin_ws/lena_stereo_param.yaml";
  std::string yaml_file_path = "/home/nvidia/catkin_ws/333333.yaml";
//  std::string yaml_file_path = "/home/nvidia/catkin_ws/lena_stereo_param_changjiao3.yaml";
  Config::setParamFile(yaml_file_path);

  //! Instantiate some relevant objects
  CameraParam::Ptr camera_left_ptr;
  CameraParam::Ptr camera_right_ptr;
  StereoFrame::Ptr stereo_frame_ptr;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> topic_synchronizer;

  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>* bbox_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_left_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_right_sub;
  message_filters::Synchronizer<topic_synchronizer>* sync;

  bbox_sub = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(nh, "/darknet_ros/bounding_boxes", 1);
  img_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/my_stereo/left/image_raw",1);
  img_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/my_stereo/right/image_raw",1);

  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(20), *img_left_sub, *img_right_sub, *bbox_sub);

  //img_left_sub->registerCallback(imageCallback);

  //! Setup stereo frame
  camera_left_ptr   = CameraParam::createCameraParam(CameraParam::FRONT_LEFT);
  camera_right_ptr  = CameraParam::createCameraParam(CameraParam::FRONT_RIGHT);
  stereo_frame_ptr = StereoFrame::createStereoFrame(camera_left_ptr, camera_right_ptr);


  //! Setup ros related stuff
  rect_img_left_publisher =
    nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/rectified_vga_front_left_image", 10);
  rect_img_right_publisher =
    nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/rectified_vga_front_right_image", 10);
  left_disparity_publisher =
    nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/disparity_front_left_image", 10);
  point_cloud_publisher =
    nh.advertise<sensor_msgs::PointCloud2>("/stereo_depth_perception/unprojected_pt_cloud", 10);
  object_info_pub =
    nh.advertise<visualization_msgs::MarkerArray>("/stereo_depth_perception/object_info", 1);

  //! For signal handling, e.g. if user terminate the program with Ctrl+C
  //! this program will unsubscribe the image stream if it's subscribed


  //! Display interactive prompt

  sync->registerCallback(boost::bind(&displayObjectPtCloudCallback, _1, _2, _3, stereo_frame_ptr));

  ros::spin();
}

void displayObjectPtCloudCallback(const sensor_msgs::ImageConstPtr& img_left, const sensor_msgs::ImageConstPtr& img_right, const darknet_ros_msgs::BoundingBoxesConstPtr& b_box, M210_STEREO::StereoFrame::Ptr& stereo_frame_ptr)
{
  std::cout
      << "yuh2"
      <<std::endl;
  //! Read raw images
  stereo_frame_ptr->readStereoImgs(img_left, img_right);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  //! Filter disparity map
  timer filter_start= std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->filterDisparityMap();
  is_disp_filterd = true;
  timer filter_end  = std::chrono::high_resolution_clock::now();

  //! Unproject image to 3D point cloud
  timer pt_cloud_start= std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->unprojectROSPtCloud();
  timer pt_cloud_end  = std::chrono::high_resolution_clock::now();

  //! Calculate object depth info
  stereo_frame_ptr->calcObjectInfo(b_box, marker_array);

  visualizeRectImgHelper(stereo_frame_ptr);

  visualizeDisparityMapHelper(stereo_frame_ptr);

  cv::waitKey(1);

  sensor_msgs::Image rect_left_img  = *img_left;
  sensor_msgs::Image rect_right_img = *img_right;
  sensor_msgs::Image disparity_map  = *img_left;
  memcpy((char*)(&rect_left_img.data[0]),
         stereo_frame_ptr->getRectLeftImg().data,
         img_left->height*img_left->width);
  memcpy((char*)(&rect_right_img.data[0]),
         stereo_frame_ptr->getRectRightImg().data,
         img_right->height*img_right->width);

  memcpy((char*)(&disparity_map.data[0]),
         stereo_frame_ptr->getFilteredDispMap().data,
         img_left->height*img_left->width);

  rect_img_left_publisher.publish(rect_left_img);
  rect_img_right_publisher.publish(rect_right_img);
  left_disparity_publisher.publish(disparity_map);
  point_cloud_publisher.publish(stereo_frame_ptr->getROSPtCloud());
  object_info_pub.publish(marker_array);

  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  duration filter_diff = filter_end - filter_start;
  duration pt_cloud_diff = pt_cloud_end - pt_cloud_start;
  ROS_INFO("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity, "
             "%.2f ms to filter, %.2f ms to unproject point cloud",
           rectify_time_diff.count()*1000.0,
           disp_time_diff.count()*1000.0,
           filter_diff.count()*1000.0,
           pt_cloud_diff.count()*1000.0);
}

void
visualizeRectImgHelper(StereoFrame::Ptr stereo_frame_ptr)
{
  cv::Mat img_to_show;

  cv::hconcat(stereo_frame_ptr->getRectLeftImg(),
              stereo_frame_ptr->getRectRightImg(),
              img_to_show);

  cv::resize(img_to_show, img_to_show,
             cv::Size(M210_STEREO::VGA_WIDTH*2, M210_STEREO::VGA_HEIGHT),
             (0, 0), (0, 0), cv::INTER_LINEAR);

  // draw epipolar lines to visualize rectification
  for(int j = 0; j < img_to_show.rows; j += 24 ){
    line(img_to_show, cv::Point(0, j),
         cv::Point(img_to_show.cols, j),
         cv::Scalar(255, 0, 0, 255), 1, 8);
  }

  cv::imshow("Rectified Stereo Imgs with epipolar lines", img_to_show);
}

void
visualizeDisparityMapHelper(StereoFrame::Ptr stereo_frame_ptr)
{
  cv::Mat raw_disp_map;
  if(is_disp_filterd) {
    raw_disp_map = stereo_frame_ptr->getFilteredDispMap().clone();
  } else {
    raw_disp_map = stereo_frame_ptr->getDisparityMap().clone();
  }

  double min_val, max_val;
  cv::minMaxLoc(raw_disp_map, &min_val, &max_val, NULL, NULL);

  cv::Mat scaled_disp_map;
  raw_disp_map.convertTo(scaled_disp_map, CV_8U, 255/(max_val-min_val), -min_val/(max_val - min_val));

  cv::imshow("Scaled disparity map", scaled_disp_map);
}

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

