#include <cam/mono_lidar_fusion.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caitu");
  ros::NodeHandle nh;

  if(argc != 2)
  {
      std::cerr << std::endl << "no config file" << std::endl;
      ros::shutdown();
      return 1;
  }
  fusion_param.readparam(argv[1]);
  setCameraParam(fusion_param);
  obstacle_info_pub = nh.advertise<cam::mono_radar_fusion>("/mono_radar/obstacle/msg", 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, my_radar::objects, cam::vision_msgs> topic_synchronizer;
  message_filters::Subscriber<sensor_msgs::Image>* img_sub;
  message_filters::Subscriber<my_radar::objects>* radar_sub;
  message_filters::Subscriber<cam::vision_msgs>* obstacke_sub;

  message_filters::Synchronizer<topic_synchronizer>* sync;
  img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/obstacle/img", 100);
  //scan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "/scan", 100);
  radar_sub = new message_filters::Subscriber<my_radar::objects>(nh, fusion_param.radar_mod, 100);
  obstacke_sub = new message_filters::Subscriber<cam::vision_msgs>(nh, "/obstacle/msg", 100);

  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(10), *img_sub, *radar_sub, *obstacke_sub);
  sync->registerCallback(boost::bind(&FusionCallback, _1, _2, _3));

  ros::spin();
  ros::shutdown();
  return 0;
}

void FusionCallback(const sensor_msgs::ImageConstPtr &img, const my_radar::objectsConstPtr &radar, const cam::vision_msgsConstPtr &obstacle)
{
  //std::cout<<"okokok"<<std::endl;
  cv::Point obsta_1, obsta_2;
  obsta_1.x = obstacle->obsta_info[0].x;
  obsta_1.y = obstacle->obsta_info[0].y;
  obsta_2.x = obstacle->obsta_info[1].x;
  obsta_2.y = obstacle->obsta_info[1].y;

  int count = radar->num;
  cv::Mat src(count,3,CV_32FC1);
  for(int i = 0; i<count; i++)
  {
    src.at<float>(i, 0) = -(radar->objs[i].Lat);
    src.at<float>(i, 1) = h;
    src.at<float>(i, 2) = radar->objs[i].Long;
  }
  //int count = scan->scan_time / scan->time_increment;
//  for(int i = 0, j = 0; i < count; i++) {
//      float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
//      if(degree > -32 && degree <32)
//      {
//        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
//        float x = scan->ranges[i] * sin(DEG2RAD(degree));
//        float y = scan->ranges[i] * cos(DEG2RAD(degree));
//        src.at<float>(j, 0) = x;
//        src.at<float>(j, 1) = h;
//        src.at<float>(j, 2) = y;
//        j++;
//      }
//  }

  cv::Mat img_cam = cv_bridge::toCvShare(img, "bgr8")->image;
//  cv::Mat undistort_img;
//  cv::undistort(img_cam, undistort_img, intrinsic, distortion_coeff);//矫正相机镜头变形
//  cv::imshow("lalala", undistort_img);
//  cv::imshow("lblblb", img_cam);
  cv::Mat point = intrinsic * src.t();
  cv::Mat scale(3, count, CV_32FC1);
  point.row(2).copyTo(scale.row(0));
  point.row(2).copyTo(scale.row(1));
  point.row(2).copyTo(scale.row(2));
  //cv::normalize(point,point,1.0,0.0,cv::NORM_INF);
  scale = 1/scale;
  point = point.mul(scale);
  std::vector<cv::Point> lidar_pts;
  //lidar_pts.resize(65);

  int lidar_obsta_1 = 0;
  int lidar_obsta_2 = 0;
  int min_dis_1 = 800;
  int min_dis_2 = 800;

  for(int i = 0; i<count; i++)
  {
    cv::Point lidar_pt;
    //std::cout<<point.at<float>(0, i)<<"  "<<point.at<float>(1, i)<<"  "<<std::endl;
    if(point.at<float>(0, i) <= 800 && point.at<float>(0, i) >=1 && point.at<float>(1, i) <= 600 && point.at<float>(1, i) >=1)
    {
      lidar_pt.x = (int)point.at<float>(0, i);
      lidar_pt.y = (int)point.at<float>(1, i);
      cv::circle(img_cam, lidar_pt, 4, cv::Scalar(0, 0, 255));
      if(abs(lidar_pt.x - obsta_1.x) < min_dis_1)
      {
        lidar_obsta_1 = i;
        min_dis_1 = abs(lidar_pt.x - obsta_1.x);
      }
      if(abs(lidar_pt.x - obsta_2.x) < min_dis_2)
      {
        lidar_obsta_2 = i;
        min_dis_2 = abs(lidar_pt.x - obsta_2.x);
      }
    }
    lidar_pts.push_back(lidar_pt);
  }

  std::string s1 = std::to_string(src.at<float>(lidar_obsta_1, 0)) + ", " +
      std::to_string(src.at<float>(lidar_obsta_1, 1)) + ", " +
      std::to_string(src.at<float>(lidar_obsta_1, 2));
  std::string s2 = std::to_string(src.at<float>(lidar_obsta_2, 0)) + ", " +
      std::to_string(src.at<float>(lidar_obsta_2, 1)) + ", " +
      std::to_string(src.at<float>(lidar_obsta_2, 2));
  cv::putText(img_cam,s1,cv::Point(400, 300),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,23,0),2,4);
  cv::putText(img_cam,s2,cv::Point(0, 150),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(255,23,0),2,4);

  cam::point obs_point1, obs_point2;
  cam::mono_radar_fusion obs_info;
  obs_point1.x = src.at<float>(lidar_obsta_1, 0);
  obs_point1.y = src.at<float>(lidar_obsta_1, 1);
  obs_point1.z = src.at<float>(lidar_obsta_1, 2);

  obs_point2.x = src.at<float>(lidar_obsta_2, 0);
  obs_point2.y = src.at<float>(lidar_obsta_2, 1);
  obs_point2.z = src.at<float>(lidar_obsta_2, 2);

  obs_info.header.stamp = ros::Time::now();
  obs_info.points.push_back(obs_point1);
  obs_info.points.push_back(obs_point2);
  obstacle_info_pub.publish(obs_info);
  //std::cout<<lidar_obsta_1<<"---"<<min_dis_1<<"---"<<lidar_obsta_2<<"---"<<min_dis_2<<std::endl;
  cv::imshow("lalala", img_cam);
  //std::cout<<point<<std::endl;
  cv::waitKey(1);
}

void setCameraParam(param param_camera)
{
  intrinsic.create(3, 3, CV_32FC1);//相机内参数
  distortion_coeff.create(5, 1, CV_32FC1);//畸变参数

  /*
  fx 0 cx
  0 fy cy
  0 0  1     内参数
  */
  intrinsic.at<float>(0, 0) = param_camera.camera_fx;//fx
  intrinsic.at<float>(0, 2) = param_camera.camera_cx;//cx
  intrinsic.at<float>(1, 1) = param_camera.camera_fy;//fy
  intrinsic.at<float>(1, 2) = param_camera.camera_cy;//cy

  intrinsic.at<float>(0, 1) = 0;
  intrinsic.at<float>(1, 0) = 0;
  intrinsic.at<float>(2, 0) = 0;
  intrinsic.at<float>(2, 1) = 0;
  intrinsic.at<float>(2, 2) = 1;

  /*
  k1 k2 p1 p2 p3    畸变参数
  */
  distortion_coeff.at<float>(0, 0) = param_camera.camera_k1;//k1
  distortion_coeff.at<float>(1, 0) = param_camera.camera_k2;//k2
  distortion_coeff.at<float>(2, 0) = param_camera.camera_p1;//p1
  distortion_coeff.at<float>(3, 0) = param_camera.camera_p2;//p2
  distortion_coeff.at<float>(4, 0) = param_camera.camera_p3;//p3
}
