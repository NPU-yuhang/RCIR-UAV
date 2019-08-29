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
  cv::Mat radar_data = cv::Mat::zeros(2,count,CV_32FC1);
  for(int i=0; i<count; i++)
  {
      radar_data.at<float>(0,i) = -(radar->objs[i].Lat);
      radar_data.at<float>(1,i) = radar->objs[i].Long;
  }

  cv::Mat point = radar2image.TransformWRadarPoint2ImagePoint(radar_data);
  cv::Mat p_camera = radar2image.TransformWRadar2Camera(radar_data);

  cv::Mat img_cam = cv_bridge::toCvShare(img, "bgr8")->image;
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

  std::string s1 = std::to_string(p_camera.at<float>(0, lidar_obsta_1)) + ", " +
      std::to_string(p_camera.at<float>(1, lidar_obsta_1)) + ", " +
      std::to_string(p_camera.at<float>(2, lidar_obsta_2));
  std::string s2 = std::to_string(p_camera.at<float>(0, lidar_obsta_2)) + ", " +
      std::to_string(p_camera.at<float>(1, lidar_obsta_2)) + ", " +
      std::to_string(p_camera.at<float>(2, lidar_obsta_2));
  cv::putText(img_cam,s1,cv::Point(400, 300),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,23,0),2,4);
  cv::putText(img_cam,s2,cv::Point(0, 150),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(255,23,0),2,4);

  cam::point obs_point1, obs_point2;
  cam::mono_radar_fusion obs_info;
  obs_point1.x = p_camera.at<float>(0, lidar_obsta_1);
  obs_point1.y = p_camera.at<float>(1, lidar_obsta_1);
  obs_point1.z = p_camera.at<float>(2, lidar_obsta_1);

  obs_point2.x = p_camera.at<float>(0, lidar_obsta_2);
  obs_point2.y = p_camera.at<float>(1, lidar_obsta_2);
  obs_point2.z = p_camera.at<float>(2, lidar_obsta_2);

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
  camParam.fu = param_camera.camera_fx;
  camParam.fv = param_camera.camera_fy;
  camParam.cu = param_camera.camera_cx;
  camParam.cv = param_camera.camera_cy;
  camParam.pitch = 90*(CV_PI*1.0/180.0);
  camParam.yaw = 0*(CV_PI*1.0/180.0);
  camParam.roll = 0*(CV_PI*1.0/180.0);
  radar2image.set_param(camParam);
}
