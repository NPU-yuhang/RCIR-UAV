#include "mymission/caitu.h"

bool vga_imgs_subscribed = false;
cv::VideoWriter writer;
std::string ss;
std::string s1 = "/media/nvidia/SD2/flyvideo/images/";
std::string s2 = ".jpg";
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_pos;
geometry_msgs::Vector3 velocity;
std::list<std::pair<double, cv::Mat>> images_buf;
std::list<std::string> sensor_buf;
std::pair<double, cv::Mat> current_img;
std::string current_sensor;

std::ofstream location_out;
std::string sensor;
double time_s;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caitu");
  ros::NodeHandle nh;

  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber posSub      = nh.subscribe("dji_sdk/local_position", 10, &pos_callback);
  ros::Subscriber velo        = nh.subscribe("dji_sdk/velocity", 10, &velo_callback);
  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh,"/camera_BGR",100);

  //writer.open("/media/nvidia/SD2/flyvideo/VideoTest_lena.avi", CV_FOURCC('F', 'L', 'V', '1'), 25, cv::Size(800, 600), true);
  if (!writer.isOpened())
  {
  std::cout << "fail write an avi file" << std::endl;
  }

  time_s = ros::Time::now().toSec();
  img_sub.registerCallback(&StereoImgCallback);

//  location_out.open("/media/nvidia/SD/flyvideo/location_out_lena.txt", std::ios::out | std::ios::app);
//  //cv::FileStorage fs("/media/nvidia/SD/flyvideo/test.yml", cv::FileStorage::WRITE);
//  while(ros::ok())
//  {
//    if(!images_buf.empty())
//    {
//      //std::cout<<"yuan image buf have: "<<images_buf.size()<<std::endl;
//      current_img = images_buf.front();
//      current_sensor = sensor_buf.front();
//      ss = s1 + std::to_string(current_img.first - time_s) + s2;
//      //ss = "1.png";
//      std::cout<<ss<<std::endl;
//      imwrite(ss, current_img.second);
//      //fs<<"resultMat"<<current_img.second;
//      //fs<<std::endl;
//      //writer.write(rect_left_img);
//      location_out << current_sensor;
//      images_buf.pop_front();
//      sensor_buf.pop_front();
//      std::cout<<"image buf have: "<<images_buf.size()<<std::endl;
//    }
//    ros::spinOnce();
//  }
//  location_out.close();
  //fs.release();
  ros::spin();
  ros::shutdown();
  return 0;
}

void StereoImgCallback(const sensor_msgs::ImageConstPtr &img_left)
{
  //std::cout<<"okokok"<<std::endl;
  double time = img_left->header.stamp.toSec();
  cv::Mat rect_img = cv_bridge::toCvShare(img_left, "bgr8")->image;
  std::pair<double, cv::Mat> image_msg = std::make_pair(time, rect_img);
  images_buf.push_back(image_msg);

  sensor = std::to_string(current_gps.latitude) + " " + std::to_string(current_gps.longitude) + " "
      + std::to_string(current_gps.altitude) + " " + std::to_string(velocity.x) + " "
      + std::to_string(velocity.y) + " " + std::to_string(velocity.z) + " "
      + std::to_string(toEulerAngle(current_atti).x) + " " + std::to_string(toEulerAngle(current_atti).y)
      + " " + std::to_string(toEulerAngle(current_atti).z) + " " + std::to_string((time-time_s))
      + "\n";
  sensor_buf.push_back(sensor);

  std::cout<<"yuan image buf have: "<<images_buf.size()<<std::endl;
  current_img = images_buf.front();
  current_sensor = sensor_buf.front();
  ss = s1 + std::to_string(current_img.first - time_s) + s2;
  //ss = "1.png";
  std::cout<<ss<<std::endl;
  imwrite(ss, current_img.second);
  //writer.write(current_img.second);
  location_out.open("/media/nvidia/SD2/flyvideo/location_out_lena.txt", std::ios::out | std::ios::app);
  location_out << current_sensor;
  location_out.close();
  images_buf.pop_front();
  sensor_buf.pop_front();
  std::cout<<"image buf have: "<<images_buf.size()<<std::endl;
  //std::cout<<rect_img.size<<std::endl;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &gps)
{
  current_gps = *gps;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &atti)
{
  current_atti = atti->quaternion;
}

void pos_callback(const geometry_msgs::PointStamped::ConstPtr &pos)
{
  current_pos = pos->point;
}

void velo_callback(const geometry_msgs::Vector3Stamped::ConstPtr &velo)
{
  velocity = velo->vector;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

