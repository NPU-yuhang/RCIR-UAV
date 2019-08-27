#include "mymission/caitu_new_cam.h"

bool vga_imgs_subscribed = false;
cv::VideoWriter writer;
std::string ss;
std::string ss1;
std::string ss2;
std::string s1 = "/media/nvidia/SD1/flyvideo/images/";
std::string s2 = ".png";
std::string s3 = "left/";
std::string s4 = "right/";
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_pos;
geometry_msgs::Vector3 velocity;

std::ofstream location_out;
std::string sensor;

ros::Time time_s;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caitu");
  ros::NodeHandle nh;

  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber posSub      = nh.subscribe("dji_sdk/local_position", 10, &pos_callback);
  ros::Subscriber velo        = nh.subscribe("dji_sdk/velocity", 10, &velo_callback);
  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> topic_synchronizer;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> topic_synchronizer;

  message_filters::Subscriber<sensor_msgs::Image>* img_left_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_right_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_front_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_back_sub;
  message_filters::Synchronizer<topic_synchronizer>* sync;

  img_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/zed/left/image_raw_color",1);
  img_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/zed/right/image_raw_color",1);

  //sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(10), *img_left_sub, *img_right_sub, *img_front_sub, *img_back_sub);
  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(10), *img_left_sub, *img_right_sub);

  writer.open("/media/nvidia/SD1/flyvideo/VideoTest_lena.avi", CV_FOURCC('F', 'L', 'V', '1'), 25, cv::Size(672*2, 376), true);
  if (!writer.isOpened())
  {
  std::cout << "fail write an avi file" << std::endl;
  }

  time_s = ros::Time::now();
  sync->registerCallback(boost::bind(&StereoImgCallback, _1, _2));
  ros::spin();
  writer.release();
}

void StereoImgCallback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right)
{
  std::cout<<"okokok"<<std::endl;
  ros::Time time = ros::Time::now();
  cv::Mat rect_left_img = cv_bridge::toCvShare(img_left, "bgr8")->image;
  cv::Mat rect_right_img = cv_bridge::toCvShare(img_right, "bgr8")->image;

  cv::Mat img_toshow;
  cv::hconcat(rect_left_img, rect_right_img, img_toshow);
  cv::resize(img_toshow, img_toshow,
             cv::Size(672*2, 376),
             (0, 0), (0, 0), cv::INTER_LINEAR);

  //cv::imshow("stereo", img_toshow);
  ss = s1 + std::to_string((time-time_s).toSec()) + s2;
  ss1 = s1 + s3 + std::to_string((time-time_s).toSec()) + s2;
  ss2 = s1 + s4 + std::to_string((time-time_s).toSec()) + s2;
  //ss = "1.png";
  std::cout<<ss<<std::endl;
  //imwrite(ss1, rect_left_img);
  //imwrite(ss2, rect_right_img);
  if(writer.isOpened())
  {
    std::cout<<"video is open"<<std::endl;
  }
  writer.write(img_toshow);

  location_out.open("/media/nvidia/SD1/flyvideo/location_out_lena.txt", std::ios::out | std::ios::app);
  sensor = std::to_string(current_gps.latitude) + " " + std::to_string(current_gps.longitude) + " " + std::to_string(current_gps.altitude) + " " + std::to_string(velocity.x) + " " + std::to_string(velocity.y) + " " + std::to_string(velocity.z) + " " + std::to_string(toEulerAngle(current_atti).x) + " " + std::to_string(toEulerAngle(current_atti).y) + " " + std::to_string(toEulerAngle(current_atti).z) + " " + std::to_string((time-time_s).toSec()) + "\n";
  location_out << sensor;
  location_out.close();

  std::cout<<img_toshow.size<<std::endl;

  cv::waitKey(1);
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
