#include "mymission/kalibr_bag.h"

std::string ss1 = "/media/nvidia/SD1/catkin_m100/src/Onboard-SDK-ROS/dji_sdk_demo/output/cam0/";
std::string ss2 = "/media/nvidia/SD1/catkin_m100/src/Onboard-SDK-ROS/dji_sdk_demo/output/cam1/";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance_task");
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> topic_synchronizer;
  ros::Subscriber imu_sub = nh.subscribe("/imu_data", 10, &imu_Callback);
  message_filters::Subscriber<sensor_msgs::Image>* img_left_sub;
  message_filters::Subscriber<sensor_msgs::Image>* img_right_sub;
  message_filters::Synchronizer<topic_synchronizer>* sync;

  img_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera_left_BGR", 1);
  img_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera_right_BGR", 1);
  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(10), *img_left_sub, *img_right_sub);

  sync->registerCallback(boost::bind(&img_Callback, _1, _2));

  ros::spin();
  ros::shutdown();
  return 0;
}

void imu_Callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
  ofstream foutC("/media/nvidia/SD1/catkin_m100/src/Onboard-SDK-ROS/dji_sdk_demo/output/imu0.csv", ios::app);
  foutC.setf(ios::fixed, ios::floatfield);
  foutC.precision(0);
  foutC << ros::Time::now().toSec() * 1e9 << ",";
  foutC.precision(5);
  foutC << imu_msg->angular_velocity.x << ","
        << imu_msg->angular_velocity.y << ","
        << imu_msg->angular_velocity.z << ","
        << imu_msg->linear_acceleration.x << ","
        << imu_msg->linear_acceleration.y << ","
        << imu_msg->linear_acceleration.z << endl;
  foutC.close();
}

void img_Callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right)
{
  cv::Mat left_img = cv_bridge::toCvShare(left, "bgr8")->image;
  cv::Mat right_img = cv_bridge::toCvShare(right, "bgr8")->image;
  ros::Time T_img = ros::Time::now();
  string s1 = ss1+to_string(T_img.toSec() * 1e9);
  s1 = s1.substr(0,s1.size()-7) + ".png";
  string s2 = ss2+to_string(T_img.toSec() * 1e9);
  s2 = s2.substr(0,s2.size()-7) + ".png";
  imwrite(s1, left_img);
  imwrite(s2, right_img);
}
