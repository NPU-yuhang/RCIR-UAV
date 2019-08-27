#ifndef AVOID_OBSTA_FROM_MONO_RADAR_H
#define AVOID_OBSTA_FROM_MONO_RADAR_H
#include "mymission/Mission.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cam/vision_msg.h>
#include <cam/vision_msgs.h>
#include <cam/point.h>
#include <cam/mono_radar_fusion.h>

#include <mymission/artifical_potencial_field.h>

#include "config.h"

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */

// global variables for subscribed topics

int vs_info_counter = 0;
std_msgs::Float32MultiArray vs_data_x;
std_msgs::Float32MultiArray vs_data_y;
std_msgs::Float32MultiArray vs_data_yaw;

geometry_msgs::Point current_local_pos;

Mission square_mission;
Control_law con_law_vision, con_law_yaw;

bool isgetdata = false;
bool start_get_path = false;
bool has_got_path = false;
std::ofstream location_out;
std::string ss;

//control task param
ros::Time time_s;
double time_now, visionok_time, land_time;
float control_height;
double start_yaw;
double vision_height;

param vision_task_param = param();
cam::mono_radar_fusion obstacle_info;
cam::vision_msgs vision_info;
std::vector< std::vector<cv::Point2f> > obstacle_nodes_s;
artifical_potencial_field Artifical;

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void mission_control_callback(const std_msgs::Float32MultiArray::ConstPtr &missioninfo, const std_msgs::Float32MultiArray::ConstPtr &attitudeinfo);
void getvisiontarget(std_msgs::Float32MultiArray Array[100][3]);
void frametrans(geometry_msgs::Quaternion atti, std_msgs::Float32MultiArray Array[3]);
void obsta_info_callback(const cam::mono_radar_fusionConstPtr& obsta_info);
void vision_info_callback(const cam::vision_msgsConstPtr &vi);
void vision_navigation();
float sum_vision(std_msgs::Float32MultiArray data);
void get_path();
bool is_get_data();
bool data_is_available();

#endif // AVOID_OBSTA_FROM_MONO_RADAR_H
