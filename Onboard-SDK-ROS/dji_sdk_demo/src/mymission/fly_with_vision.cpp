/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "mymission/fly_with_vision.h"
#include "dji_sdk/dji_sdk.h"

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
const float lim_vel = 5.0;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint16_t count = 0;

int info_counter = 0;

float object_x;
float object_y;
float object_z;

std_msgs::Float32MultiArray array[100][3];
std_msgs::Float32MultiArray posit[3];
std_msgs::Float32MultiArray gps_posit[3];
std_msgs::Float32MultiArray data_x;
std_msgs::Float32MultiArray data_y;
std_msgs::Float32MultiArray data_z;

sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

Mission square_mission;
Control_data control_data;
Control_law con_law;

bool isgetdata = false;
std::ofstream location_out;
std::string ss;

ros::Time time_s;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber object_info = nh.subscribe("/stereo_depth_perception/object_info", 10, &object_info_callback);
  ros::Subscriber mission_track = nh.subscribe("/mission_track", 10, &mission_track_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  con_law.set_control_law(2.0, 0.5, 0.0, 0.95, 1.1);


  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

  if(takeoff_result)
  {
    time_s = ros::Time::now();
    square_mission.reset();
    square_mission.start_gps_location = current_gps;
    square_mission.start_local_position = current_local_pos;
    square_mission.setTarget(control_data.mission_x, control_data.mission_y, control_data.mission_h, 0);
    square_mission.state = 1;
  }

  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void Mission::step()
{
  geometry_msgs::Vector3     localOffset;

//  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  square_mission.setTarget(control_data.mission_x, control_data.mission_y, control_data.mission_h, 90);

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  if(info_counter < 3)
  {
    data_x.data.push_back(xOffsetRemaining);
    data_y.data.push_back(yOffsetRemaining);
  }
  else{
    for(int i = 1; i >= 0; i--)
    {
      data_x.data[i+1] = data_x.data[i];
      data_y.data[i+1] = data_y.data[i];
    }
    data_x.data[0] = xOffsetRemaining;
    data_y.data[0] = yOffsetRemaining;

    data_x.data.resize(3);
    data_y.data.resize(3);
  }

  std::cout<<xOffsetRemaining<<std::endl;
  std::cout<<localOffset.x<<std::endl;
  std::cout<<target_offset_x<<std::endl;

  info_counter++;

  xCmd = con_law.Kp * xOffsetRemaining + con_law.Ki * sumlat() +  con_law.Kd * (xOffsetRemaining - data_x.data[data_x.data.size() - 2]) + con_law.Kv * control_data.mission_dx + con_law.Ka * control_data.mission_ddx;
  yCmd = con_law.Kp * yOffsetRemaining + con_law.Ki * sumlong() + con_law.Kd * (yOffsetRemaining - data_y.data[data_y.data.size() - 2]) + con_law.Kv * control_data.mission_dy + con_law.Ka * control_data.mission_ddy;
  zCmd = target_offset_z;

  if(xCmd >= lim_vel)
  {
    xCmd = lim_vel;
  }
  if(xCmd <= -lim_vel)
  {
    xCmd = -lim_vel;
  }

  if(yCmd >= lim_vel)
  {
    yCmd = lim_vel;
  }
  if(yCmd <= -lim_vel)
  {
    yCmd = -lim_vel;
  }


  std::cout<<xCmd<<" "<<yCmd<<" "<<zCmd<<std::endl;
  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */
  sensor_msgs::Joy controlPosYaw;
  uint8_t flag1 = (DJISDK::VERTICAL_POSITION   |
                  DJISDK::HORIZONTAL_VELOCITY |
                  DJISDK::YAW_ANGLE            |
                  DJISDK::HORIZONTAL_GROUND   |
                  DJISDK::STABLE_ENABLE);

  controlPosYaw.axes.push_back(xCmd);
  controlPosYaw.axes.push_back(yCmd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(yawDesiredRad);
  controlPosYaw.axes.push_back(flag1);
  ctrlPosYawPub.publish(controlPosYaw);
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  double current_yaw = toEulerAngle(current_atti).z * rad2deg;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    switch (square_mission.state) {
    case 0:
      break;
    case 1:
      if(!square_mission.finished && is_get_data())
      {
        square_mission.setTarget(control_data.mission_x, control_data.mission_y, control_data.mission_h, current_yaw);
        square_mission.state = 1;
        square_mission.step();

        location_out.open("location_out.txt", std::ios::out | std::ios::app);
        ss = std::to_string(current_local_pos.x) + " " + std::to_string(current_local_pos.y) + " " + std::to_string(current_local_pos.z) + " " + std::to_string(toEulerAngle(current_atti).x) + " " + std::to_string(toEulerAngle(current_atti).y) + " " + std::to_string(toEulerAngle(current_atti).z) + " " +std::to_string((start_time-time_s).toSec()) + "\n";
        location_out << ss << std::endl;
        location_out.close();
      }
      else{
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, current_local_pos.z, current_yaw);
      }
      break;
    }
  }

}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

void object_info_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  count ++ ;
  if(count<=100)
  {
    int object_num = msg->markers.size();
    ROS_INFO("Got %d detections", object_num);
    visualization_msgs::Marker maker;
    maker.text = msg->markers[0].text;
    std::string object_info = msg->markers[0].text;
    object_x = msg->markers[0].pose.position.x + 1;
    object_y = msg->markers[0].pose.position.y;
    object_z = msg->markers[0].pose.position.z;
    //ROS_INFO("airplane has detected: %s", object_info);
    array[count-1][0].data.push_back(object_x);
    array[count-1][1].data.push_back(object_y);
    array[count-1][2].data.push_back(object_z);
    std::cout<<object_x<<" "<<object_y<<" "<<object_z<<std::endl;
  }
  else
  {
    getvisiontarget(array);
  }
}

void mission_track_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  control_data.mission_x   = msg->data[0];
  control_data.mission_y   = msg->data[1];
  control_data.mission_h   = msg->data[2];
  control_data.mission_dx  = msg->data[3];
  control_data.mission_dy  = msg->data[4];
  control_data.mission_ddx = msg->data[5];
  control_data.mission_ddy = msg->data[6];
}

void getvisiontarget(std_msgs::Float32MultiArray Array[100][3])
{
  float object_xsum = 0;
  float object_ysum = 0;
  float object_zsum = 0;
  for(int i = 0; i < 100; i++)
  {
    object_xsum = object_xsum + Array[i][0].data.back();
    object_ysum = object_ysum + Array[i][1].data.back();
    object_zsum = object_zsum + Array[i][2].data.back();
  }
  object_x = object_xsum/100;
  object_y = object_ysum/100;
  object_z = object_zsum/100;
  posit[0].data.push_back(object_x);
  posit[1].data.push_back(object_y);
  posit[2].data.push_back(object_z);

  frametrans(current_atti, posit);
}

void frametrans(geometry_msgs::Quaternion atti, std_msgs::Float32MultiArray Array[3])
{
  double pitch = toEulerAngle(atti).x;
  double roll  = toEulerAngle(atti).y;
  double yaw   = toEulerAngle(atti).z;

  float position_x = Array[0].data.back();
  float position_y = Array[1].data.back();
  float position_z = Array[2].data.back();

  std_msgs::Float32MultiArray trans[3][3];

  trans[0][0].data.push_back(std::cos(pitch)*std::cos(yaw));
  trans[0][1].data.push_back(std::sin(yaw));
  trans[0][2].data.push_back(- std::sin(pitch)*std::cos(yaw));
  trans[1][0].data.push_back(- std::cos(pitch)*std::sin(yaw)*std::cos(roll) + std::cos(pitch)*std::sin(roll));
  trans[1][1].data.push_back(std::cos(yaw)*std::cos(roll));
  trans[1][2].data.push_back(std::sin(pitch)*std::sin(yaw)*std::cos(roll) + std::cos(pitch)*std::sin(roll));
  trans[2][0].data.push_back(std::cos(pitch)*std::sin(yaw)*std::sin(roll) + std::sin(pitch)*std::cos(roll));
  trans[2][1].data.push_back(-std::cos(yaw)*std::sin(roll));
  trans[2][2].data.push_back(-std::sin(pitch)*std::sin(yaw)*std::sin(roll) + std::cos(pitch)*std::cos(roll));

  float GPS_position_x = trans[0][0].data.back() * position_x + trans[0][1].data.back() * position_y + trans[0][2].data.back() * position_z;
  float GPS_position_y = trans[1][0].data.back() * position_x + trans[1][1].data.back() * position_y + trans[1][2].data.back() * position_z;
  float GPS_position_z = trans[2][0].data.back() * position_x + trans[2][1].data.back() * position_y + trans[2][2].data.back() * position_z;

  gps_posit[0].data.push_back(GPS_position_x);
  gps_posit[1].data.push_back(GPS_position_y);
  gps_posit[2].data.push_back(GPS_position_z);

  std::cout<<pitch*180/C_PI<<" "<<roll*180/C_PI<<" "<<yaw*180/C_PI<<" "<<std::endl;
  std::cout<<GPS_position_x<<" "<<GPS_position_y<<" "<<GPS_position_z<<std::endl;
}

double sumlat()
{
  double ans;
  for (int i = 0; i < data_x.data.size(); i++)
  {
    ans = ans + data_x.data[i];
  }
}

double sumlong()
{
  double ans;
  for (int i = 0; i < data_y.data.size(); i++)
  {
    ans = ans + data_y.data[i];
  }
}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

bool is_get_data()
{
  if(control_data.mission_h != 0)
  {
    return true;
  }
  return false;
}
