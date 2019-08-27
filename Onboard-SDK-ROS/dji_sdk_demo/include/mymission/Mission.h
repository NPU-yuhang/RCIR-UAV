#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

// DJI SDK includes
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <mymission/Control_law.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.

  const float deg2rad = C_PI/180.0;
  const float rad2deg = 180.0/C_PI;
  const float lim_vel = 1.5;
  int state;

  uint8_t flight_status = 255;
  uint8_t display_mode  = 255;

  int inbound_counter;
  int outbound_counter;
  int break_counter;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;

  int info_counter = 0;
  std_msgs::Float32MultiArray data_x;
  std_msgs::Float32MultiArray data_y;
  std_msgs::Float32MultiArray data_z;

  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;
  sensor_msgs::NavSatFix current_gps;
  geometry_msgs::Quaternion current_atti;

  ros::Publisher ctrlBrakePub;
  ros::Publisher ctrlPosYawPub;

  ros::ServiceClient set_local_pos_reference;
  ros::ServiceClient sdk_ctrl_authority_service;
  ros::ServiceClient drone_task_service;
  ros::ServiceClient query_version_service;
  Control_law con_law;
  bool finished;

  Mission();
  void setService(ros::NodeHandle service_nh);
  void pubControl(ros::NodeHandle control_nh);
  void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }

  void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                           sensor_msgs::NavSatFix& target,
                           sensor_msgs::NavSatFix& origin);

  geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
  double sumlong();
  double sumlat();
  bool takeoff_land(int task);
  bool obtain_control();
  bool is_M100();
  bool monitoredTakeoff();
  bool monitoredLand();
  bool M100monitoredTakeoff();
  bool set_local_position();
};
