#include "mymission/auto_landing_task.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  if(argc != 2)
  {
      std::cerr << std::endl << "no config file" << std::endl;
      ros::shutdown();
      return 1;
  }

  vision_task_param.readparam(argv[1]);
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  message_filters::Subscriber<cam::vision_msg> land_infosub(nh, "vision2/msg", 10);
  land_infosub.registerCallback(land_info_callback);

  square_mission.pubControl(nh);
  square_mission.setService(nh);
  bool obtain_control_result = square_mission.obtain_control();
  bool takeoff_result;

  //con_law.set_control_law(2.0, 0.5, 0.0, 0.95, 1.1);
  //con_law.set_control_law(1, 0.2, 5, 0.95, 1.1);
  square_mission.con_law.set_control_law(vision_task_param.GPS_nav_Kp, vision_task_param.GPS_nav_Ki, vision_task_param.GPS_nav_Kd, vision_task_param.GPS_nav_Ka, vision_task_param.GPS_nav_Kv);
  con_law_vision.set_control_law(vision_task_param.vision_nav_Kp, vision_task_param.vision_nav_Ki, vision_task_param.vision_nav_Kd, vision_task_param.vision_nav_Ka, vision_task_param.vision_nav_Kv);
  con_law_yaw.set_control_law(vision_task_param.yaw_nav_Kp, vision_task_param.yaw_nav_Ki, vision_task_param.yaw_nav_Kd, vision_task_param.yaw_nav_Ka, vision_task_param.yaw_nav_Kv);

  if (!square_mission.set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }
  square_mission.reset();
  if(square_mission.is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = square_mission.M100monitoredTakeoff();
  }
//    ROS_INFO("A3/N3 taking off!");
//    takeoff_result = monitoredTakeoff();

  if(takeoff_result)
  {
    time_s = ros::Time::now();
    square_mission.reset();
    square_mission.start_gps_location = square_mission.current_gps;
    square_mission.start_local_position = current_local_pos;
    start_yaw = square_mission.toEulerAngle(square_mission.current_atti).z * square_mission.rad2deg;
    square_mission.setTarget(0, 0, current_local_pos.z, start_yaw);
    square_mission.state = 1;
    ROS_INFO("##### Start route %d ....", square_mission.state);
  }

  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

void vision_navigation()
{
  float xcmd, ycmd, zcmd, yawcmd;
  float xdesired = Land_Info.x;
  float ydesired = Land_Info.y;
  float yawdesired = Land_Info.theta;
  double yawDesiredRad     = square_mission.deg2rad * start_yaw;

  if(vs_info_counter < 5)
  {
    vs_data_x.data.push_back(xdesired);
    vs_data_y.data.push_back(ydesired);
    vs_data_yaw.data.push_back(yawdesired);
  }
  else {
    for(int i = 3; i >= 0; i--)
    {
      vs_data_x.data[i+1] = vs_data_x.data[i];
      vs_data_y.data[i+1] = vs_data_y.data[i];
      vs_data_yaw.data[i+1] = vs_data_yaw.data[i];
    }
    vs_data_x.data[0] = xdesired;
    vs_data_y.data[0] = ydesired;
    vs_data_yaw.data[0] = yawdesired;

    vs_data_x.data.resize(5);
    vs_data_y.data.resize(5);
    vs_data_yaw.data.resize(5);
  }
  vs_info_counter++;
  for(int i = 0; i<vs_data_x.data.size(); i++)
  {
    std::cout<<"data: "<<vs_data_x.data[i];
  }

  std::cout<<"xdesired: "<<xdesired<<"ydesired: "<<ydesired<<std::endl;

  xcmd = con_law_vision.Kp * xdesired + con_law_vision.Ki * sum_vision(vs_data_x)
      + con_law_vision.Kd * (xdesired - vs_data_x.data[vs_data_x.data.size() - 2]);
  std::cout<<"I xiang: "<<sum_vision(vs_data_x)<<std::endl;

  ycmd = con_law_vision.Kp * ydesired + con_law_vision.Ki * sum_vision(vs_data_y)
      + con_law_vision.Kd * (ydesired - vs_data_y.data[vs_data_y.data.size() - 2]);

  yawcmd = con_law_yaw.Kp * yawdesired + con_law_yaw.Ki * sum_vision(vs_data_yaw)
      + con_law_yaw.Kd * (yawdesired - vs_data_yaw.data[vs_data_yaw.data.size() - 2]);
  ycmd = -ycmd;
  zcmd = square_mission.target_offset_z;

  std::cout<<"xcmd: "<<xcmd<<" ycmd: "<<ycmd<<" yawcmd: "<<yawcmd<<" zcmd: "<<zcmd<<std::endl;

  if(xcmd >= square_mission.lim_vel)
    xcmd = square_mission.lim_vel;
  if(xcmd <= -square_mission.lim_vel)
    xcmd = -square_mission.lim_vel;

  if(ycmd >= square_mission.lim_vel)
    ycmd = square_mission.lim_vel;
  if(ycmd <= -square_mission.lim_vel)
    ycmd = -square_mission.lim_vel;

  if(yawcmd >= 10)
    yawcmd = 10;
  if(yawcmd <= -10)
    yawcmd = -10;

  std::cout<<"vx: "<<xcmd<<" vy: "<<ycmd<<" vyaw: "<<yawcmd<<std::endl;

  sensor_msgs::Joy controlVelYawRate;
  uint8_t flag1 = (DJISDK::VERTICAL_POSITION  |
                  DJISDK::HORIZONTAL_VELOCITY |
                  DJISDK::YAW_ANGLE           |
                  DJISDK::HORIZONTAL_BODY     |
                  DJISDK::STABLE_ENABLE);

  controlVelYawRate.axes.push_back(xcmd);
  controlVelYawRate.axes.push_back(ycmd);
  controlVelYawRate.axes.push_back(zcmd);
  controlVelYawRate.axes.push_back(yawDesiredRad);
  controlVelYawRate.axes.push_back(flag1);
  square_mission.ctrlBrakePub.publish(controlVelYawRate);
}

bool correct_ok()
{
  bool x_ok = true;
  bool y_ok = true;
  bool yaw_ok = true;
  for(int i = 0; i < vs_data_x.data.size(); i++)
  {
    if(vs_data_x.data[i] > 3 || vs_data_x.data[i] < -3)
    {
      x_ok = false;
      break;
    }
  }
  for(int i = 0; i < vs_data_y.data.size(); i++)
  {
    if(vs_data_y.data[i] > 3 || vs_data_y.data[i] < -3)
    {
      y_ok = false;
      break;
    }
  }
  for(int i = 0; i < vs_data_yaw.data.size(); i++)
  {
    if(vs_data_yaw.data[i] > 3 || vs_data_yaw.data[i] < -3)
    {
      yaw_ok = false;
      break;
    }
  }

  if(x_ok && y_ok && yaw_ok)
    return true;
  else
    return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  square_mission.current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;

  double current_yaw = square_mission.toEulerAngle(square_mission.current_atti).z * square_mission.rad2deg;
  square_mission.current_gps = *msg;
  //std::cout<<"current_yaw: "<<current_yaw<<std::endl;

  if(elapsed_time > ros::Duration(0.02))
  {
    if (square_mission.state == 0) {
    }

    if (square_mission.state == 1)
    {
      std::cout<<"start mode 1"<<std::endl;
      time_now = ros::Time::now().toSec() - time_s.toSec();
      if(time_now <= 5)
      {
        std::cout<<"shang shen jie duan"<<std::endl;
        control_height = 1.2 + (4-1.2)*(time_now/5);
        square_mission.setTarget(0, 0, control_height, start_yaw);
        square_mission.state = 1;
        square_mission.step();
      }
      if(time_now>5 && time_now<20)
      {
        std::cout<<"xuan ting jie duan"<<std::endl;
        square_mission.setTarget(5, 0, 4, start_yaw);
        square_mission.state = 1;
        square_mission.step();
      }
      if(time_now>20 && time_now<40 && !is_get_data())
      {
        std::cout<<"xuan ting wu shu ju jie duan"<<std::endl;
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, current_local_pos.z, start_yaw);
        square_mission.state = 1;
        square_mission.step();
      }
      if(time_now>20 && time_now<40 && is_get_data())
      {
        std::cout<<"dui zhun jie duan"<<std::endl;
          vision_navigation();
          if(correct_ok())
            square_mission.state = 2;
      }
      if(time_now>40)
      {
        std::cout<<"tiao zhuan jie duan"<<std::endl;
        square_mission.state = 2;
      }
    }

    if (square_mission.state == 2)
    {
      std::cout<<"start mode 2"<<std::endl;
      visionok_time = ros::Time::now().toSec() - time_s.toSec();
      std::cout<<"visionok_time: "<<visionok_time<<" time_now: "<<time_now<<std::endl;
      if(!is_get_data() && (visionok_time - time_now)<=15)
      {
        if(visionok_time - time_now <= 12)
          control_height = 4 - (4-0.8)*((visionok_time - time_now)/10);
        std::cout<<"height: "<<control_height<<std::endl;
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, control_height, start_yaw);
        square_mission.state = 2;
        square_mission.step();
      }
      if(is_get_data() && (visionok_time - time_now)<=15)
      {
        control_height = 4 - (4-0.8)*((visionok_time - time_now)/10);
        std::cout<<"height: "<<control_height<<std::endl;
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, control_height, start_yaw);
        square_mission.state = 2;
        vision_navigation();
      }
      if(is_get_data() && (visionok_time - time_now)>15)
      {
        con_law_vision.set_control_law(vision_task_param.vision_nav_Kp_low, vision_task_param.vision_nav_Ki_low, vision_task_param.vision_nav_Kd_low, vision_task_param.vision_nav_Ka_low, vision_task_param.vision_nav_Kv_low);

        //vision_navigation();
//        if(correct_ok())
//          square_mission.state = 3;
        if(visionok_time - time_now>15)
          square_mission.state = 3;
      }
      if(!is_get_data() && (visionok_time - time_now)>15)
      {
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, current_local_pos.z, start_yaw);
        square_mission.step();
        square_mission.state = 3;
      }
    }

    if (square_mission.state == 3)
    {
      std::cout<<"start mode 3"<<std::endl;
      land_time = ros::Time::now().toSec() - time_s.toSec();
      if(square_mission.monitoredLand())
        square_mission.state = 0;
    }
  }

  // Down sampled to 50Hz loop
//  if(elapsed_time > ros::Duration(0.02))
//  {
//    start_time = ros::Time::now();
//    switch (square_mission.state) {
//    case 0:
//      break;
//    case 1:
//      if(!square_mission.finished && is_get_data())
//      {
//        square_mission.setTarget(control_data.mission_x, control_data.mission_y, control_data.mission_h, current_yaw);
//        square_mission.state = 1;
//        square_mission.step();

//        location_out.open("location_out.txt", std::ios::out | std::ios::app);
//        ss = std::to_string(current_local_pos.x) + " " + std::to_string(current_local_pos.y) + " " + std::to_string(current_local_pos.z) + " " + std::to_string(toEulerAngle(current_atti).x) + " " + std::to_string(toEulerAngle(current_atti).y) + " " + std::to_string(toEulerAngle(current_atti).z) + " " +std::to_string((start_time-time_s).toSec()) + "\n";
//        location_out << ss << std::endl;
//        location_out.close();
//      }
//      else{

//        square_mission.setTarget(current_local_pos.x, current_local_pos.y, current_local_pos.z, current_yaw);
//      }
//      break;
//    }
//  }

}

void land_info_callback(const cam::vision_msgConstPtr& land_info)
{
  Land_Info.theta = land_info->theta;
  Land_Info.x = land_info->x;
  Land_Info.y = land_info->y;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  square_mission.flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  square_mission.display_mode = msg->data;
}

float sum_vision(std_msgs::Float32MultiArray data)
{
  float ans = 0;
  for(int i = 0; i < data.data.size(); i++)
  {
    ans = ans + data.data[i];
  }
  return ans;
}

bool is_get_data()
{
  if(Land_Info.theta == 0 && Land_Info.x == 0 && Land_Info.y == 0)
  {
    return false;
  }
  return true;
}
