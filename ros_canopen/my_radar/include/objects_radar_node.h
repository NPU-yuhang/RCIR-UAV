#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <can_msgs/Frame.h>
#include <object_extended_info_60d.h>
#include <object_general_info_60b.h>
#include <object_list_status_60a.h>
#include <object_quality_info_60c.h>
#include <radar_state_201.h>
#include <my_radar/object.h>
#include <my_radar/objects.h>

ObjectExtendedInfo60D Obj_exten_info = ObjectExtendedInfo60D();
ObjectGeneralInfo60B Obj_gene_info = ObjectGeneralInfo60B();
ObjectListStatus60A Obj_list_status = ObjectListStatus60A();
ObjectQualityInfo60C Obj_qua_info = ObjectQualityInfo60C();
RadarState201 Radar_state = RadarState201();
ros::Publisher my_frame_pub;
my_radar::objects my_objects;
bool is_first_frame;

void frame_callback(const can_msgs::FrameConstPtr &frame);
