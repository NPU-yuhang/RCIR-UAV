#ifndef CLUSTERS_RADAR_NODE_H
#define CLUSTERS_RADAR_NODE_H
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
#include <clusters_general_info_701.h>
#include <clusters_list_status_600.h>
#include <clusters_quality_info_702.h>
#include <radar_state_201.h>
#include <my_radar/object.h>
#include <my_radar/objects.h>

ClusterGeneralInfo701 Clus_gene_info = ClusterGeneralInfo701();
ClusterListStatus600 Clus_list_status = ClusterListStatus600();
ClusterQualityInfo702 Clus_qua_info = ClusterQualityInfo702();
ros::Publisher my_frame_pub;
my_radar::objects my_clusters;
bool is_first_frame;

void frame_callback(const can_msgs::FrameConstPtr &frame);

#endif // CLUSTERS_RADAR_NODE_H
