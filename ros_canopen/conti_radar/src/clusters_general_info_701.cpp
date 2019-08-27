//
// Created by zxkj on 18-12-01.
//

#include "clusters_general_info_701.h"
#include <cstdint>
#include <iostream>

ClusterGeneralInfo701::ClusterGeneralInfo701()
{
    cluster_id_ = -1;
    cluster_dist_long_ = -1;
    cluster_dist_lat_ = -1;
    cluster_vre_long_ = -1;
    cluster_vre_lat_ = -1;
    cluster_dyn_prop_ = -1;
    cluster_rcs_ = -1;
}

void ClusterGeneralInfo701::unPackBytes(const can_msgs::Frame& can_frame)
{
    cluster_id_ = (uint8_t)(can_frame.data[0]);
    cluster_dist_long_ = ((uint16_t)(can_frame.data[1]<<5) + (uint8_t)((can_frame.data[2] & 0xf8)>>3)) * 0.2  - 500;
    cluster_dist_lat_ = ((uint16_t)((can_frame.data[2] & 0x03) << 8) + (uint8_t)(can_frame.data[3])) * 0.2 - 102.3;
    cluster_vre_long_ = ((uint16_t)(can_frame.data[4]<<2) + (uint8_t)((can_frame.data[5] & 0xc0)>>6)) * 0.25 - 128;
    cluster_vre_lat_ = ((uint16_t)((can_frame.data[5] & 0x3f) << 3) + (uint8_t)((can_frame.data[6]&0xe0)>>5)) * 0.25 - 64;
    cluster_dyn_prop_ = (uint8_t)(can_frame.data[6] & 0x07);
    cluster_rcs_ = (uint8_t)(can_frame.data[7]) * 0.5 -64.0;
}

int ClusterGeneralInfo701::getClusterID()
{
    return cluster_id_;
}
float ClusterGeneralInfo701::getClusterDistLong()
{
    return cluster_dist_long_;
}
float ClusterGeneralInfo701::getClusterDistLat()
{
    return cluster_dist_lat_;
}
float ClusterGeneralInfo701::getClusterVreLong()
{
    return cluster_vre_long_;
}
float ClusterGeneralInfo701::getClusterVreLat()
{
    return cluster_vre_lat_;
}
int ClusterGeneralInfo701::getClusterDynProp()
{
    return cluster_dyn_prop_;
}
float ClusterGeneralInfo701::getClusterRcs()
{
    return cluster_rcs_;
}
GeneralInfo_Clus ClusterGeneralInfo701::getGeneralInfo_Clus()
{
    GeneralInfo_Clus general_info_;
    general_info_.cluster_id_ = cluster_id_;
    general_info_.cluster_dist_long_ = cluster_dist_long_;
    general_info_.cluster_dist_lat_ = cluster_dist_lat_;
    general_info_.cluster_vre_lat_ = cluster_vre_lat_;
    general_info_.cluster_vre_long_ = cluster_vre_long_;
    general_info_.cluster_dyn_prop_ = cluster_dyn_prop_;
    general_info_.cluster_rcs_ = cluster_rcs_;
    return general_info_;
}
