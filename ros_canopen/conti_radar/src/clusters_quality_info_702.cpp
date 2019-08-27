//
// Created by zxkj on 18-12-01.
//

#include "clusters_quality_info_702.h"
#include <cstdint>
#include <iostream>

ClusterQualityInfo702::ClusterQualityInfo702()
{
    cluster_id_ = -1;
    cluster_dist_long_rms_ = -1;
    cluster_vrel_long_rms_ = -1;
    cluster_dist_lat_rms_ = -1;
    cluster_vrel_lat_rms_ = -1;
    cluster_pd_h0_ = -1;
    cluster_ambig_state_ = -1;
    cluster_invalid_state_ = -1;
}

void ClusterQualityInfo702::unPackBytes(const can_msgs::Frame &can_frame)
{
    cluster_id_ = (uint8_t)(can_frame.data[0]);
    cluster_dist_long_rms_ = (uint8_t)((can_frame.data[1] & 0xf8) >> 3);
    cluster_dist_lat_rms_ = (uint8_t)((can_frame.data[1] & 0x07)<<2 + (can_frame.data[2] & 0xc0) >>6);
    cluster_vrel_long_rms_ = (uint8_t)((can_frame.data[2] & 0x3e) >> 1);
    cluster_vrel_lat_rms_ = (uint8_t)((can_frame.data[2] & 0x01)<< 4 + (can_frame.data[3] &0xf0) >> 4);
    cluster_pd_h0_ = (uint8_t)(can_frame.data[3] & 0x07);
    cluster_ambig_state_ = (uint8_t)(can_frame.data[4] & 0x07);
    cluster_invalid_state_ = (uint8_t)(can_frame.data[4] & 0xf8);
}

int ClusterQualityInfo702::getClusterId() {
    return cluster_id_;
}

int ClusterQualityInfo702::getClusterDistLongRms() {
    return cluster_dist_long_rms_;
}

int ClusterQualityInfo702::getClusterDistLatRms() {
    return cluster_dist_lat_rms_;
}

int ClusterQualityInfo702::getClusterVrelLongRms() {
    return cluster_vrel_long_rms_;
}

int ClusterQualityInfo702::getClusterVrelLatRms() {
    return cluster_vrel_lat_rms_;
}

int ClusterQualityInfo702::getClusterPdh0() {
    return cluster_pd_h0_;
}

int ClusterQualityInfo702::getClusterAmbigState() {
    return cluster_ambig_state_;
}

int ClusterQualityInfo702::getClusterInvalidState() {
    return cluster_invalid_state_;
}

QualityInfo_Cluster ClusterQualityInfo702::getQualityInfo_Cluster()
{
    QualityInfo_Cluster quality_info;
    quality_info.cluster_id_ = cluster_id_;
    quality_info.cluster_dist_long_rms_ = cluster_dist_long_rms_;
    quality_info.cluster_dist_lat_rms_ = cluster_dist_lat_rms_;
    quality_info.cluster_vrel_long_rms_ = cluster_vrel_long_rms_;
    quality_info.cluster_vrel_lat_rms_ = cluster_vrel_lat_rms_;
    quality_info.cluster_pd_h0_ = cluster_pd_h0_;
    quality_info.cluster_ambig_state_ = cluster_ambig_state_;
    quality_info.cluster_invalid_state_ = cluster_invalid_state_;
    return quality_info;
}

