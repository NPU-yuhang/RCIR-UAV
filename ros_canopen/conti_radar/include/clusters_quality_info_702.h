#ifndef CLUSTERS_EXTENDED_INFO_702_H
#define CLUSTERS_EXTENDED_INFO_702_H
#include <can_msgs/Frame.h>

struct QualityInfo_Cluster
{
    int cluster_id_;
    int cluster_dist_long_rms_;
    int cluster_vrel_long_rms_;
    int cluster_dist_lat_rms_;
    int cluster_vrel_lat_rms_;
    int cluster_pd_h0_;
    int cluster_ambig_state_;
    int cluster_invalid_state_;
};


class ClusterQualityInfo702
{
private:
  int cluster_id_;
  int cluster_dist_long_rms_;
  int cluster_vrel_long_rms_;
  int cluster_dist_lat_rms_;
  int cluster_vrel_lat_rms_;
  int cluster_pd_h0_;
  int cluster_ambig_state_;
  int cluster_invalid_state_;

public:
    ClusterQualityInfo702();
    void unPackBytes(const can_msgs::Frame& can_frame);

    int getClusterId();
    int getClusterDistLongRms();
    int getClusterVrelLongRms();
    int getClusterDistLatRms();
    int getClusterVrelLatRms();
    int getClusterPdh0();
    int getClusterAmbigState();
    int getClusterInvalidState();

    QualityInfo_Cluster getQualityInfo_Cluster();
};
#endif // CLUSTERS_EXTENDED_INFO_702_H
