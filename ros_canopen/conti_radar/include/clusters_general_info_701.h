#ifndef CLUSTERS_GENERAL_INFO_701_H
#define CLUSTERS_GENERAL_INFO_701_H

#include <vector>
#include <can_msgs/Frame.h>

struct GeneralInfo_Clus
{
    int cluster_id_;
    float cluster_dist_long_;
    float cluster_dist_lat_;
    float cluster_vre_long_;
    float cluster_vre_lat_;
    int cluster_dyn_prop_;
    float cluster_rcs_;
};

class ClusterGeneralInfo701
{
private:
    int cluster_id_;
    float cluster_dist_long_;
    float cluster_dist_lat_;
    float cluster_vre_long_;
    float cluster_vre_lat_;
    int cluster_dyn_prop_;
    float cluster_rcs_;

public:
    ClusterGeneralInfo701();
    void unPackBytes(const can_msgs::Frame& can_frame);
    int getClusterID();
    float getClusterDistLong();
    float getClusterDistLat();
    float getClusterVreLong();
    float getClusterVreLat();
    int getClusterDynProp();
    float getClusterRcs();
    GeneralInfo_Clus getGeneralInfo_Clus();
};
#endif // CLUSTERS_GENERAL_INFO_701_H
