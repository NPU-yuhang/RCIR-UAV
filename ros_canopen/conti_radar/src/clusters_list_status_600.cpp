#include "clusters_list_status_600.h"

ClusterListStatus600::ClusterListStatus600()
{
    num_of_clus = -1;
}

void ClusterListStatus600::unPackBytes(const can_msgs::Frame& can_frame)
{
    num_of_clus = (uint8_t)(can_frame.data[0]) + (uint8_t)(can_frame.data[1]);
}

int ClusterListStatus600::getNumOfClus()
{
    return num_of_clus;
}
