#ifndef CLUSTERS_LIST_STATUS_600_H
#define CLUSTERS_LIST_STATUS_600_H

#include <iostream>
#include <can_msgs/Frame.h>

class ClusterListStatus600
{
private:
    int num_of_clus;
public:
    ClusterListStatus600();
    void unPackBytes(const can_msgs::Frame& can_frame);
    int getNumOfClus();
};

#endif // CLUSTERS_LIST_STATUS_600_H
