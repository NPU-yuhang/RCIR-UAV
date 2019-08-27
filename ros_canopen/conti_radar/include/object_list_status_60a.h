//
// Created by zxkj on 18-12-01.
//

#ifndef CONTI_RADAR_OBJECT_LIST_STATUS_60A_H
#define CONTI_RADAR_OBJECT_LIST_STATUS_60A_H

#include <iostream>
#include <can_msgs/Frame.h>

class ObjectListStatus60A
{
private:
    int num_of_obj;
public:
    ObjectListStatus60A();
    void unPackBytes(const can_msgs::Frame& can_frame);
    int getNumOfObj();
};

#endif //CONTI_RADAR_OBJECT_LIST_STATUS_60A_H
