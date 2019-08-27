//
// Created by zxkj on 18-12-01.
//

#include "object_list_status_60a.h"

ObjectListStatus60A::ObjectListStatus60A()
{
    num_of_obj = -1;
}

void ObjectListStatus60A::unPackBytes(const can_msgs::Frame& can_frame)
{
    num_of_obj = (uint8_t)(can_frame.data[0]);
}

int ObjectListStatus60A::getNumOfObj()
{
    return num_of_obj;
}



