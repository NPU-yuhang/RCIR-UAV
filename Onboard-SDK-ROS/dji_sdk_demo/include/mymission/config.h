#ifndef CONFIG_H
#define CONFIG_H
#include <stdlib.h>
#include <opencv2/opencv.hpp>
class param{
public:

    param()
    {

    }

    double GPS_nav_Kp;
    double GPS_nav_Ki;
    double GPS_nav_Kd;
    double GPS_nav_Ka;
    double GPS_nav_Kv;

    double vision_nav_Kp;
    double vision_nav_Ki;
    double vision_nav_Kd;
    double vision_nav_Ka;
    double vision_nav_Kv;

    double vision_nav_Kp_low;
    double vision_nav_Ki_low;
    double vision_nav_Kd_low;
    double vision_nav_Ka_low;
    double vision_nav_Kv_low;

    double yaw_nav_Kp;
    double yaw_nav_Ki;
    double yaw_nav_Kd;
    double yaw_nav_Ka;
    double yaw_nav_Kv;

    double velo_lim;

    float goal_x;
    float goal_y;

    void readparam(const std::string path)
    {
        cv::FileStorage fSettings(path, cv::FileStorage::READ);

        if(!fSettings.isOpened())
        {
           std::cerr << "Failed to open config file at: " << path << std::endl;
           exit(-1);
        }
        getparam(fSettings);
    }

    void getparam(cv::FileStorage fSettings)
    {
        GPS_nav_Kp = (double)fSettings["GPSnavigationKp"];
        GPS_nav_Ki = (double)fSettings["GPSnavigationKi"];
        GPS_nav_Kd = (double)fSettings["GPSnavigationKd"];
        GPS_nav_Ka = (double)fSettings["GPSnavigationKa"];
        GPS_nav_Kv = (double)fSettings["GPSnavigationKv"];

        vision_nav_Kp = (double)fSettings["visionnavigationKp"];
        vision_nav_Ki = (double)fSettings["visionnavigationKi"];
        vision_nav_Kd = (double)fSettings["visionnavigationKd"];
        vision_nav_Ka = (double)fSettings["visionnavigationKa"];
        vision_nav_Kv = (double)fSettings["visionnavigationKv"];

        vision_nav_Kp_low = (double)fSettings["visionnavigation2Kp"];
        vision_nav_Ki_low = (double)fSettings["visionnavigation2Ki"];
        vision_nav_Kd_low = (double)fSettings["visionnavigation2Kd"];
        vision_nav_Ka_low = (double)fSettings["visionnavigation2Ka"];
        vision_nav_Kv_low = (double)fSettings["visionnavigation2Kv"];

        yaw_nav_Kp = (double)fSettings["yawnavigationKp"];
        yaw_nav_Ki = (double)fSettings["yawnavigationKi"];
        yaw_nav_Kd = (double)fSettings["yawnavigationKd"];
        yaw_nav_Ka = (double)fSettings["yawnavigationKa"];
        yaw_nav_Kv = (double)fSettings["yawnavigationKv"];

        velo_lim = (double)fSettings["velolim"];

        goal_x = (float)fSettings["goalx"];
        goal_y = (float)fSettings["goaly"];

    }
};
#endif // CONFIG_H
