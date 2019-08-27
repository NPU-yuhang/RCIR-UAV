#include <stdlib.h>
#include <opencv2/opencv.hpp>
class param{
public:

    param()
    {

    }

    float camera_fx;
    float camera_fy;
    float camera_cx;
    float camera_cy;
    float camera_k1;
    float camera_k2;
    float camera_p1;
    float camera_p2;
    float camera_p3;
    std::string radar_mod;

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
        camera_fx = (double)fSettings["fx"];
        camera_fy = (double)fSettings["fy"];
        camera_cx = (double)fSettings["cx"];
        camera_cy = (double)fSettings["cy"];
        camera_k1 = (double)fSettings["k1"];
        camera_k2 = (double)fSettings["k2"];
        camera_p1 = (double)fSettings["p1"];
        camera_p2 = (double)fSettings["p2"];
        camera_p3 = (double)fSettings["p3"];
        radar_mod = (std::string)fSettings["radar"];
    }
};

