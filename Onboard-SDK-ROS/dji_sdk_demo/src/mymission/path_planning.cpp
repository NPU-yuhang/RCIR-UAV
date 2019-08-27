#include <iostream>
#include <fstream>
#include <string.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pangolin/pangolin.h>
#include <mymission/artifical_potencial_field.h>

int main(int argc, char** argv)
{
    artifical_potencial_field Artifical = artifical_potencial_field();
    cv::Point2f StartPo = cv::Point2f(0, 0);
    Artifical.SetStartPoint(StartPo);
    std::vector<cv::Point2f> obs;
    obs.push_back(cv::Point2f(5, 15));
    obs.push_back(cv::Point2f(-0.8, 2));
    obs.push_back(cv::Point2f(3.2, 5.6));
    Artifical.SetObstacle(obs);
    Artifical.path_planning();

    return 0;
}
