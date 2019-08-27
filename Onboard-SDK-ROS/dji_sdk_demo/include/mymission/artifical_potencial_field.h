#ifndef ARTIFICAL_POTENCIAL_FIELD_H
#define ARTIFICAL_POTENCIAL_FIELD_H

#include <string.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

const double PI = 3.141592653;

class artifical_potencial_field
{
private:
    cv::Point2f X0;
    int att_gain_coe;
    int rep_gain_coe;
    int count;
    float P0;
    int obstacle;
    float a;
    float l;
    int iteration;
    std::vector<cv::Point2f> Xsum;
    cv::Point2f current_x;
    cv::Point2f next_x;
public:
    std::vector<cv::Point2f> Goal;

public:
    artifical_potencial_field();
    void path_planning();
    void SetObstacle(std::vector<cv::Point2f> obs);
    void SetStartPoint(cv::Point2f X_s);
    float get_path_length();

private:
    std::vector<float> compute_angle(cv::Point2f X, std::vector<cv::Point2f> Xs, int n_obs);
    cv::Point2f compute_Attact(cv::Point2f X, std::vector<cv::Point2f> Xs, int att_gain, float angle, int b, float P0, int n_obs);
    std::vector<cv::Point2f> compute_repulsion(cv::Point2f X, std::vector<cv::Point2f> Xs, int rep_gain, float angle_at, std::vector<float> angle_re, int n_obs, float P0, float a);

};

#endif // ARTIFICAL_POTENCIAL_FIELD_H
