#ifndef WAVERADAR2IMAGE_H
#define WAVERADAR2IMAGE_H
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
//system
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

class CameraParam
{
public:
    double fu, fv, cu, cv;
    double height;
    double pitch, yaw, roll;
    int image_width, image_height;
    double waveincamera_x, waveincamera_y, waveincamera_z;
    double objectwidth, objectheight;
};

class waveradar2image
{
public:
    waveradar2image() {}
    void set_param(CameraParam cam_param)
    {
        cv::Mat Rx = (cv::Mat_<float>(3,3)<<1,0,0,0,std::cos(cam_param.pitch),
                      -std::sin(cam_param.pitch),0,std::sin(cam_param.pitch),
                      std::cos(cam_param.pitch));
        cv::Mat Ry = (cv::Mat_<float>(3,3)<<std::cos(cam_param.yaw),0,std::sin(cam_param.yaw),
                      0,1,0,-std::sin(cam_param.yaw),0,std::cos(cam_param.yaw));
        cv::Mat Rz = (cv::Mat_<float>(3,3)<<std::cos(cam_param.roll),-std::sin(cam_param.roll),
                      0,std::sin(cam_param.roll),std::cos(cam_param.roll),0,0,0,1);

        cv::Mat R = Rz*Ry*Rx;

        Tc_r = (cv::Mat_<float>(4,4)<<R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),cam_param.waveincamera_x,
                R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),cam_param.waveincamera_y,
                R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),cam_param.waveincamera_z,
                0,0,0,1);

        K = (cv::Mat_<float>(4,4)<<cam_param.fu,0,cam_param.cu,0,
                     0,cam_param.fv,cam_param.cv,0,
                     0,0,1,0,0,0,0,1);

        object_w = cam_param.objectwidth;
        object_h = cam_param.objectheight;
    }

    cv::Mat Tc_r;//External reference to be calibrated
    cv::Mat K;//internal reference
    double object_w, object_h;
    int object_num = 0;
    cv::Mat TransformWRadar2Image2(cv::Mat pos_wave)
    {
        cv::Mat pos_image;
        object_num = pos_wave.cols;
        //std::cout<<"pos_wave: "<<pos_wave<<std::endl;
        pos_image = cv::Mat::zeros(4,object_num,CV_32FC1);
        cv::Mat pos_wave_1, pos_wave_2, pos_camera_1, pos_camera_2, pos_image_1, pos_image_2;
        pos_wave_1 = cv::Mat::zeros(4,object_num,CV_32FC1);
        pos_wave_2 = cv::Mat::zeros(4,object_num,CV_32FC1);
        for(int i=0; i<object_num; i++)
        {
          pos_wave_1.at<float>(0,i) = pos_wave.at<float>(0,i)-object_w/2;
          pos_wave_1.at<float>(1,i) = pos_wave.at<float>(1,i);
          pos_wave_1.at<float>(2,i) = object_h/2;
          pos_wave_1.at<float>(3,i) = 1;

          pos_wave_2.at<float>(0,i) = pos_wave.at<float>(0,i)+object_w/2;
          pos_wave_2.at<float>(1,i) = pos_wave.at<float>(1,i);
          pos_wave_2.at<float>(2,i) = -object_h/2;
          pos_wave_2.at<float>(3,i) = 1;
        }
        pos_camera_1 = Tc_r*pos_wave_1;
        pos_camera_2 = Tc_r*pos_wave_2;
        //std::cout<<"pos_camera_1: "<<pos_camera_1<<std::endl;
        //std::cout<<"Tc_r: "<<Tc_r<<std::endl;
//        std::cout<<"pos_wave_1: "<<pos_wave_1<<std::endl;
//        std::cout<<"pos_wave_2: "<<pos_wave_2<<std::endl;
//        pos_camera = (cv::Mat_<float>(3,1)<<(pos_camera_1.at<float>(0,0)+pos_camera_2.at<float>(0,0))/2,
//                      (pos_camera_1.at<float>(1,0)+pos_camera_2.at<float>(1,0))/2,
//                      (pos_camera_1.at<float>(2,0)+pos_camera_2.at<float>(2,0))/2);

        pos_image_1 = K*pos_camera_1;
        pos_image_2 = K*pos_camera_2;
        //std::cout<<"pos_image: "<<pos_image_1<<std::endl;
        cv::Mat scale_1(4,object_num,CV_32FC1), scale_2(4,object_num,CV_32FC1);
        pos_camera_1.row(2).copyTo(scale_1.row(0));
        pos_camera_1.row(2).copyTo(scale_1.row(1));
        pos_camera_1.row(2).copyTo(scale_1.row(2));
        pos_camera_1.row(2).copyTo(scale_1.row(3));

        pos_camera_2.row(2).copyTo(scale_2.row(0));
        pos_camera_2.row(2).copyTo(scale_2.row(1));
        pos_camera_2.row(2).copyTo(scale_2.row(2));
        pos_camera_2.row(2).copyTo(scale_2.row(3));

        scale_1 = 1/scale_1;
        scale_2 = 1/scale_2;
        pos_image_1 = pos_image_1.mul(scale_1);
        pos_image_2 = pos_image_2.mul(scale_2);

        pos_image_1.row(0).copyTo(pos_image.row(0));
        pos_image_1.row(1).copyTo(pos_image.row(1));
        pos_image_2.row(0).copyTo(pos_image.row(2));
        pos_image_2.row(1).copyTo(pos_image.row(3));
        //std::cout<<pos_image<<std::endl;
        return pos_image;
    }

    cv::Mat TransformWRadarPoint2ImagePoint(cv::Mat pos_wave)
    {
      cv::Mat pos_image;
      object_num = pos_wave.cols;
      //std::cout<<"pos_wave: "<<pos_wave<<std::endl;
      pos_image = cv::Mat::zeros(2,object_num,CV_32FC1);
      cv::Mat pos_wave_4, pos_camera;
      pos_wave_4 = cv::Mat::zeros(4,object_num,CV_32FC1);
      for(int i=0; i<object_num; i++)
      {
        pos_wave_4.at<float>(0,i) = pos_wave.at<float>(0,i);
        pos_wave_4.at<float>(1,i) = pos_wave.at<float>(1,i);
        pos_wave_4.at<float>(2,i) = 0;
        pos_wave_4.at<float>(3,i) = 1;
      }

      pos_camera = Tc_r*pos_wave_4;
      pos_image = K*pos_camera;

      //std::cout<<"pos_image: "<<pos_image_1<<std::endl;
      cv::Mat scale(4,object_num,CV_32FC1);
      pos_camera.row(2).copyTo(scale.row(0));
      pos_camera.row(2).copyTo(scale.row(1));
      pos_camera.row(2).copyTo(scale.row(2));
      pos_camera.row(2).copyTo(scale.row(3));
      scale = 1/scale;
      pos_image = pos_image.mul(scale);
      //std::cout<<pos_image<<std::endl;
      return pos_image;
    }

    cv::Mat TransformWRadar2Camera(cv::Mat pos_wave)
    {
      cv::Mat pos_camera;
      object_num = pos_wave.cols;
      //std::cout<<"pos_wave: "<<pos_wave<<std::endl;
      cv::Mat pos_wave_4;
      pos_wave_4 = cv::Mat::zeros(4,object_num,CV_32FC1);

      for(int i=0; i<object_num; i++)
      {
        pos_wave_4.at<float>(0,i) = pos_wave.at<float>(0,i);
        pos_wave_4.at<float>(1,i) = pos_wave.at<float>(1,i);
        pos_wave_4.at<float>(2,i) = 0;
        pos_wave_4.at<float>(3,i) = 1;
      }
      pos_camera = Tc_r*pos_wave_4;
      return pos_camera;
    }

};

#endif // WAVERADAR2IMAGE_H
