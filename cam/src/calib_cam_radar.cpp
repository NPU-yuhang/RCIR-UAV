#include "cam/calib_cam_radar.h"

void calibCallback(const sensor_msgs::ImageConstPtr &img, const my_radar::objectsConstPtr &radar)
{
  int count = radar->num;
  radar_data = cv::Mat::zeros(2,count,CV_32FC1);
  for(int i=0; i<count; i++)
  {
    if(std::abs(radar->objs[i].Lat)<2)
    {
      radar_data.at<float>(0,i) = -(radar->objs[i].Lat);
      radar_data.at<float>(1,i) = radar->objs[i].Long;
    }
  }
  img_src = cv_bridge::toCvShare(img, "bgr8")->image;
  onChange(0, &img_src);
  cv::waitKey(1);
}

void onChange(int value,void* param)
{
    //获取waveX,waveY
    //X:0-10,对应-5--5
    //Y:0-10 对应0-50
    float k_waveX = (-0.01-0.01) / (0 - 100);
    float b_waveX = -0.01 - k_waveX * 0;

    float k_waveY = (-0.01-0.01) / (0 - 100);
    float b_waveY = -0.01 - k_waveY * 0;

    float k_waveZ = (-0.01-0.01) / (0 - 100);
    float b_waveZ = -0.01 - k_waveZ * 0;

    //获取pitch,yaw角度
    //0-3000,0对应-30.0;3000对应30.0;
    //x     0       3000.0
    //y    -30.0     30.0
    float k_pitch = (60.0-120.0)/(0-3000.0);
    float b_pitch = 60.0 - k_pitch*0;

    float k_yaw = (-30.0-30.0)/(0-3000.0);
    float b_yaw = -30.0 - k_yaw*0;

    float k_roll = (-30.0-30.0)/(0-3000.0);
    float b_roll = -30.0 - k_roll*0;

    float waveX = (float)cv::getTrackbarPos("waveX/m","TrackBar");
    waveX = k_waveX*waveX + b_waveX;

    float waveY = (float)cv::getTrackbarPos("waveY/m","TrackBar");
    waveY = k_waveY*waveY + b_waveY;

    float waveZ = (float)cv::getTrackbarPos("waveZ/m","TrackBar");
    waveZ = k_waveZ*waveZ + b_waveZ;

    float pitch = (float)cv::getTrackbarPos("pitch/°(60°-120°)","TrackBar");
    pitch = k_pitch*pitch + b_pitch;

    float yaw = (float)cv::getTrackbarPos("yaw/°(-30°-30°)","TrackBar");
    yaw = k_yaw*yaw + b_yaw;

    float roll = (float)cv::getTrackbarPos("roll/°(-30°-30°)","TrackBar");
    roll = k_roll*roll + b_roll;

    //====================图像处理========================//
    cv::Mat posInImage,posInCamera;
    cv::Mat src=*(cv::Mat*)param;
    cv::Mat dst;
    dst.release();
    {
        CameraParam cameraPara;
        cameraPara.fu = 630.489651;
        cameraPara.fv = 630.434369;
        cameraPara.cu = 400.735172;//
        cameraPara.cv = 308.768174;
        cameraPara.height = 1750;//mm
        cameraPara.pitch = pitch*(CV_PI*1.0/180.0);
        cameraPara.yaw  = yaw*(CV_PI*1.0/180.0);
        cameraPara.roll = roll*(CV_PI*1.0/180.0);
        cameraPara.image_width = 800;
        cameraPara.image_height = 600;

        cameraPara.waveincamera_x = waveX;//mm;
        cameraPara.waveincamera_y = waveY;
        cameraPara.waveincamera_z = waveZ;
        cameraPara.objectheight = 1.0;//mm
        cameraPara.objectwidth = 1.5;//mm

        waveradar2image waveRadar2Image = waveradar2image();
        waveRadar2Image.set_param(cameraPara);

        cv::Mat posInImage = waveRadar2Image.TransformWRadar2Image2(radar_data);

        dst = src.clone();

        if( dst.channels() == 1 )
            cv::cvtColor(dst,dst,CV_GRAY2BGR);

        cv::Mat temp(100,dst.cols,CV_8UC3,cv::Scalar(255,255,255));
        cv::vconcat(dst,temp,dst);

        int nObjectNum = posInImage.cols;
        //std::cout<<"nObjectNum: "<<nObjectNum<<std::endl;
        for(int i=0; i!=nObjectNum;++i)
        {
            cv::rectangle(dst,cv::Rect(posInImage.at<float>(0,i),
                                       posInImage.at<float>(1,i),
                                       posInImage.at<float>(2,i)-posInImage.at<float>(0,i),
                                       posInImage.at<float>(3,i)-posInImage.at<float>(1,i)),cv::Scalar(0,255,0),2);

            cv::circle(dst,cv::Point((posInImage.at<float>(0,i)+posInImage.at<float>(2,i))/2,
                                     (posInImage.at<float>(1,i)+posInImage.at<float>(3,i))/2),3,cv::Scalar(0,255,0),4);

            //std::cout<<"width in pixel="<<posInImage.at<float>(2,i)<<std::endl;
        }

        cv::circle(dst,cv::Point(cameraPara.cu,cameraPara.cv),6,cv::Scalar(0,0,255),10);
    }

    //cv::rectangle(dst,cv::Point(0,0),cv::Point(220,110),cv::Scalar(255,255,0),1);
    int nHeight = dst.rows;
    int nWidth = dst.cols;
    cv::line(dst,cv::Point(nWidth/2,0),cv::Point(nWidth/2,nHeight),cv::Scalar(0,0,255),2);
    cv::line(dst,cv::Point(0,nHeight/2),cv::Point(nWidth,nHeight/2),cv::Scalar(0,0,255),2);

    cv::line(dst,cv::Point(nWidth/4,0),cv::Point(nWidth/4,nHeight),cv::Scalar(0,0,255),1);
    cv::line(dst,cv::Point(nWidth*3/4,0),cv::Point(nWidth*3/4,nHeight),cv::Scalar(0,0,255),1);


    cv::line(dst,cv::Point(0,nHeight/4),cv::Point(nWidth,nHeight/4),cv::Scalar(0,0,255),1);
    cv::line(dst,cv::Point(0,nHeight*3/4),cv::Point(nWidth,nHeight*3/4),cv::Scalar(0,0,255),1);

    char info[256];
    sprintf(info,"waveX:%.4f",waveX);
    cv::putText(dst,info,cv::Point(0,20),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);

    sprintf(info,"waveY:%.4f",waveY);
    cv::putText(dst,info,cv::Point(0,45),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);

    sprintf(info,"waveZ:%.4f",waveZ);
    cv::putText(dst,info,cv::Point(0,70),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);

    sprintf(info,"roll:%.1f",roll);
    cv::putText(dst,info,cv::Point(0,95),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);

    sprintf(info,"pitch:%.1f",pitch);
    cv::putText(dst,info,cv::Point(0,120),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);

    sprintf(info,"yaw:%.1f",yaw);
    cv::putText(dst,info,cv::Point(0,145),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);

    cv::imshow("TrackBar",dst);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calib_cam_radar");
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, my_radar::objects> topic_synchronizer;
  message_filters::Subscriber<sensor_msgs::Image>* img_sub;
  message_filters::Subscriber<my_radar::objects>* radar_sub;

  message_filters::Synchronizer<topic_synchronizer>* sync;
  img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera_BGR", 100);
  //scan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "/scan", 100);
  radar_sub = new message_filters::Subscriber<my_radar::objects>(nh, "/objects/msg", 100);
  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(10), *img_sub, *radar_sub);

  cv::namedWindow("TrackBar",CV_WINDOW_AUTOSIZE);
  int s = 50;
  cv::createTrackbar("waveX/m","TrackBar",&s,100,onChange,&img_src);
  cv::createTrackbar("waveY/m","TrackBar",&s,100,onChange,&img_src);
  cv::createTrackbar("waveZ/m","TrackBar",&s,100,onChange,&img_src);

  s = 1500;
  cv::createTrackbar("pitch/°(60°-120°)","TrackBar",&s,3000,onChange,&img_src);
  cv::createTrackbar("yaw/°(-30°-30°)","TrackBar",&s,3000,onChange,&img_src);
  cv::createTrackbar("roll/°(-30°-30°)","TrackBar",&s,3000,onChange,&img_src);

  sync->registerCallback(boost::bind(&calibCallback, _1, _2));

  ros::spin();
  ros::shutdown();
  return 0;
}
