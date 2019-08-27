#include "cam/obstacle_detection.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;
    obstacle_pub = nh.advertise<cam::vision_msgs>("/obstacle/msg", 1);

    image_transport::ImageTransport it( nh );
    obstacle_image = it.advertise( "/obstacle/img", 1 );

    frame->encoding = sensor_msgs::image_encodings::BGR8;

    ros::start();
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera_BGR", 1);
    img_sub.registerCallback(image_callback);
    ros::spin();
    ros::shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &img)
{
  Mat resive_im = cv_bridge::toCvShare(img, "bgr8")->image;
  row = resive_im.rows;
  col = resive_im.cols;
  detect(resive_im);
}

void detect(Mat img)
{
  //
  Mat img_src = img;
  cvtColor(img, img, CV_BGR2GRAY);
  equalizeHist(img, img);
  //imshow("lalala", img);
  GaussianBlur(img,dstImage,Size(5,5),0,0);
  //imshow("lblblb", dstImage);

  namedWindow(window_name, WINDOW_AUTOSIZE);
  createTrackbar( T_value, window_name, &threshold_value, 255, Threshold_Demo );
  Threshold_Demo(0, 0);

  Mat element_d = getStructuringElement(MORPH_RECT, Size(g_nKernelSize*2+1,g_nKernelSize*2+1),Point(g_nKernelSize,g_nKernelSize));
  Mat element_e = getStructuringElement(MORPH_RECT, Size((g_nKernelSize-15)*2+1,(g_nKernelSize-15)*2+1),Point(g_nKernelSize-15,g_nKernelSize-15));
  Mat element_e2 = getStructuringElement(MORPH_RECT, Size((g_nKernelSize-15)*2+1,(g_nKernelSize-15)*2+1),Point(g_nKernelSize-18,g_nKernelSize-18));

  Mat fushi;
  dilate(threshold_output, fushi, element_d);//进行腐蚀操作
  erode(fushi, fushi, element_e);
  //imshow("fushi", fushi);//显示效果图

  Canny(fushi, edge, 3, 9, 3);
  dilate(edge, edge, element_e2);
  //imshow("lclclc", edge);

  vector<vector<Point>> contours;    //储存轮廓
  vector<Vec4i> hierarchy;
  findContours(edge, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);    //获取轮廓
  Mat linePic = Mat::zeros(edge.rows, edge.cols, CV_8UC3);
  std::cout<<"contours.size: "<<contours.size()<<std::endl;

  if(contours.size()>1)
  {
    for (int index = 0; index < contours.size(); index++){
            drawContours(linePic, contours, index, Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8/*, hierarchy*/);
    }

    vector<vector<Point>> polyContours(contours.size()), sepolyContours(contours.size());
    int maxArea = 0;
    int semaxArea = 0;
    for (int index = 0; index < contours.size(); index++){
            if (contourArea(contours[index]) > contourArea(contours[maxArea]))
            {
                semaxArea = maxArea;
                maxArea = index;
            }
            approxPolyDP(contours[index], polyContours[index], 10, true);
            approxPolyDP(contours[index], sepolyContours[index], 10, true);
    }
    if (maxArea == 0 && contours.size() > 1)
    {
      semaxArea = 1;
      for (int index = 1; index < contours.size(); index++)
      {
        if (contourArea(contours[index]) > contourArea(contours[semaxArea]))
        {
            semaxArea = index;
        }
        approxPolyDP(contours[index], sepolyContours[index], 10, true);
      }
    }

    std::cout<<maxArea<<"---"<<semaxArea<<std::endl;
    vector<Moments> mu(2);
    mu[0] = moments(polyContours[maxArea], false);
    mu[1] = moments(sepolyContours[semaxArea], false);

    ///  计算中心矩:
    vector<Point2f> mc(2);
    mc[0] = Point2f(mu[0].m10 / mu[0].m00, mu[0].m01 / mu[0].m00);
    mc[1] = Point2f(mu[1].m10 / mu[1].m00, mu[1].m01 / mu[1].m00);

    Mat polyPic = Mat::zeros(img.size(), CV_8UC3);
    drawContours(img_src, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);
    circle(img_src, mc[0], 4, Scalar(255), -1, 8, 0);
    drawContours(img_src, sepolyContours, semaxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);
    circle(img_src, mc[1], 4, Scalar(255), -1, 8, 0);

    obstacle.header.stamp = ros::Time::now();
    obstacle.obsta_info.resize(2);
    obstacle.obsta_info[0].x = mc[0].x;
    obstacle.obsta_info[0].y = mc[0].y;
    obstacle.obsta_info[0].col = col;
    obstacle.obsta_info[0].row = row;

    obstacle.obsta_info[1].x = mc[1].x;
    obstacle.obsta_info[1].y = mc[1].y;
    obstacle.obsta_info[1].col = col;
    obstacle.obsta_info[1].row = row;
    obstacle_pub.publish(obstacle);

    frame->image = img_src;
    frame->header.stamp = ros::Time::now();
    obstacle_image.publish(frame->toImageMsg());
    //imshow("lalala", img_src);
  }

//  addWeighted(polyPic, 0.5, img, 0.5, 0, img);

  waitKey(1);
}

void Threshold_Demo( int, void* )
{

    threshold( dstImage, threshold_output, threshold_value, 255, THRESH_BINARY );

    imshow( window_name, threshold_output );
}

void K_means()
{
  Mat samples(col*row, 1, CV_32FC3);
  Mat labels(col*row, 1, CV_32SC1);
}
