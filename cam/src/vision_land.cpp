#include <cam/vision_land.h>

using namespace cv;
using namespace std;
const double PI = 3.141592653;
int threshold_value = 240;
const string T_value = "threshold_value";
const string window_name = "yuchulihou";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_land");
    ros::NodeHandle nh;
    vision_pub = nh.advertise<cam::vision_msg>("vision/msg", 1);
    ros::start();
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/my_stereo/left/image_raw", 1);
    img_sub.registerCallback(image_callback);
    ros::spin();
    ros::shutdown();
}

cam::vision_msg vision_location()
{
  cam::vision_msg v_msg;
  v_msg.header.stamp = ros::Time::now();
  //resize(src, src, cv::Size(640,640));
  Mat src_all = src.clone();
  //cvtColor(src, src, CV_BGR2GRAY);
  //  src_gray = Scalar::all(255) - src_gray;
  blur(src, src, Size(3, 3));
  equalizeHist(src, src);
  //imshow("lalala", src);

  Scalar color = Scalar(1, 1, 255);

  vector<vector<Point> > contours, contours2;
  vector<Vec4i> hierarchy;
  Mat drawing = Mat::zeros(src.size(), CV_8UC3);
  Mat drawing2 = Mat::zeros(src.size(), CV_8UC3);
  namedWindow(window_name, WINDOW_AUTOSIZE);
  createTrackbar( T_value, window_name, &threshold_value, 255, Threshold_Demo );
//  threshold(src, threshold_output, threashold_value, 255, THRESH_BINARY);
//  //Canny(src, threshold_output,136,196,3);
//  imshow("yuchulihou",threshold_output);
  Threshold_Demo(0, 0);

  //寻找轮廓
  //第一个参数是输入图像 2值化的
  //第二个参数是内存存储器，FindContours找到的轮廓放到内存里面。
  //第三个参数是层级，**[Next, Previous, First_Child, Parent]** 的vector
  //第四个参数是类型，采用树结构
  //第五个参数是节点拟合模式，这里是全部寻找
  findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
  //CHAIN_APPROX_NONE全体,CV_CHAIN_APPROX_SIMPLE,,,RETR_TREE    RETR_EXTERNAL    RETR_LIST   RETR_CCOMP

  int c = 0, ic = 0, k = 0, area = 0;

  //程序的核心筛选
  int parentIdx = -1;
  for (int i = 0; i< contours.size(); i++)
  {
      if (hierarchy[i][2] != -1 && ic == 0)
      {
          parentIdx = i;
          ic++;
      }
      else if (hierarchy[i][2] != -1)
      {
          ic++;
      }
      else if (hierarchy[i][2] == -1)
      {
          ic = 0;
          parentIdx = -1;
      }

      if (ic >= 2)
      {
          contours2.push_back(contours[parentIdx]);
          drawContours(drawing, contours, parentIdx, CV_RGB(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 1, 8);
          ic = 0;
          parentIdx = -1;
          area = contourArea(contours[i]);//得出一个二维码定位角的面积，以便计算其边长（area_side）（数据覆盖无所谓，三个定位角中任意一个数据都可以）
      }
      //cout<<"i= "<<i<<" hierarchy[i][2]= "<<hierarchy[i][2]<<" parentIdx= "<<parentIdx<<" ic= "<<ic<<endl;
  }
  std::cout<<"coutours2 size: "<<contours2.size()<<std::endl;
  if(contours2.size()>=3)
  {
    for (int i = 0; i<contours2.size(); i++)
        drawContours(drawing2, contours2, i, CV_RGB(rng.uniform(100, 255), rng.uniform(100, 255), rng.uniform(100, 255)), -1, 4, hierarchy[k][2], 0, Point());

    Point2f point[3];
    for (int i = 0; i<contours2.size(); i++)
    {
        point[i] = Center_cal(contours2, i);
        std::cout<<point[i].x<<" "<<point[i].y<<std::endl;
    }

    double line_max, theta;
    double line1 = sqrt(pow((point[0].x - point[1].x), 2) + pow((point[0].y - point[1].y), 2));
    double line2 = sqrt(pow((point[1].x - point[2].x), 2) + pow((point[1].y - point[2].y), 2));
    double line3 = sqrt(pow((point[0].x - point[2].x), 2) + pow((point[0].y - point[2].y), 2));
    std::cout<<line1<<"  "<<line2<<"  "<<line3<<std::endl;
    line_max = line1;
    theta = atan((point[0].y - point[1].y) / (point[1].x - point[0].x));
    if(point[2].y > ((point[0].y + point[1].y)/2) && point[2].x > ((point[0].x + point[1].x)/2))
      theta = theta - PI;
    if(point[2].y > ((point[0].y + point[1].y)/2) && point[2].x < ((point[0].x + point[1].x)/2))
      theta = theta + PI;
    if(line_max < line2)
    {
      line_max = line2;
      theta = atan((point[1].y - point[2].y) / (point[2].x - point[1].x));
      if(point[0].y > ((point[1].y + point[2].y)/2) && point[0].x > ((point[1].x + point[2].x)/2))
        theta = theta - PI;
      if(point[0].y > ((point[1].y + point[2].y)/2) && point[0].x < ((point[1].x + point[2].x)/2))
        theta = theta + PI;
    }
    if(line_max < line3)
    {
      line_max = line3;
      theta = atan((point[0].y - point[2].y) / (point[2].x - point[0].x));
      if(point[1].y > ((point[0].y + point[2].y)/2) && point[1].x > ((point[0].x + point[2].x)/2))
        theta = theta - PI;
      if(point[1].y > ((point[0].y + point[2].y)/2) && point[1].x < ((point[0].x + point[2].x)/2))
        theta = theta + PI;
    }
    theta = theta*180/PI;
    theta = theta - 45;
    if(theta>-225 && theta<-180)
      theta = theta+360;
    std::cout<<"line_max: "<<line_max<<"---theta: "<<theta<<std::endl;

    v_msg.theta = theta;

    area = contourArea(contours2[1]);//为什么这一句和前面一句计算的面积不一样呢
    int area_side = cvRound(sqrt(double(area)));
    for (int i = 0; i<contours2.size(); i++)
    {
        line(drawing2, point[i%contours2.size()], point[(i + 1) % contours2.size()], color, area_side / 4, 8);
    }

    //imshow("tiquhou", drawing2);
    printf("%d\n", contours.size());
    //imshow( "Contours", drawing );

    //接下来要框出这整个二维码
    Mat gray_all, threshold_output_all;
    vector<vector<Point> > contours_all;
    vector<Vec4i> hierarchy_all;
    cvtColor(drawing2, gray_all, CV_BGR2GRAY);


    threshold(gray_all, threshold_output_all, 45, 255, THRESH_BINARY);

    //表示只寻找最外层轮廓
    findContours(threshold_output_all, contours_all, hierarchy_all, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));//RETR_EXTERNAL表示只寻找最外层轮廓

    //求最小包围矩形，斜的也可以哦
     RotatedRect rectPoint = minAreaRect(contours_all[0]);
    Point2f fourPoint2f[4];
    Point2f center;
    //将rectPoint变量中存储的坐标值放到 fourPoint的数组中
    rectPoint.points(fourPoint2f);
    for (int i = 0; i < 4; i++)
    {
        line(src_all, fourPoint2f[i % 4], fourPoint2f[(i + 1) % 4], Scalar(20, 21, 237), 3);
    }
    center.x = (fourPoint2f[0].x + fourPoint2f[1].x + fourPoint2f[2].x + fourPoint2f[3].x)/4 - 320;
    center.y = (fourPoint2f[0].y + fourPoint2f[1].y + fourPoint2f[2].y + fourPoint2f[3].y)/4 - 180;
    std::cout<<center.x<<" "<<center.y<<std::endl;
    v_msg.x = center.x;
    v_msg.y = center.y;

    imshow("erweima", src_all);
  }
  waitKey(1);
  return v_msg;
}

void image_callback(const sensor_msgs::ImageConstPtr &img)
{
    Mat resive_im = cv_bridge::toCvShare(img, "mono8")->image;
    //imshow("lalala", resive_im);
    Mat ims = imread("01.jpg", 1);
    src = resive_im;

    cam::vision_msg vision = vision_location();
    vision_pub.publish(vision);
}


void Threshold_Demo( int, void* )
{

    threshold( src, threshold_output, threshold_value, 255, THRESH_BINARY );

    imshow( window_name, threshold_output );
}
