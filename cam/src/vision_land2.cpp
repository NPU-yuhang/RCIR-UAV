#include <cam/vision_land2.h>

const double PI = 3.141592653;
int i;
int threshold_value = 170;
const char* T_value = "threshold_value";
const char* window_name = "yuchulihou";
double max_val = 255; //阈值化后的最大值
IplImage* bw_image;
IplImage* gray_image;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_land2");
    ros::NodeHandle nh;
    vision_pub = nh.advertise<cam::vision_msg>("vision2/msg", 1);
    ros::start();
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera_BGR", 1);
    img_sub.registerCallback(image_callback);
    ros::spin();
    ros::shutdown();
}

cam::vision_msg vision_location()
{
    cam::vision_msg v_msg;
    //string video_dir = "video.MOV";
    //CvCapture *capture = cvCreateCameraCapture(1);
      //src = cvQueryFrame(capture);
    //src=cvLoadImage(argv[1]);//读入图像
//     tmp=cvCloneImage(src);//复制图像到临时图像上
//     org=cvCloneImage(src);//保存原始图像
    cvNamedWindow("src",CV_WINDOW_AUTOSIZE);//新建窗口
//    cvMoveWindow("src",0,0);
//     cvSetMouseCallback( "src", on_mouse, 0 );//注册鼠标响应回调函数
/*    cvShowImage("src",src);*///显示图像
    //-----------------------------------------灰度图像-----------------------------------------//
  //图像灰度化
    gray_image = cvCreateImage(cvGetSize(src),8,1);
    cvCvtColor(src,gray_image,CV_BGR2GRAY);
    //进行高斯处理，处理的是指针img指向的内存，将处理后的数据交给out指针指向的内存，对每个像素周围3x3的区域进行高斯平滑处理（其实输入输出图像可以是相同的）
    cvSmooth(gray_image,gray_image,CV_GAUSSIAN,9,9);
    //-----------------------------------------二值图像-----------------------------------------//
    //图像二值化
    bw_image = cvCreateImage(cvGetSize(gray_image),IPL_DEPTH_8U, 1);
    cvNamedWindow(window_name);
    cvCreateTrackbar( T_value, window_name, &threshold_value, max_val, Threshold_Demo );
    Threshold_Demo(0);
    //-----------------------------------------腐蚀图像-----------------------------------------//
    IplConvKernel *elem = cvCreateStructuringElementEx(2, 2, 0, 0,CV_SHAPE_RECT,NULL);
    IplImage *dst=cvCreateImage(cvGetSize(bw_image),IPL_DEPTH_8U, 1);
    cvErode(bw_image,dst,elem);
    //----------------------------------------------------------------------------------------//

    IplImage *canny_image = cvCreateImage(cvGetSize(dst), 8, 1);
    cvCanny(dst,canny_image,3,9,3);

#if 1
    /************************************************Circle identification********************************************/
    CvMemStorage* storagecircle = cvCreateMemStorage(0);
    CvMemStorage *storage = cvCreateMemStorage(0);
    CvSeq *first_contour = NULL;

    double dp=10;
    double min_dist=1000;
    int min_radius=1;
    int max_radius=100;
    //only greyimage is needed. cvHoughCircles would call cvSobel() internally.
    CvSeq* circles = cvHoughCircles(canny_image, storagecircle, CV_HOUGH_GRADIENT, dp, min_dist, min_radius, max_radius);
    cvFindContours(dst, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    int circlecnt=0;
    for( i = 0; i < circles->total; i++ )
    {
      float* p = (float*)cvGetSeqElem( circles, i );
      int RectCnt=0;
      for(; first_contour != 0; first_contour = first_contour->h_next)
      {
        CvRect rect = cvBoundingRect(first_contour,0);
        if(((rect.x + rect.width/2) - cvRound(p[0]))*((rect.x + rect.width/2) - cvRound(p[0])) + ((rect.y + rect.height/2) - cvRound(p[1]))*((rect.y + rect.height/2) - cvRound(p[1])) < cvRound(p[2])*cvRound(p[2]))
        {
          RectCnt++;
        }
      }
      if(RectCnt>=4){
        cvCircle( src, cvPoint(cvRound(p[0]),cvRound(p[1])),10, CV_RGB(255,0,0), -1, 4, 0 );
        circlecnt++;
        v_msg.y = p[0] - col/2;
        v_msg.x = -(p[1] - row/2);
        //std::cout<<cvRound(p[2])<<std::endl;
      }

    }
    std::cout<<"x: "<<v_msg.x<<"---y: "<<v_msg.y<<std::endl;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8);
    char textp[20];
    sprintf(textp,"target num %d",circlecnt);
    cvPutText(src, textp, cvPoint(50, 50), &font, CV_RGB(255,0,0));
    cvShowImage("src",src);
    #endif
    cvReleaseImage(&gray_image);//释放图像
    cvReleaseImage(&bw_image);//释放图像
    cvReleaseImage(&dst);//释放图像
    cvReleaseStructuringElement(&elem);
    cvReleaseImage(&canny_image);
    cvReleaseMemStorage(&storagecircle);
    cvReleaseMemStorage(&storage);

    cvWaitKey(1);//等待按键按下
    //cvDestroyAllWindows();//销毁所有窗口
    cvReleaseImage(&src);//释放图像
    return v_msg;
}

void image_callback(const sensor_msgs::ImageConstPtr &img)
{
    Mat resive_im = cv_bridge::toCvShare(img, "bgr8")->image;
    row = resive_im.rows;
    col = resive_im.cols;
    //imshow("lalala", resive_im);
    src_mat = resive_im;
    IplImage src_1 = IplImage(src_mat);
    src_ptr = &src_1;
    src = cvCloneImage(src_ptr);
    cam::vision_msg vision = vision_location();
    vision_pub.publish(vision);
}

void Threshold_Demo(int)
{
  cvThreshold(gray_image, bw_image, threshold_value, max_val , CV_THRESH_BINARY); //调用OTSU算法的参数设置---CV_THRESH_OTSU
  cvShowImage(window_name,bw_image);
}
