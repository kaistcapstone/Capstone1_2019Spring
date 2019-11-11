//기본 코드에서 주어지는 것들이다. 수식을 사용하기 위해 math.h를 작성하였다.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

//ball의 색깔마다 정의
#define BLUE 3
#define RED 4
#define GREEN 5

//std와 cv관련 함수를 많이 사용하기 때문에 namespace를 정의해준 것이다.
using namespace std;
using namespace cv;

//기본 코드에서 주어지는 것들이다. red ball detect에 대한 trackbar 함수를 정의하는 것이다.
void on_low_h_thresh_trackbar_red(int, void *);
void on_high_h_thresh_trackbar_red(int, void *);
void on_low_h2_thresh_trackbar_red(int, void *);
void on_high_h2_thresh_trackbar_red(int, void *);
void on_low_s_thresh_trackbar_red(int, void *);
void on_high_s_thresh_trackbar_red(int, void *);
void on_low_v_thresh_trackbar_red(int, void *);
void on_high_v_thresh_trackbar_red(int, void *);

//red ball을 찾는데 hsv값을 트랙바를 통해 찾아낸 후 정의한 것이다.
int low_h2_r=179, high_h2_r=180;
int low_h_r=0, low_s_r=150, low_v_r=30;
int high_h_r=10, high_s_r=255, high_v_r=255;

//기본 코드에서 주어지는 것들이다. blue ball detect에 대한 trackbar 함수를 정의하는 것이다.
void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);

//blue ball을 찾는데 hsv값을 트랙바를 통해 찾아낸 후 정의한 것이다.
int low_h_b=100, low_s_b=150, low_v_b=0;
int high_h_b=150, high_s_b=255, high_v_b=255;

//blue ball2을 찾는데 hsv값을 트랙바를 통해 찾아낸 후 정의한 것이다.
int low_h_b2=80, low_s_b2=140, low_v_b2=0;
int high_h_b2=120, high_s_b2=255, high_v_b2=255;


//기본 코드에서 주어지는 것들이다. green ball detect에 대한 trackbar 함수를 정의하는 것이다.
void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);

//green ball을 찾는데 hsv값을 트랙바를 통해 찾아낸 후 정의한 것이다.
int low_h_g=50, low_s_g=70, low_v_g=40;
int high_h_g=90, high_s_g=255, high_v_g=255;

//green ball2을 찾는데 hsv값을 트랙바를 통해 찾아낸 후 정의한 것이다.
int low_h_g2=40, low_s_g2=70, low_v_g2=40;
int high_h_g2=90, high_s_g2=210, high_v_g2=255;


//기본 코드에서 주어지는 것들이다.
// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);
// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);
// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);
vector<float> pixel2point2(Point center, int radius);
// Declaration of trackbars function that set canny edge's parameters
void on_canny_edge_trackbar_red(int, void *);
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
void on_canny_edge_trackbar_blue(int, void *);03
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;
void on_canny_edge_trackbar_green(int, void *);
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;
// Initialization of variable for dimension of the target


float fball_diameter = 0.0737; // meter
// Initialization of variable for camera calibration paramters

//기본 코드에서 주어지는 것들이다. 카메라 켈리브레이션을 통해 구해진 숫자들을 아래에 적어 넣으면 된다.
Mat distCoeffs;
float intrinsic_data[9] = {644.250703, 0, 341.781725, 0, 644.31873, 214.069506, 0, 0, 1};
float distortion_data[5] = {0.022867, -0.043961, 0.002345, -0.009070, 0};

float intrinsic_data2[9] = {643.261791, 0, 313.614993, 0, 646.846154, 215.713748, 0, 0, 1};
float distortion_data2[5] = {0.031861, -0.138421, 0.00654, -0.001543, 0};
// Initialization of variable for text drawing
double fontScale = 2;
int thickness = 3;
String text;

//기본 코드에서 주어지는 것들이다.
int iMin_tracking_ball_size = 0;


//메인 함수에서 이용할 함수들을 위에 먼저 정의해 두는 것이다.
void remove_duplicate(vector<Point2f> &centers, vector<float> &radii);

void pub_ball_info(int ball_num, vector<vector<float> > ball_position, ros::Publisher markers, int color);
void pub_ball_info2(int ball_num, vector<vector<float> > ball_position, ros::Publisher markers, int color);


vector<vector<float>> position_info( vector<float> radius, vector<Point2f> center, Scalar color, int &ball_num, Mat &result);
vector<vector<float>> position_info2( vector<float> radius, vector<Point2f> center, Scalar color, int &ball_num, Mat &result);



int main(int argc, char **argv){
  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh);
  //각각의 색깔마다 퍼블리쉬하는 퍼블리셔를 정의하는 코드이다.
  ros::Publisher blueball_info = nh.advertise<core_msgs::ball_position>("/blueball_info",1);
  ros::Publisher redball_info = nh.advertise<core_msgs::ball_position>("/redball_info",1);
  ros::Publisher greenball_info = nh.advertise<core_msgs::ball_position>("/greenball_info",1);

  ros::Publisher blueball_info2 = nh.advertise<core_msgs::ball_position>("/blueball_info2",1);
  ros::Publisher greenball_info2 = nh.advertise<core_msgs::ball_position>("/greenball_info2",1);

  //연습 데모를 진행할 때 웹캠의 영상을 저장하기 위해 퍼블리셔를 만든 것이다.
  image_transport::Publisher pub1 = it.advertise("camera/image1", 1);
  image_transport::Publisher pub2 = it.advertise("camera/image2", 1);
  sensor_msgs::ImagePtr msg1;
  sensor_msgs::ImagePtr msg2;
  cv::Mat buffer1;
  cv::Mat buffer2;
  bool reduced = true; //boolean variable that decides whether you want to use reduced image or not-reduced image. If there is slow-down caused by big-sized data, then set it true.

  //이미지 프로세싱을 하는데 필요한 각종 Mat들을 정의해 놓은 것이다. 거의 기본 코드에 주어져있다.
  Mat frame,frame2, bgr_frame, bgr_frame2, hsv_frame, hsv_frame2,
  hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue,hsv_frame_blue2, hsv_frame_green,
  hsv_frame_red_blur, hsv_frame_blue_blur,hsv_frame_blue_blur2, hsv_frame_green_blur,
  hsv_frame_red_canny, hsv_frame_blue_canny,hsv_frame_blue_canny2, hsv_frame_green_canny, result,result2;
  Mat hsv_frame_green2, hsv_frame_green_blur2, hsv_frame_green_canny2;
  Mat calibrated_frame, calibrated_frame2;
  Mat intrinsic = Mat(3,3, CV_32FC1);
  Mat intrinsic2 = Mat(3,3, CV_32FC1);
  Mat distCoeffs;
  Mat distCoeffs2;
  intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  distCoeffs = Mat(1, 5, CV_32F, distortion_data);
  intrinsic2 = Mat(3, 3, CV_32F, intrinsic_data2);
  distCoeffs2 = Mat(1, 5, CV_32F, distortion_data2);

  vector<Vec4i> hierarchy_r;
  vector<Vec4i> hierarchy_b;
  vector<Vec4i> hierarchy_b2;
  vector<Vec4i> hierarchy_g;
  vector<Vec4i> hierarchy_g2;

  vector<vector<Point>> contours_r;
  vector<vector<Point>> contours_b;
  vector<vector<Point>> contours_b2;
  vector<vector<Point>> contours_g;
  vector<vector<Point>> contours_g2;

  //웹캠1의 영상은 cap1(0)으로, 웹캠2의 영상은 cap2(1)로 저장된다.
  VideoCapture cap1(0);
  VideoCapture cap2(1);
  //namedWindow("Video Capture", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Green", WINDOW_NORMAL);
  namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
  namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
  namedWindow("Canny Edge for Green Ball", WINDOW_NORMAL);
  namedWindow("Canny Edge for Green Ball2", WINDOW_NORMAL);
  namedWindow("blur_red",WINDOW_NORMAL);
  namedWindow("Result", WINDOW_NORMAL);
  namedWindow("Result2", WINDOW_NORMAL);
  namedWindow("Result2_flip",WINDOW_NORMAL);
  //moveWindow("Video Capture",50, 0);
  moveWindow("Object Detection_HSV_Red", 50,370);
  moveWindow("Object Detection_HSV_Blue",470,370);
  moveWindow("Canny Edge for Red Ball", 50,730);
  moveWindow("Canny Edge for Blue Ball", 470,730);
  moveWindow("Result", 470, 0);
  moveWindow("Result2", 470, 0);



  // Trackbars to set thresholds for HSV values : Red ball
  createTrackbar("Low H","Object Detection_HSV_Red", &low_h_r, 180,on_low_h_thresh_trackbar_red);
  createTrackbar("High H","Object Detection_HSV_Red", &high_h_r, 180,on_high_h_thresh_trackbar_red);
  createTrackbar("Low H2","Object Detection_HSV_Red", &low_h2_r, 180,on_low_h2_thresh_trackbar_red);
  createTrackbar("High H2","Object Detection_HSV_Red", &high_h2_r, 180,on_high_h2_thresh_trackbar_red);
  createTrackbar("Low S","Object Detection_HSV_Red", &low_s_r, 255,on_low_s_thresh_trackbar_red);
  createTrackbar("High S","Object Detection_HSV_Red", &high_s_r, 255,on_high_s_thresh_trackbar_red);
  createTrackbar("Low V","Object Detection_HSV_Red", &low_v_r, 255,on_low_v_thresh_trackbar_red);
  createTrackbar("High V","Object Detection_HSV_Red", &high_v_r, 255,on_high_v_thresh_trackbar_red);
  // Trackbars to set thresholds for HSV values : Blue ball
  createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180,on_low_h_thresh_trackbar_blue);
  createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180,on_high_h_thresh_trackbar_blue);
  createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255,on_low_s_thresh_trackbar_blue);
  createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255,on_high_s_thresh_trackbar_blue);
  createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255,on_low_v_thresh_trackbar_blue);
  createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255,on_high_v_thresh_trackbar_blue);
  // Trackbars to set thresholds for HSV values : Green ball
  createTrackbar("Low H","Object Detection_HSV_Green", &low_h_g, 180,on_low_h_thresh_trackbar_green);
  createTrackbar("High H","Object Detection_HSV_Green", &high_h_g, 180,on_high_h_thresh_trackbar_green);
  createTrackbar("Low S","Object Detection_HSV_Green", &low_s_g, 255,on_low_s_thresh_trackbar_green);
  createTrackbar("High S","Object Detection_HSV_Green", &high_s_g, 255,on_high_s_thresh_trackbar_green);
  createTrackbar("Low V","Object Detection_HSV_Green", &low_v_g, 255,on_low_v_thresh_trackbar_green);
  createTrackbar("High V","Object Detection_HSV_Green", &high_v_g, 255,on_high_v_thresh_trackbar_green);
  // Trackbar to set parameter for Canny Edge
  createTrackbar("Min Threshold:","Canny Edge for Red Ball", &lowThreshold_r, 100, on_canny_edge_trackbar_red);
  createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);
  createTrackbar("Min Threshold:","Canny Edge for Green Ball", &lowThreshold_g, 100, on_canny_edge_trackbar_green);

  ros::Rate rate(20);

  while((char)waitKey(1)!='q'){
    //웹캠의 이미지를 frame에 넣는다.
    cap1>>frame;
    cap2>>frame2;

    //이미지가 없다면 멈춘다.
    if(frame.empty())
    break;
    if(frame2.empty())
    break;


    //카메라 켈리브레이션을 통해 얻어낸 메트릭스를 이용해서 영상을 켈리브레이션하고 메디안블러를 한 후 RGB색을 HSV색으로 변환한다.
    undistort(frame, calibrated_frame, intrinsic, distCoeffs);
    result = calibrated_frame.clone();
    medianBlur(calibrated_frame, calibrated_frame, 3);
    cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

    undistort(frame2, calibrated_frame2, intrinsic2, distCoeffs2);
    result2 = calibrated_frame2.clone();
    medianBlur(calibrated_frame2, calibrated_frame2, 3);
    cvtColor(calibrated_frame2, hsv_frame2, cv::COLOR_BGR2HSV);

    // Detect the object based on RGB and HSV Range Values
    //프로세싱하는 순서는 다음과 같다.
    //1.먼저 각각의 공의 색에 따라 범위에 맞는 hsv값을 찾는다. (inRange함수를 통해)
    //2.morphOps함수를 통해 이미지 프로세싱하기 쉽게 만든다.
    //3.가우시안블러를 통해 이미지 프로세싱하기 쉽게 만든다.
    //4.Canny함수를 통해 엣지를 찾아내고
    //5.findContours함수를 통해 엣지의 정보 conturs_에 저장한다.
    //6.각 색깔별로 모두 반복한다.
    inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
    inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
    addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);
    morphOps(hsv_frame_red);
    GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
    Canny(hsv_frame_red_blur,hsv_frame_red_canny,lowThreshold_r,lowThreshold_r*ratio_r, kernel_size_r);
    findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));


    inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
    morphOps(hsv_frame_blue);06
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
    Canny(hsv_frame_blue_blur,hsv_frame_blue_canny,lowThreshold_b,lowThreshold_b*ratio_b, kernel_size_b);
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));


    inRange(hsv_frame2,Scalar(low_h_b2,low_s_b2,low_v_b2),Scalar(high_h_b2,high_s_b2,high_v_b2),hsv_frame_blue2);
    morphOps(hsv_frame_blue2);
    GaussianBlur(hsv_frame_blue2, hsv_frame_blue_blur2, cv::Size(9, 9), 2, 2);
    Canny(hsv_frame_blue_blur2,hsv_frame_blue_canny2,lowThreshold_b,lowThreshold_b*ratio_b, kernel_size_b);
    findContours(hsv_frame_blue_canny2, contours_b2, hierarchy_b2, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));

    inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);
    morphOps(hsv_frame_green);
    GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);
    Canny(hsv_frame_green_blur,hsv_frame_green_canny,lowThreshold_g,lowThreshold_g*ratio_g, kernel_size_g);
    findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));

    inRange(hsv_frame2,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green2);
    morphOps(hsv_frame_green2);
    GaussianBlur(hsv_frame_green2, hsv_frame_green_blur2, cv::Size(9, 9), 2, 2);
    Canny(hsv_frame_green_blur2,hsv_frame_green_canny2,lowThreshold_g,lowThreshold_g*ratio_g, kernel_size_g);
    findContours(hsv_frame_green_canny2, contours_g2, hierarchy_g2, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));

    //컨투어의 집합을 저장하는 벡터를 정의한 것이다.
    vector<vector<Point>> contours_r_poly(contours_r.size());
    vector<vector<Point>> contours_b_poly(contours_b.size());
    vector<vector<Point>> contours_b_poly2(contours_b2.size());
    vector<vector<Point>> contours_g_poly(contours_g.size());
    vector<vector<Point>> contours_g_poly2(contours_g2.size());

    //컨투어를 통해 원의 중심과 반지름을 저장할 벡터를 정의한 것이다.
    vector<Point2f>center_r(contours_r.size());
    vector<Point2f>center_b(contours_b.size());
    vector<Point2f>center_b2(contours_b2.size());
    vector<Point2f>center_g(contours_g.size());
    vector<Point2f>center_g2(contours_g2.size());


    vector<float>radius_r(contours_r.size());
    vector<float>radius_b(contours_b.size());
    vector<float>radius_b2(contours_b2.size());
    vector<float>radius_g(contours_g.size());
    vector<float>radius_g2(contours_g2.size());

    //컨투어를 approxPolyDP함수를 이용해 다항식으로 변환시킨 후 minEnclosingCircle함수를 통해 그 컨투어의 외접원에 대한 중심과 반지름을 얻어내는 코드이다.
    //실제 실행시켜보면 반지름이 너무 크게잡혀서 이를 거리별로 값을 구한 후 실제 값과 1차추세선을 이용해 켈리브레이션한 것이 마지막에 radius_r[i] = ~~~이다.
    for(size_t i = 0; i <contours_r.size(); i++){
    approxPolyDP(contours_r[i], contours_r_poly[i], 3, true);
    minEnclosingCircle(contours_r_poly[i], center_r[i], radius_r[i]);
    radius_r[i] = 0.7536*radius_r[i]+0.6771;
    }


    for( size_t i = 0; i < contours_b.size(); i++ ){
        approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
        minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
        radius_b[i] = 0.8538*radius_b[i]-4.0814;
    }

    for( size_t i = 0; i < contours_b2.size(); i++ ){
        approxPolyDP( contours_b2[i], contours_b_poly2[i], 3, true );
        minEnclosingCircle( contours_b_poly2[i], center_b2[i], radius_b2[i] );
        radius_b2[i] = 0.8538*radius_b2[i]-4.0814;
    }

    for(size_t i = 0; i < contours_g.size(); i++){
      approxPolyDP(contours_g[i], contours_g_poly[i], 3, true);
      minEnclosingCircle(contours_g_poly[i], center_g[i], radius_g[i]);
      radius_g[i] = 0.7224*radius_g[i]+1.8049;
    }

    for(size_t i = 0; i < contours_g2.size(); i++){
      approxPolyDP(contours_g2[i], contours_g_poly2[i], 3, true);
      minEnclosingCircle(contours_g_poly2[i], center_g2[i], radius_g2[i]);
      radius_g2[i] = 0.7224*radius_g2[i]+0.421;
    }

    //한 원 안에 다른 작은 원이 있을 경우(에러로 인해) 이를 없애주는 함수이다.
    remove_duplicate(center_r, radius_r);
    remove_duplicate(center_b, radius_b);
    remove_duplicate(center_b2, radius_b2);
    remove_duplicate(center_g, radius_g);
    remove_duplicate(center_g2, radius_g2);

    //각 색깔마다 반지름과 중심을 넣으면 xyz값을 얻어내는 position_info함수를 통해 위치를 저장하고 이를 pub_ball_info함수를 통해 msg를 보내는 코드이다.
    int ball_num = 0; Scalar color_r, color_b, color_g ; color_g = Scalar(0,255,0); color_r  = Scalar(0,0,255); color_b = Scalar(255,0,0);

    vector<vector<float>> ball_position_r = position_info(radius_r, center_r, color_r, ball_num, result);
    pub_ball_info(ball_num, ball_position_r, redball_info, RED);

    vector<vector<float>> ball_position_b = position_info(radius_b, center_b, color_b, ball_num, result);
    pub_ball_info(ball_num, ball_position_b, blueball_info, BLUE);

    vector<vector<float>> ball_position_b2 = position_info2(radius_b2, center_b2, color_b, ball_num, result2);
    pub_ball_info2(ball_num, ball_position_b2, blueball_info2, BLUE);

    vector<vector<float>> ball_position_g = position_info(radius_g, center_g, color_g, ball_num, result);
    pub_ball_info(ball_num, ball_position_g, greenball_info, GREEN);

    vector<vector<float>> ball_position_g2 = position_info(radius_g2, center_g2, color_g, ball_num, result2);
    pub_ball_info(ball_num, ball_position_g2, greenball_info2, GREEN);


    // // Show the frames
    // imshow("Video Capture", calibrated_frame);
    // imshow("Object Detection_HSV_Red", hsv_frame_red);
    // imshow("Object Detection_HSV_Blue", hsv_frame_blue);
    // imshow("Object Detection_HSV_Blue", hsv_frame_blue2);
    // imshow("Object Detection_HSV_Green", hsv_frame_green);
    // imshow("Object Detection_HSV_Green", hsv_frame_green2);
    // imshow("Canny Edge for Red Ball", hsv_frame_red_canny);
    // imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
    // imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny2);
    imshow("Canny Edge for Green Ball", hsv_frame_green_canny);
    imshow("Canny Edge for Green Ball2", hsv_frame_green_canny2);
    // imshow("blur_red",hsv_frame_red_blur);
    imshow("Result", result);
    // imshow("Result2", result2);
    Mat result2_flip;
    flip(result2,result2_flip,-1);
    imshow("Result2_flip", result2_flip);

    //위에서 설명했듯이 연습데모에서 웹캠의 영상을 보기 위해 메세지를 보내 이를 rosbag으로 저장해 확인하기 위해 메세지를 보내는 코드이다.
    if(reduced==true){
      cv::resize(result, buffer1, cv::Size(320, 240)); //reduced the size of the image
      cv::resize(result2, buffer2, cv::Size(320,240));
        }
        else{
      buffer1 = result;
      buffer2 = result2;
        }
    msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer1).toImageMsg(); //converting a image 'buffer' to ros message
    msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer2).toImageMsg(); //converting a image 'buffer' to ros message
    pub1.publish(msg1);
    pub2.publish(msg2);

    rate.sleep();

    }

    cout<<"haha"<<endl;
    return 0;
  }

  //공의 반지름과 중심을 넣으면 xyz값을 가지고 있는 벡터를 반환하는 코드이다. pixel2point함수를 통해 중심과 반지름을 넣으면 xyz값을 가진 벡터를 반환한다.
  //이외의 코드들은 확인을 위해 영상에 공의 중심과 위치를 나타내는 코드들이다.
  vector<vector<float>> position_info( vector<float> radius, vector<Point2f> center, Scalar color, int &ball_num, Mat &result){
      ball_num = 0;
      vector<vector<float>> ball_positions;
      for( size_t i = 0; i< center.size(); i++ ){
          if (radius[i] > iMin_tracking_ball_size){
              vector<float> ball_position;
              ball_position = pixel2point(center[i], radius[i]);
              ball_positions.push_back(ball_position);

              //여기서부터는 공의 위치를 영상에 나타내는 코드로 중요하지 않다.
              circle( result, center[i], (int)radius[i], color, 2, 8, 0 );
              float isz = ball_position[2];
              string sz = floatToString(isz);
              text = "DISTANCE:" + sz;
              for( size_t i = 0; i< ball_positions.size(); i++ ){ //거리가 너무 먼 공에 대해서는 표시안하는 코드이다.
                if(ball_positions[i][0]>3 || ball_positions[i][1]<0 || ball_positions[i][2]>6){
                }
                else{
                  putText(result, text, center[i],2,1,Scalar(255,255,255),2);
                }
              }
              ball_num++;
          }
      }
      return ball_positions;
  }

  //웹캠2에서는 pixel2point2함수를 사용하기떄문에 비슷한 함수를 2개 만들어 준 것이다.
  vector<vector<float>> position_info2( vector<float> radius, vector<Point2f> center, Scalar color, int &ball_num, Mat &result){
      ball_num = 0;
      vector<vector<float>> ball_positions;
      for( size_t i = 0; i< center.size(); i++ ){
          if (radius[i] > iMin_tracking_ball_size){
              vector<float> ball_position;
              ball_position = pixel2point2(center[i], radius[i]);
              ball_positions.push_back(ball_position);
              circle( result, center[i], (int)radius[i], color, 2, 8, 0 );
              float isz = ball_position[2];
              string sz = floatToString(isz);
              text = "DISTANCE:" + sz;
              for( size_t i = 0; i< ball_positions.size(); i++ ){
                if(ball_positions[i][0]>0.5 || ball_positions[i][1]<0 || ball_positions[i][2]>1){
                }
                else{
                  putText(result, text, center[i],2,1,Scalar(255,255,255),2);
                }
              }
              ball_num++;
          }
      }

      return ball_positions;
  }





  //공의 중심과 반지름을 넣으면 xyz로 반환해 주는 코드이다. 기본 코드에 이미 있다.
  //웹캠1의 경우 아래로 각도가 내려가있기 때문에 각도에 따라 위치를 수정해주는 코드가 들어가있다.
    vector<float> pixel2point(Point center, int radius){
      vector<float> position;
      float x, y, u, v, Xc, Yc, Zc, theta, yy, zz;
      x = center.x;//.x;// .at(0);
      y = center.y;//.y;//
      u = (x-intrinsic_data[2])/intrinsic_data[0];
      v = (y-intrinsic_data[5])/intrinsic_data[4];
      Zc = (644.250703*fball_diameter)/(2*(float)radius);
      Xc = u*Zc ;
      Yc = v*Zc ;
      //이 아래부터 각도수정해주는 코드이다.
      theta = 19.65/180*3.141592;
      if(Zc<1.041){
        yy = Zc*sin(atan(abs(Yc)/Zc)+theta);
        zz = Zc*cos(atan(abs(Yc)/Zc)+theta);
        position.push_back(Xc);
        position.push_back(yy);
        position.push_back(zz);
      }
      else{
        yy = sqrt(Zc*Zc+Yc*Yc)*sin(theta - atan(abs(Yc)/Zc));
        zz = sqrt(Zc*Zc+Yc*Yc)*cos(theta - atan(abs(Yc)/Zc));
        position.push_back(Xc);
        position.push_back(yy);
        position.push_back(zz);
      }
      return position;
    }

    //웹캠2의 경우 각도를 수정해줄 필요가 없이 정면을 바라보고있기 떄문에 기본코드를 사용한 것이다.
    vector<float> pixel2point2(Point center, int radius){
      vector<float> position;
      float x, y, u, v, Xc, Yc, Zc, theta, yy, zz;
      x = center.x;//.x;// .at(0);
      y = center.y;//.y;//
      u = (x-intrinsic_data2[2])/intrinsic_data2[0];
      v = (y-intrinsic_data2[5])/intrinsic_data2[4];
      Zc = (643.261791*fball_diameter)/(2*(float)radius);
      Xc = u*Zc ;
      Yc = v*Zc ;
      position.push_back(Xc);
      position.push_back(Yc);
      position.push_back(Zc);
      return position;
    }


    //원 안에 또다른 작은 원이 에러로 인해 들어있을 경우 이를 지워주는 코드이다.
    //i번째 원과 j번쨰 원을 비교해서 둘 중 최대 반지름보다 중심사이 거리가 가까울 경우 원안에 원이 있는 것이므로 지워준다.
    void remove_duplicate(vector<Point2f> &centers, vector<float> &radii){
        size_t i = 0;
        while (i < radii.size()){
            bool cheak = true;
            for (size_t j = 0; j < radii.size() ; j++){
                if (i!=j&&(norm(centers[i] - centers[j]) < max(radii[i],radii[j]))){
                  if(radii[i]<radii[j]){
                      centers.erase(centers.begin()+i);
                      radii.erase(radii.begin()+i);
                  }
                  else{
                    centers.erase(centers.begin()+j);
                    radii.erase(radii.begin()+j);
                  }
                  cheak = false;

                  break;
                }
            }
            if(cheak){
                i++;
            }
        }
    }



    //core msgs 형식의 ball_list를 정의해 x,y,z정보를 담아 퍼블리쉬 하는 코드이다.
    void pub_ball_info(int ball_num, vector<vector<float>> ball_position, ros::Publisher markers, int color){
        core_msgs::ball_position ball_list;  //declare marker

        //웹캠1의 경우 x가 3m보다 크거나, y가 0보다 작거나, z가 0.01m보다 작거나 6m보다 클 경우가 없기 때문에 이 경우 생기는 에러를 보내지 않는코드이다.
        for( size_t i = 0; i< ball_position.size(); i++ ){
          if(ball_position[i][0]>3 || ball_position[i][1]<0 || ball_position[i][2]>6 || ball_position[i][2]<0.01){
            ball_position.erase(ball_position.begin()+i);
            ball_num--;
          }
        }
        if(ball_num){
            for(int k=0;k<ball_position.size();k++){
                ball_list.img_x.push_back(ball_position[k][0]);
                ball_list.img_y.push_back(ball_position[k][1]);
                ball_list.img_z.push_back(ball_position[k][2]);
                ball_list.size = ball_num;
            }
        }
        markers.publish(ball_list);
    }



    //위와 같지만 에러를 보내지 않는 xyz범위가 다르기 때문에 하나 더 함수를 만들어 준 것이다.
    void pub_ball_info2(int ball_num, vector<vector<float>> ball_position, ros::Publisher markers, int color){
        core_msgs::ball_position ball_list;  //declare marker
        //x가 0.5m보다 크거나 y가 0보다 작거나 z가 0.2m보다 작거나 3m보다 큰 공에 대해서는 메세지를 보내지 않도록 벡터에서 지우는 코드이다.
        for( size_t i = 0; i< ball_position.size(); i++ ){
          if(ball_position[i][0]>0.5 || ball_position[i][1]<0 || ball_position[i][2]>3 || ball_position[i][2]< 0.2){
            ball_position.erase(ball_position.begin()+i);
            ball_num--;
          }
        }
        if(ball_num){
            for(int k=0;k<ball_position.size();k++){
                if(ball_position[k][2]<3){
                ball_list.img_x.push_back(ball_position[k][0]);
                ball_list.img_y.push_back(ball_position[k][1]);
                ball_list.img_z.push_back(ball_position[k][2]);
                cout<<ball_position[k][2]<<endl;
                ball_list.size = ball_num;
              }
              else{
              }
            }
        }
        markers.publish(ball_list);
    }





//아래는 모두 기본 코드에서 주어지는 것들이다.
  string intToString(int n){
    stringstream s;
    s << n;
    return s.str();
  }

  string floatToString(float f){
    ostringstream buffer;
    buffer << f;
    return buffer.str();
  }

  void morphOps(Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
  }

  // Trackbar for image threshodling in HSV colorspace : Red
  void on_low_h_thresh_trackbar_red(int, void *){
    low_h_r = min(high_h_r-1, low_h_r);
    setTrackbarPos("Low H","Object Detection_HSV_Red", low_h_r);
  }
  void on_high_h_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
  }
  void on_low_h2_thresh_trackbar_red(int, void *){
    low_h2_r = min(high_h2_r-1, low_h2_r);
    setTrackbarPos("Low H2","Object Detection_HSV_Red", low_h2_r);
  }
  void on_high_h2_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
  }
  void on_low_s_thresh_trackbar_red(int, void *){
    low_s_r = min(high_s_r-1, low_s_r);
    setTrackbarPos("Low S","Object Detection_HSV_Red", low_s_r);
  }
  void on_high_s_thresh_trackbar_red(int, void *){
    high_s_r = max(high_s_r, low_s_r+1);
    setTrackbarPos("High S", "Object Detection_HSV_Red", high_s_r);
  }
  void on_low_v_thresh_trackbar_red(int, void *){
    low_v_r= min(high_v_r-1, low_v_r);
    setTrackbarPos("Low V","Object Detection_HSV_Red", low_v_r);
  }
  void on_high_v_thresh_trackbar_red(int, void *){
    high_v_r = max(high_v_r, low_v_r+1);
    setTrackbarPos("High V", "Object Detection_HSV_Red", high_v_r);
  }
  // Trackbar for image threshodling in HSV colorspace : Blue
  void on_low_h_thresh_trackbar_blue(int, void *){
    low_h_b = min(high_h_b-1, low_h_b);
    setTrackbarPos("Low H","Object Detection_HSV_Blue", low_h_b);
  }
  void on_high_h_thresh_trackbar_blue(int, void *){
    high_h_b = max(high_h_b, low_h_b+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_b);
  }
  void on_low_s_thresh_trackbar_blue(int, void *){
    low_s_b = min(high_s_b-1, low_s_b);
    setTrackbarPos("Low S","Object Detection_HSV_Blue", low_s_b);
  }
  void on_high_s_thresh_trackbar_blue(int, void *){
    high_s_b = max(high_s_b, low_s_b+1);
    setTrackbarPos("High S", "Object Detection_HSV_Blue", high_s_b);
  }
  void on_low_v_thresh_trackbar_blue(int, void *){
    low_v_b= min(high_v_b-1, low_v_b);
    setTrackbarPos("Low V","Object Detection_HSV_Blue", low_v_b);
  }
  void on_high_v_thresh_trackbar_blue(int, void *){
    high_v_b = max(high_v_b, low_v_b+1);
    setTrackbarPos("High V", "Object Detection_HSV_Blue", high_v_b);
  }
  // Trackbar for image threshodling in HSV colorspace : Green
  void on_low_h_thresh_trackbar_green(int, void *){
    low_h_g = min(high_h_g-1, low_h_g);
    setTrackbarPos("Low H","Object Detection_HSV_Green", low_h_g);
  }
  void on_high_h_thresh_trackbar_green(int, void *){
    high_h_g = max(high_h_g, low_h_g+1);
    setTrackbarPos("High H", "Object Detection_HSV_Green", high_h_g);
  }
  void on_low_s_thresh_trackbar_green(int, void *){
    low_s_g = min(high_s_g-1, low_s_g);
    setTrackbarPos("Low S","Object Detection_HSV_Green", low_s_g);
  }
  void on_high_s_thresh_trackbar_green(int, void *){
    high_s_g = max(high_s_g, low_s_g+1);
    setTrackbarPos("High S", "Object Detection_HSV_Green", high_s_g);
  }
  void on_low_v_thresh_trackbar_green(int, void *){
    low_v_g = min(high_v_g-1, low_v_g);
    setTrackbarPos("Low V","Object Detection_HSV_Green", low_v_g);
  }
  void on_high_v_thresh_trackbar_green(int, void *){
    high_v_g = max(high_v_g, low_v_g+1);
    setTrackbarPos("High V", "Object Detection_HSV_Green", high_v_g);
  }
  // Trackbar for Canny edge algorithm
  void on_canny_edge_trackbar_red(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Red Ball",  lowThreshold_r);
  }
  void on_canny_edge_trackbar_blue(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
  }

  void on_canny_edge_trackbar_green(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
  }
