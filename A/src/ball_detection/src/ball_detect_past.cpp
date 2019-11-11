// 2019.05.17. 21:30 by HS

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>

#define PI 3.14159265

using namespace std;
using namespace cv;
// demo place
// int low_h2_r=169, high_h2_r=180;
// int low_h_r=0, low_s_r=90, low_v_r=93;
// int high_h_r=8, high_s_r=255, high_v_r=255;
// int low_h_b=100, low_s_b=126, low_v_b=60;
// int high_h_b=121, high_s_b=255, high_v_b=255;
// int low_h_g=60, low_s_g=80, low_v_g=50;
// int high_h_g=95, high_s_g=255, high_v_g=255;

// capstone room
int low_h2_r=169, high_h2_r=180;
int low_h_r=0, low_s_r=90, low_v_r=112;
int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h_b=100, low_s_b=126, low_v_b=60;
int high_h_b=121, high_s_b=255, high_v_b=255;
int low_h_g=55, low_s_g=80, low_v_g=40;
int high_h_g=95, high_s_g=255, high_v_g=255;

int low_r_r=87, low_g_r=0, low_b_r=0;
int high_r_r=209, high_g_r=60, high_b_r=32;
int low_r_b=0, low_g_b=0, low_b_b=89;
int high_r_b=63, high_g_b=136, high_b_b=207;

int high_hough_b = 200;
int low_hough_b=35;

// Declaration of trackbars function that set canny edge's parameters
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;

int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

int highthreshold=200;
int lowthreshold = 100;

// Initialization of variable for camera calibration paramters
float intrinsic_data[9] = {1390.429077, 0, 991.590027, 0, 1390.645996, 544.835266, 0, 0, 1};
float distortion_data[5] = {0, 0, 0, 0, 0};

// Initialization of variable for text drawing
String text;
int iMin_tracking_ball_size = 10;
float fball_radius = 0.072 ; // Initialization of variable for dimension of the target (meter)
float theta = 18;
int radcon=0;

std::string color_type="HSV";
std::string circle_type="Poly";
Mat buffer;
Mat depth_mat_glo = Mat::zeros(1280, 720, CV_32F);
Mat result;
ros::Publisher pub;
ros::Publisher pub_markers;

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh) {
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(5,5));
  erode(thresh,thresh,erodeElement);
  erode(thresh,thresh,erodeElement);
  dilate(thresh,thresh,dilateElement);
  dilate(thresh,thresh,dilateElement);
}

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius) {
  vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = center.x;//.x;// .at(0);
  y = center.y;//.y;//
  u = (x-intrinsic_data[2])/intrinsic_data[0];
  v = (y-intrinsic_data[5])/intrinsic_data[4];
  Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius) ;
  Xc = u*Zc ;
  Yc = v*Zc ;
  Xc = roundf(Xc * 1000) / 1000;
  Yc = roundf(Yc * 1000) / 1000;
  Zc = roundf(Zc * 1000) / 1000;
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc);
  return position;
}

string intToString(int n) {
  stringstream s;
  s << n;
  return s.str();
}

string floatToString(float f) {
  ostringstream buffer;
  buffer << f;
  return buffer.str();
}

void locator(int event, int x, int y, int flag, void *userdata) {
  if (event == EVENT_LBUTTONDOWN) cout<<"Left click has been made, Position: ("<<x<<","<<y<<")"<<endl;
}


void ball_detect() {
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);
  Mat frame;
  if(buffer.size().width==320) cv::resize(buffer, frame, cv::Size(640, 480)); //if the size of the image is 320x240, then resized it to 640x480
  else frame = buffer;

  Mat calibrated_frame;
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);
  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;
  Mat hsv_frame_green;

  result=calibrated_frame.clone();
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_g,high_v_g), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);
  morphOps(hsv_frame_red);
  morphOps(hsv_frame_blue);
  morphOps(hsv_frame_green);

  Mat frame_red_blur;
  Mat frame_blue_blur;
  Mat frame_green_blur;
  Mat frame_red_canny;
  Mat frame_blue_canny;
  Mat frame_green_canny;

  GaussianBlur(hsv_frame_red, frame_red_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_blue, frame_blue_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_green, frame_green_blur, cv::Size(9, 9), 2, 2);
  Canny(frame_red_blur, frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
  Canny(frame_blue_blur, frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
  Canny(frame_green_blur, frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);


  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<vector<Point> > contours_r_poly( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );
  for( size_t i = 0; i < contours_r.size(); i++ ) {
    approxPolyDP( contours_r[i], contours_r_poly[i], 1, true );
    minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
  }

  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<Point2f> center_b( contours_b.size() );
  vector<vector<Point> > contours_b_poly( contours_b.size() );
  vector<float> radius_b( contours_b.size() );
  for( size_t i = 0; i < contours_b.size(); i++ ) {
    approxPolyDP( contours_b[i], contours_b_poly[i], 1, true );
    minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
  }

  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_g;
  findContours(frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<vector<Point> > contours_g_poly( contours_g.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_g( contours_g.size() );
  for( size_t i = 0; i < contours_g.size(); i++ ) {
    approxPolyDP( contours_g[i], contours_g_poly[i], 1, true );
    minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
  }

  int real_ball_r = 0;
  int* real_ball_radius_pix_r = new int[contours_r.size()];
  int* real_ball_pix_r_x=new int[contours_r.size()];
  int* real_ball_pix_r_y=new int[contours_r.size()];
  float** carpos_r = new float*[contours_r.size()];
  for(int j = 0; j < contours_r.size(); ++j) carpos_r[j] = new float[3];
  for( size_t i = 0; i< contours_r.size(); i++ ) {
    bool diff_ball_r=true;
    if (radius_r[i] > iMin_tracking_ball_size) {
      int pix_x_r=center_r[i].x;
      int pix_y_r=center_r[i].y;
      int radius_pix_r=((int)radius_r[i]-radcon);
      for (int j=0 ; j<real_ball_r ; j++) {
        int real_pix_x_r=real_ball_pix_r_x[j];
        int real_pix_y_r=real_ball_pix_r_y[j];
        int radius_sum_r=real_ball_radius_pix_r[j]+radius_pix_r;
        int ball_dist_pix_r=pow(pix_x_r-real_pix_x_r,2)+pow(pix_y_r-real_pix_y_r,2);
        if(ball_dist_pix_r < pow(radius_sum_r,2)) {
          if (radius_pix_r>real_ball_radius_pix_r[j]) carpos_r[j][2]=-1;
          else diff_ball_r=false;
        }
      }
      if (diff_ball_r) {
        Scalar color = Scalar( 0, 0, 255);
        vector<float> ball_position_r;
        radius_r[i]=radius_r[i]-radcon;
        ball_position_r = pixel2point(center_r[i], radius_r[i]);
        float isx = ball_position_r[0];
        float isy = ball_position_r[1];
        float isz = ball_position_r[2];
        carpos_r[real_ball_r][0] = isx;
        carpos_r[real_ball_r][1] = isy;
        carpos_r[real_ball_r][2] = isz;
        real_ball_radius_pix_r[real_ball_r]=radius_r[i];
        real_ball_pix_r_x[real_ball_r]=center_r[i].x;
        real_ball_pix_r_y[real_ball_r]=center_r[i].y;
        real_ball_r++;
      }
    }
  }
  int real_ball_b = 0;
  int* real_ball_radius_pix_b = new int[contours_b.size()];
  int* real_ball_pix_b_x = new int[contours_b.size()];
  int* real_ball_pix_b_y = new int[contours_b.size()];
  float** carpos_b = new float*[contours_b.size()];
  for(int j = 0; j < contours_b.size(); ++j) carpos_b[j] = new float[3];
  for( size_t i = 0; i< contours_b.size(); i++ ) {
    bool diff_ball_b=true;
    if (radius_b[i] > iMin_tracking_ball_size) {
      int pix_x_b=center_b[i].x;
      int pix_y_b=center_b[i].y;
      int radius_pix_b=(radius_b[i]-radcon);
      for (int j=0 ; j<real_ball_b ; j++) {
        int real_pix_x_b=real_ball_pix_b_x[j];
        int real_pix_y_b=real_ball_pix_b_y[j];
        int radius_sum_b=real_ball_radius_pix_b[j]+radius_pix_b;
        int ball_dist_pix_b=pow(pix_x_b-real_pix_x_b,2)+pow(pix_y_b-real_pix_y_b,2);
        if(ball_dist_pix_b < pow(radius_sum_b,2)) {
          if (radius_b[i]>real_ball_radius_pix_b[j]) carpos_b[j][2]=-1;
          else diff_ball_b=false;
        }
      }
      if (diff_ball_b) {
        Scalar color = Scalar( 255, 0, 0);
        vector<float> ball_position_b;
        radius_b[i]=radius_b[i]-radcon;
        ball_position_b = pixel2point(center_b[i], radius_b[i]);
        float isx = ball_position_b[0];
        float isy = ball_position_b[1];
        float isz = ball_position_b[2];
        carpos_b[real_ball_b][0] = isx;
        carpos_b[real_ball_b][1] = isy;
        carpos_b[real_ball_b][2] = isz;
        real_ball_radius_pix_b[real_ball_b]=radius_b[i];
        real_ball_pix_b_x[real_ball_b]=center_b[i].x;
        real_ball_pix_b_y[real_ball_b]=center_b[i].y;
        real_ball_b++;
      }
    }
  }
  int real_ball_g = 0;
  int* real_ball_radius_pix_g = new int[contours_g.size()];
  int* real_ball_pix_g_x = new int[contours_g.size()];
  int* real_ball_pix_g_y = new int[contours_g.size()];
  float** carpos_g = new float*[contours_g.size()];
  for(int j = 0; j < contours_g.size(); ++j) carpos_g[j] = new float[3];
  for( size_t i = 0; i< contours_g.size(); i++ ) {
    bool diff_ball_g=true;
    if (radius_g[i] > iMin_tracking_ball_size) {
      int pix_x_g=center_g[i].x;
      int pix_y_g=center_g[i].y;
      int radius_pix_g=(radius_g[i]-radcon);
      for (int j=0 ; j<real_ball_g ; j++) {
        int real_pix_x_g=real_ball_pix_g_x[j];
        int real_pix_y_g=real_ball_pix_g_y[j];
        int radius_sum_g=real_ball_radius_pix_g[j]+radius_pix_g;
        int ball_dist_pix_g=pow(pix_x_g-real_pix_x_g,2)+pow(pix_y_g-real_pix_y_g,2);
        if(ball_dist_pix_g < pow(radius_sum_g,2)) {
          if (radius_g[i]>real_ball_radius_pix_g[j]) carpos_g[j][2]=-1;
          else diff_ball_g=false;
        }
      }
      if (diff_ball_g) {

        vector<float> ball_position_g;
        radius_g[i]=radius_g[i]-radcon;
        ball_position_g = pixel2point(center_g[i], radius_g[i]);
        float isx = ball_position_g[0];
        float isy = ball_position_g[1];
        float isz = ball_position_g[2];
        carpos_g[real_ball_g][0] = isx;
        carpos_g[real_ball_g][1] = isy;
        carpos_g[real_ball_g][2] = isz;
        real_ball_radius_pix_g[real_ball_g]=radius_g[i];
        real_ball_pix_g_x[real_ball_g]=center_g[i].x;
        real_ball_pix_g_y[real_ball_g]=center_g[i].y;
        real_ball_g++;
      }
    }
  }

  int small_ball_num_r=0;
  for (int i=0 ; i<real_ball_r;i++) {
    if (carpos_r[i][2]==-1) {
      small_ball_num_r++;
      continue;
    }
    for (int j=0; j<3 ; j++) carpos_r[i-small_ball_num_r][j]=carpos_r[i][j];
    real_ball_radius_pix_r[i-small_ball_num_r] = real_ball_radius_pix_r[i];
    real_ball_pix_r_x[i-small_ball_num_r] = real_ball_pix_r_x[i];
    real_ball_pix_r_y[i-small_ball_num_r] = real_ball_pix_r_y[i];
  } real_ball_r=real_ball_r-small_ball_num_r;

  int small_ball_num_b=0;
  for (int i=0 ; i<real_ball_b;i++) {
    if (carpos_b[i][2]==-1) {
      small_ball_num_b++;
      continue;
    }
    for (int j=0; j<3 ; j++) carpos_b[i-small_ball_num_b][j]=carpos_b[i][j];
    real_ball_radius_pix_b[i-small_ball_num_b] = real_ball_radius_pix_b[i];
    real_ball_pix_b_x[i-small_ball_num_b] = real_ball_pix_b_x[i];
    real_ball_pix_b_y[i-small_ball_num_b] = real_ball_pix_b_y[i];
  } real_ball_b=real_ball_b-small_ball_num_b;

  int small_ball_num_g=0;
  for (int i=0 ; i<real_ball_g;i++) {
    if (carpos_g[i][2]==-1) {
      small_ball_num_g++;
      continue;
    }
    for (int j=0; j<3 ; j++) carpos_g[i-small_ball_num_g][j]=carpos_g[i][j];
    real_ball_radius_pix_g[i-small_ball_num_g] = real_ball_radius_pix_g[i];
    real_ball_pix_g_x[i-small_ball_num_g] = real_ball_pix_g_x[i];
    real_ball_pix_g_y[i-small_ball_num_g] = real_ball_pix_g_y[i];
  } real_ball_g=real_ball_g-small_ball_num_g;


  Mat depth_frame;
  cv::resize(depth_mat_glo, depth_frame, cv::Size(1920, 1080));
  float* ball_mean_depth_r = new float[real_ball_r];
  for (int i=0 ; i<real_ball_r; i++) {
    Scalar color = Scalar(0,0,255);
    Mat find_depth_img_r(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
    float pix_r_x_rat = real_ball_pix_r_x[i];
    float pix_r_y_rat = real_ball_pix_r_y[i];
    float ball_rad_r_rat = real_ball_radius_pix_r[i];
    circle(find_depth_img_r, Point2f((int)pix_r_x_rat,(int)pix_r_y_rat),(int)(ball_rad_r_rat*0.25),color,CV_FILLED,8,0);
    Mat mask_r;
    inRange(find_depth_img_r, Scalar(0, 0, 254, 0), Scalar(0, 0, 255, 0), mask_r);
    Scalar mean_value_r;
    mean_value_r = mean(depth_frame, mask_r); //Mean distance value of red pixels
    float mean_val_r = sum(mean_value_r)[0];
    ball_mean_depth_r[i]=mean_val_r;
  }
  float* ball_mean_depth_b = new float[real_ball_b];
  for (int i=0 ; i<real_ball_b; i++) {
    Scalar color = Scalar(255,0,0);
    float pix_b_x_rat = real_ball_pix_b_x[i];
    float pix_b_y_rat = real_ball_pix_b_y[i];
    float ball_rad_b_rat = real_ball_radius_pix_b[i];
    Mat find_depth_img_b(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
    circle(find_depth_img_b, Point2f((int)pix_b_x_rat,(int)pix_b_y_rat),(int)(ball_rad_b_rat*0.25),color,CV_FILLED,8,0);
    Mat mask_b;
    inRange(find_depth_img_b, Scalar(254, 0, 0, 0), Scalar(255, 0, 0, 0), mask_b);
    Scalar mean_value_b;
    mean_value_b = mean(depth_frame, mask_b);		//Mean distance value of red pixels
    float mean_val_b = sum(mean_value_b)[0];
    ball_mean_depth_b[i]=mean_val_b;
  }
  float* ball_mean_depth_g = new float[real_ball_g];
  for (int i=0 ; i<real_ball_g; i++) {
    Scalar color = Scalar(0,255,0);
    float pix_g_x_rat = real_ball_pix_g_x[i];
    float pix_g_y_rat = real_ball_pix_g_y[i];
    float ball_rad_g_rat = real_ball_radius_pix_g[i];
    Mat find_depth_img_g(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
    circle(find_depth_img_g, Point2f((int)pix_g_x_rat,(int)pix_g_y_rat),(int)(ball_rad_g_rat*0.25),color,CV_FILLED,8,0);
    Mat mask_g;
    inRange(find_depth_img_g, Scalar(0, 254, 0, 0), Scalar(0, 255, 0, 0), mask_g);
    Scalar mean_value_g;
    mean_value_g = mean(depth_frame, mask_g);		//Mean distance value of red pixels
    float mean_val_g = sum(mean_value_g)[0];
    ball_mean_depth_g[i]=mean_val_g;
  }

/*
  for(int k=0; k<real_ball_g; k++)
  {
    if (ball_mean_depth_g[k] == 0) ball_mean_depth_g[k] = carpos_g[k][2];
    if (ball_mean_depth_g[k]*0.7 < carpos_g[k][2])
    {


      vector<Vec4i> hierarchy_roi_g;
      vector<vector<Point> > contours_roi_g;
      findContours(roi_mask_f_g, contours_roi_g, hierarchy_roi_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
      vector<vector<Point> > contours_roi_g_poly( contours_roi_g.size() );
      vector<Point2f> center_roi_g( contours_roi_g.size() );
      vector<float> radius_roi_g( contours_roi_g.size() );
      for( size_t i = 0; i < contours_roi_g.size(); i++ ) {
        approxPolyDP( contours_roi_g[i], contours_roi_g_poly[i], 1, true );
        minEnclosingCircle( contours_roi_g_poly[i], center_roi_g[i], radius_roi_g[i] );
      }

      int real_ball_roi_g = 0;
      int* real_ball_radius_pix_roi_g = new int[contours_roi_g.size()];
      int* real_ball_pix_roi_g_x = new int[contours_roi_g.size()];
      int* real_ball_pix_roi_g_y = new int[contours_roi_g.size()];
      float** carpos_roi_g = new float*[contours_roi_g.size()];
      for(int j = 0; j < contours_roi_g.size(); ++j) carpos_roi_g[j] = new float[3];

      for( size_t i = 0; i< contours_roi_g.size(); i++ )
      {
        bool diff_ball_roi_g=true;
        if (radius_roi_g[i] > iMin_tracking_ball_size)
        {
          int pix_x_roi_g=center_roi_g[i].x;
          int pix_y_roi_g=center_roi_g[i].y;
          int radius_pix_roi_g=(radius_roi_g[i]-radcon);
          for (int j=0 ; j<real_ball_roi_g ; j++)
          {
            int real_pix_x_roi_g=real_ball_pix_roi_g_x[j];
            int real_pix_y_roi_g=real_ball_pix_roi_g_y[j];
            int radius_sum_roi_g=real_ball_radius_pix_roi_g[j]+radius_pix_roi_g;
            int ball_dist_pix_roi_g=pow(pix_x_roi_g-real_pix_x_roi_g,2)+pow(pix_y_roi_g-real_pix_y_roi_g,2);
            if(ball_dist_pix_roi_g < pow(radius_sum_roi_g,2))
            {
              if (radius_roi_g[i]>real_ball_radius_pix_roi_g[j]) carpos_g[j][2]=-1;
              else diff_ball_roi_g=false;
            }
          }
          if (diff_ball_roi_g)
          {
            vector<float> ball_position_roi_g;
            radius_roi_g[i]=radius_roi_g[i]-radcon;
            center_roi_g[i].x=roi_x+center_roi_g[i].x;
            center_roi_g[i].y=roi_y+center_roi_g[i].y;
            ball_position_roi_g = pixel2point(center_roi_g[i], radius_roi_g[i]);
            float isx = ball_position_roi_g[0];
            float isy = ball_position_roi_g[1];
            float isz = ball_position_roi_g[2];
            carpos_roi_g[real_ball_roi_g][0] = isx;
            carpos_roi_g[real_ball_roi_g][1] = isy;
            carpos_roi_g[real_ball_roi_g][2] = isz;
            real_ball_radius_pix_roi_g[real_ball_roi_g]=radius_roi_g[i];
            real_ball_pix_roi_g_x[real_ball_roi_g]=center_roi_g[i].x;
            real_ball_pix_roi_g_y[real_ball_roi_g]=center_roi_g[i].y;
            real_ball_roi_g++;
          }
        }
      }





    }
  }
*/



  for (int i=0 ; i<real_ball_r; i++) {
    Scalar color = Scalar( 0, 0, 255);
    float isx = carpos_r[i][0];
    float isy = carpos_r[i][1];
    float isz = carpos_r[i][2];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    text = "Red ball:" + sx + "," + sy + "," + sz;
    putText(result, text, Point2f(real_ball_pix_r_x[i],real_ball_pix_r_y[i]),2,1,Scalar(0,255,0),2);
    circle(result, Point2f(real_ball_pix_r_x[i],real_ball_pix_r_y[i]), (int)real_ball_radius_pix_r[i], color, 2, 8, 0 );
  }
  for (int i=0 ; i<real_ball_b ; i++) {
    Scalar color = Scalar( 255, 0, 0);
    float isx = carpos_b[i][0];
    float isy = carpos_b[i][1];
    float isz = carpos_b[i][2];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    text = "Blue ball:" + sx + "," + sy + "," + sz;
    putText(result, text, Point2f(real_ball_pix_b_x[i],real_ball_pix_b_y[i]),2,1,Scalar(0,255,0),2);
    circle( result, Point2f(real_ball_pix_b_x[i],real_ball_pix_b_y[i]), (int)real_ball_radius_pix_b[i], color, 2, 8, 0 );
  }

  for (int i=0 ; i<real_ball_g ; i++) {
    Scalar color = Scalar( 0, 255, 0);
    float isx = carpos_g[i][0];
    float isy = carpos_g[i][1];
    float isz = carpos_g[i][2];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    text = "Green ball:" + sx + "," + sy + "," + sz;
    putText(result, text, Point2f(real_ball_pix_g_x[i],real_ball_pix_g_y[i]),2,1,Scalar(0,255,0),2);
    circle( result, Point2f(real_ball_pix_g_x[i],real_ball_pix_g_y[i]), (int)real_ball_radius_pix_g[i], color, 2, 8, 0 );
  }


  core_msgs::ball_position msg;
  msg.r_size = real_ball_r;
  msg.r_img_x.resize(real_ball_r);
  msg.r_img_y.resize(real_ball_r);
  msg.b_size = real_ball_b;
  msg.b_img_x.resize(real_ball_b);
  msg.b_img_y.resize(real_ball_b);
  msg.g_size = real_ball_g;
  msg.g_img_x.resize(real_ball_g);
  msg.g_img_y.resize(real_ball_g);
  int overlap_b = -1;
  for(int k=0; k<real_ball_r; k++) {
    if (ball_mean_depth_r[k]*0.7 < carpos_r[k][2] && ball_mean_depth_r[k]*1.3 > carpos_r[k][2]) {
      msg.r_img_x[k] = carpos_r[k][0]*100;
      msg.r_img_y[k] = (ball_mean_depth_r[k]*cos(theta* PI / 180.0)-carpos_r[k][1]*sin(theta* PI / 180.0))*100;
      std::cout << "red ball " << ball_mean_depth_r[k] << endl;
    }
    else {
      msg.r_img_x[k] = 0;
      msg.r_img_y[k] = -100;
      std::cout << "red ball -100" << endl;
    }
  }
  for(int k=0; k<real_ball_b; k++) {
    if (ball_mean_depth_b[k]*0.7 > carpos_b[k][2]) {
      overlap_b = k;
      double ratio_over = ball_mean_depth_b[k] / carpos_b[k][2];
      msg.b_img_x[k] = carpos_b[k][0]*100*ratio_over;
      msg.b_img_y[k] = (ball_mean_depth_b[k]*cos(theta* PI / 180.0)-carpos_b[k][1]*sin(theta* PI / 180.0))*100*ratio_over;
      std::cout << "blue ball " << ball_mean_depth_b[k] << endl;
    }
    else if(ball_mean_depth_b[k]*1.3<carpos_b[k][2])
    {
      msg.b_img_x[k]=0;
      msg.b_img_y[k]=-100;
    }
    else {
      msg.b_img_x[k] = carpos_b[k][0]*100;
      msg.b_img_y[k] = (ball_mean_depth_b[k]*cos(theta* PI / 180.0)-carpos_b[k][1]*sin(theta* PI / 180.0))*100;
      std::cout << "blue ball " << ball_mean_depth_b[k] << endl;
    }
  }
  msg.b_over = overlap_b;
  for(int k=0; k<real_ball_g; k++)
  {
    if (ball_mean_depth_g[k]*1.2 > carpos_g[k][2] )
    {
      msg.g_img_x[k] = carpos_g[k][0]*100;
      msg.g_img_y[k] = (ball_mean_depth_g[k]*cos(theta* PI / 180.0)-carpos_g[k][1]*sin(theta* PI / 180.0))*100;
      std::cout << "green ball_msg_x" << msg.g_img_x[k]<<endl;
      std::cout << "green ball_msg_y" << msg.g_img_y[k]<<endl;
    }
    else
    {
      msg.g_img_x[k] = 0;
      msg.g_img_y[k] = -100;
    }
    //std::cout << "green ball " << ball_mean_depth_g[k] << endl;
    /*
    else
    {
      int roi_x=real_ball_pix_g_x[k];
      int roi_y=real_ball_pix_g_y[k];
      int roi_r=real_ball_radius_pix_g[k]-radcon;
      float roi_d=ball_mean_depth_g[k];
      float roi_d_low=roi_d-0.15;
      float roi_d_high=roi_d+0.15;
      Mat roi_g;
      roi_g=depth_frame(Rect(roi_x-(int)(roi_r),roi_y-(int)(roi_r), (int)(2*roi_r), (int)(2*roi_r)));
      Mat roi_mask_g;
      inRange(roi_g,roi_d_low, roi_d_high, roi_mask_g);
      Mat roi_hsv_g;
      roi_hsv_g=hsv_frame_green(Rect(roi_x-(int)(roi_r),roi_y-(int)(roi_r), (int)(2*roi_r), (int)(2*roi_r)));
      Mat roi_mask_f_g;
      bitwise_and(roi_mask_g,roi_hsv_g,roi_mask_f_g);
      imshow("ROI",roi_g);
      imshow("ROI_MASK", roi_mask_g);
      imshow("ROI_HSV",roi_hsv_g);
      imshow("ROI_F",roi_mask_f_g);
      cout<<"mid"<<roi_d<<endl;
      cout<<"low"<<roi_d_low<<endl;
      cout<<"high"<<roi_d_high<<endl;


    }*/
  }


  namedWindow("result", WINDOW_NORMAL);
  imshow("result",result);
  /*
  visualization_msgs::Marker ball_list;  //declare marker
  ball_list.header.frame_id = "/camera_link";  //set the frame
  ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.os
  ball_list.ns = "balls";   //name of markers
  ball_list.action = visualization_msgs::Marker::ADD;
  ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
  ball_list.pose.position.y=0;
  ball_list.pose.position.z=0;
  ball_list.pose.orientation.x=0;
  ball_list.pose.orientation.y=0;
  ball_list.pose.orientation.z=0;
  ball_list.pose.orientation.w=1.0;
  ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
  ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker
  double radius = 0.10;
  ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
  ball_list.scale.y=radius;
  ball_list.scale.z=radius;
  */
  pub.publish(msg);  //publish a message
  if (real_ball_r>0 && contours_r.size()>0) {
    delete real_ball_radius_pix_r;
    delete real_ball_pix_r_x;
    delete real_ball_pix_r_y;
    delete carpos_r;
    delete ball_mean_depth_r;
  }
  if (real_ball_b>0 && contours_b.size()>0) {
    delete real_ball_radius_pix_b;
    delete real_ball_pix_b_x;
    delete real_ball_pix_b_y;
    delete carpos_b;
    delete ball_mean_depth_b;
  }
  if (real_ball_g>0 && contours_g.size()>0) {
    delete real_ball_radius_pix_g;
    delete real_ball_pix_g_x;
    delete real_ball_pix_g_y;
    delete carpos_g;
    delete ball_mean_depth_g;
  }
}

void depthImage_cb(const sensor_msgs::ImageConstPtr &msg_depth){
  try {
  	cv_bridge::CvImageConstPtr depth_img_cv;
  	depth_img_cv = cv_bridge::toCvShare (msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  	depth_img_cv->image.convertTo(depth_mat_glo, CV_32F, 0.001);
    waitKey(1);
  }
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'z16'.", msg_depth->encoding.c_str());}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  ball_detect();
}

int main(int argc, char **argv) {
  /*
  Mat test(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
  circle(test, Point2f(0,0),(int)350, Scalar(255,255,255),CV_FILLED,8,0);
  imshow("test", test);
  waitKey(0);
  */
  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthImage_cb);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
  pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
  ros::Rate loop_rate(30);
  cout << "while start"<<endl;
  while(ros::ok()){
     ros::spinOnce();
     loop_rate.sleep();
  }
  return 0;
}
