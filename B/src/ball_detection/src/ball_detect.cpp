#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_pos.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>


using namespace cv;
using namespace std;


Mat buffer(320,240,CV_8UC1);
ros::Publisher pub;
ros::Publisher pub_markers;

// Red_Ball
void on_low_h_thresh_trackbar_red_1(int, void *);
void on_low_h_thresh_trackbar_red_2(int, void *);
void on_high_h_thresh_trackbar_red_1(int, void *);
void on_high_h_thresh_trackbar_red_2(int, void *);
void on_low_h2_thresh_trackbar_red_1(int, void *);
void on_low_h2_thresh_trackbar_red_2(int, void *);
void on_high_h2_thresh_trackbar_red_1(int, void *);
void on_high_h2_thresh_trackbar_red_2(int, void *);
void on_low_s_thresh_trackbar_red_1(int, void *);
void on_low_s_thresh_trackbar_red_2(int, void *);
void on_high_s_thresh_trackbar_red_1(int, void *);
void on_high_s_thresh_trackbar_red_2(int, void *);
void on_low_v_thresh_trackbar_red_1(int, void *);
void on_low_v_thresh_trackbar_red_2(int, void *);
void on_high_v_thresh_trackbar_red_1(int, void *);
void on_high_v_thresh_trackbar_red_2(int, void *);
int low_h2_r_1=161, high_h2_r_1=180;
int low_h2_r_2=161, high_h2_r_2=180;
int low_h_r_1=0, low_s_r_1=94, low_v_r_1=95;
int low_h_r_2=0, low_s_r_2=94, low_v_r_2=95;
int high_h_r_1=10, high_s_r_1=255, high_v_r_1=255;
int high_h_r_2=10, high_s_r_2=255, high_v_r_2=255;
//Blue Ball
void on_low_h_thresh_trackbar_blue_1(int, void *);
void on_low_h_thresh_trackbar_blue_2(int, void *);
void on_high_h_thresh_trackbar_blue_1(int, void *);
void on_high_h_thresh_trackbar_blue_2(int, void *);
void on_low_s_thresh_trackbar_blue_1(int, void *);
void on_low_s_thresh_trackbar_blue_2(int, void *);
void on_high_s_thresh_trackbar_blue_1(int, void *);
void on_high_s_thresh_trackbar_blue_2(int, void *);
void on_low_v_thresh_trackbar_blue_1(int, void *);
void on_low_v_thresh_trackbar_blue_2(int, void *);
void on_high_v_thresh_trackbar_blue_1(int, void *);
void on_high_v_thresh_trackbar_blue_2(int, void *);
int low_h_b_1=90, low_s_b_1=100, low_v_b_1=100;
int high_h_b_1=130, high_s_b_1=255, high_v_b_1=255;
int low_h_b_2=90, low_s_b_2=100, low_v_b_2=100;
int high_h_b_2=130, high_s_b_2=255, high_v_b_2=255;
 //Green Ball
void on_low_h_thresh_trackbar_green_1(int, void *);
void on_low_h_thresh_trackbar_green_2(int, void *);
void on_high_h_thresh_trackbar_green_1(int, void *);
void on_high_h_thresh_trackbar_green_2(int, void *);
void on_low_s_thresh_trackbar_green_1(int, void *);
void on_low_s_thresh_trackbar_green_2(int, void *);
void on_high_s_thresh_trackbar_green_1(int, void *);
void on_high_s_thresh_trackbar_green_2(int, void *);
void on_low_v_thresh_trackbar_green_1(int, void *);
void on_low_v_thresh_trackbar_green_2(int, void *);
void on_high_v_thresh_trackbar_green_1(int, void *);
void on_high_v_thresh_trackbar_green_2(int, void *);
int low_h_g_1=30, low_s_g_1=100, low_v_g_1=49;
int high_h_g_1=80, high_s_g_1=255, high_v_g_1=255;
int low_h_g_2=30, low_s_g_2=100, low_v_g_2=49;
int high_h_g_2=80, high_s_g_2=255, high_v_g_2=255;
// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);
// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);
// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point_1(Point center_1, int radius_1);//change
vector<float> pixel2point_2(Point center_2, int radius_2);//change
// Declaration of trackbars function that set canny edge's parameters
void on_canny_edge_trackbar_red(int, void *);
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;
void on_canny_edge_trackbar_green(int, void *);
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;
// Initialization of variable for dimension of the target
float fball_radius = 0.072 ; // meter
// Initialization of variable for camera calibration paramters

Mat distCoeffs;
float intrinsic_data_1[9] = {640.1159311178064, 0.0, 310.1838077432763, 0.0, 643.2627383990927, 245.6804732994471, 0.0, 0.0, 1.0};//change
float distortion_data_1[5] = {0.04217408540221617, -0.12869625203625892, 0.0031626402785077242, -0.00453397005336903, 0.0};
float intrinsic_data_2[9] = {630.3498179789707, 0.0, 319.37260121468324, 0.0, 633.1882173869892, 238.98330179295047, 0.0, 0.0, 1.0};//change
float distortion_data_2[5] = {0.0038749795465425926, -0.08779331036450917, -0.002573957942542779, -0.0009438151615896782, 0.0};
// Initialization of variable for Camera Matrix
float Rotation_matrix_data[9] = {0.9998371379964848, -0.008370031412091706, 0.01598874782961575, 0.008288266440529145, 0.9999522694312795, 0.0051733450150507326, -0.016031285737870264, -0.005039983471654451, 0.9998587882517193};
float Transfer_matrix_data[3] = {-0.18184857994284165, 0.0011076329446815109, -0.0052189264432324885};
// Initialization of variable for text drawing
double fontScale = 2;
int thickness = 3;
String text ;
int iMin_tracking_ball_size = 0;

string intToString(int n);
string floatToString(float f);

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

vector<float> pixel2point_1(Point center_1, int radius_1){
vector<float> position_1;
float x_1, y_1, u_1, v_1, Xc_1, Yc_1, Zc_1;
x_1 = center_1.x;//.x;// .at(0);
y_1 = center_1.y;//.y;//
u_1 = (x_1-intrinsic_data_1[2])/intrinsic_data_1[0];
v_1 = (y_1-intrinsic_data_1[5])/intrinsic_data_1[4];
Zc_1 = (intrinsic_data_1[0]*fball_radius)/(2*(float)radius_1) ;
Xc_1 = u_1*Zc_1 ;
Yc_1 = v_1*Zc_1 ;

position_1.push_back(Xc_1);
position_1.push_back(Yc_1);
position_1.push_back(Zc_1);
return position_1;
}
vector<float> pixel2point_2(Point center_2, int radius_2){
vector<float> position_2;
float x_2, y_2, u_2, v_2, Xc_2, Yc_2, Zc_2;
x_2 = center_2.x;//.x;// .at(0);
y_2 = center_2.y;//.y;//
u_2 = (x_2-intrinsic_data_2[2])/intrinsic_data_2[0];
v_2 = (y_2-intrinsic_data_2[5])/intrinsic_data_2[4];
Zc_2 = (intrinsic_data_2[0]*fball_radius)/(2*(float)radius_2) ;
Xc_2 = u_2*Zc_2 ;
Yc_2 = v_2*Zc_2 ;

position_2.push_back(Xc_2);
position_2.push_back(Yc_2);
position_2.push_back(Zc_2);
return position_2;
}

// Trackbar for image threshodling in HSV colorspace : Red1
void on_low_h_thresh_trackbar_red_1(int, void *){
low_h_r_1 = min(high_h_r_1-1, low_h_r_1);
setTrackbarPos("Low H","Object Detection_HSV_Red1", low_h_r_1);
}
void on_high_h_thresh_trackbar_red_1(int, void *){
high_h_r_1 = max(high_h_r_1, low_h_r_1+1);
setTrackbarPos("High H", "Object Detection_HSV_Red1", high_h_r_1);
}
void on_low_h2_thresh_trackbar_red_1(int, void *){
low_h2_r_1 = min(high_h2_r_1-1, low_h2_r_1);
setTrackbarPos("Low H2","Object Detection_HSV_Red1", low_h2_r_1);
}
void on_high_h2_thresh_trackbar_red_1(int, void *){
high_h_r_1 = max(high_h_r_1, low_h_r_1+1);
setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r_1);
}
void on_low_s_thresh_trackbar_red_1(int, void *){
low_s_r_1 = min(high_s_r_1-1, low_s_r_1);
setTrackbarPos("Low S","Object Detection_HSV_Red1", low_s_r_1);
}
void on_high_s_thresh_trackbar_red_1(int, void *){
high_s_r_1 = max(high_s_r_1, low_s_r_1+1);
setTrackbarPos("High S", "Object Detection_HSV_Red1", high_s_r_1);
}
void on_low_v_thresh_trackbar_red_1(int, void *){
low_v_r_1= min(high_v_r_1-1, low_v_r_1);
setTrackbarPos("Low V","Object Detection_HSV_Red1", low_v_r_1);
}
void on_high_v_thresh_trackbar_red_1(int, void *){
high_v_r_1 = max(high_v_r_1, low_v_r_1+1);
setTrackbarPos("High V", "Object Detection_HSV_Red1", high_v_r_1);
}

// Trackbar for image threshodling in HSV colorspace : Red2
void on_low_h_thresh_trackbar_red_2(int, void *){
low_h_r_2 = min(high_h_r_2-1, low_h_r_2);
setTrackbarPos("Low H","Object Detection_HSV_Red2", low_h_r_2);
}
void on_high_h_thresh_trackbar_red_2(int, void *){
high_h_r_2 = max(high_h_r_2, low_h_r_2+1);
setTrackbarPos("High H", "Object Detection_HSV_Red2", high_h_r_2);
}
void on_low_h2_thresh_trackbar_red_2(int, void *){
low_h2_r_2 = min(high_h2_r_2-1, low_h2_r_2);
setTrackbarPos("Low H2","Object Detection_HSV_Red2", low_h2_r_2);
}
void on_high_h2_thresh_trackbar_red_2(int, void *){
high_h_r_2 = max(high_h_r_2, low_h_r_2+1);
setTrackbarPos("High H", "Object Detection_HSV_Red2", high_h_r_2);
}
void on_low_s_thresh_trackbar_red_2(int, void *){
low_s_r_2 = min(high_s_r_2-1, low_s_r_2);
setTrackbarPos("Low S","Object Detection_HSV_Red2", low_s_r_2);
}
void on_high_s_thresh_trackbar_red_2(int, void *){
high_s_r_2 = max(high_s_r_2, low_s_r_2+1);
setTrackbarPos("High S", "Object Detection_HSV_Red2", high_s_r_2);
}
void on_low_v_thresh_trackbar_red_2(int, void *){
low_v_r_2= min(high_v_r_2-1, low_v_r_2);
setTrackbarPos("Low V","Object Detection_HSV_Red2", low_v_r_2);
}
void on_high_v_thresh_trackbar_red_2(int, void *){
high_v_r_2 = max(high_v_r_2, low_v_r_2+1);
setTrackbarPos("High V", "Object Detection_HSV_Red2", high_v_r_2);
}

// Trackbar for image threshodling in HSV colorspace : Blue1
void on_low_h_thresh_trackbar_blue_1(int, void *){
low_h_b_1 = min(high_h_b_1-1, low_h_b_1);
setTrackbarPos("Low H","Object Detection_HSV_Blue1", low_h_b_1);
}
void on_high_h_thresh_trackbar_blue_1(int, void *){
high_h_b_1 = max(high_h_b_1, low_h_b_1+1);
setTrackbarPos("High H", "Object Detection_HSV_Blue1", high_h_b_1);
}
void on_low_s_thresh_trackbar_blue_1(int, void *){
low_s_b_1 = min(high_s_b_1-1, low_s_b_1);
setTrackbarPos("Low S","Object Detection_HSV_Blue1", low_s_b_1);
}
void on_high_s_thresh_trackbar_blue_1(int, void *){
high_s_b_1 = max(high_s_b_1, low_s_b_1+1);
setTrackbarPos("High S", "Object Detection_HSV_Blue1", high_s_b_1);
}
void on_low_v_thresh_trackbar_blue_1(int, void *){
low_v_b_1= min(high_v_b_1-1, low_v_b_1);
setTrackbarPos("Low V","Object Detection_HSV_Blue1", low_v_b_1);
}
void on_high_v_thresh_trackbar_blue_1(int, void *){
high_v_b_1 = max(high_v_b_1, low_v_b_1+1);
setTrackbarPos("High V", "Object Detection_HSV_Blue1", high_v_b_1);
}

// Trackbar for image threshodling in HSV colorspace : Blue2
void on_low_h_thresh_trackbar_blue_2(int, void *){
low_h_b_2 = min(high_h_b_2-1, low_h_b_2);
setTrackbarPos("Low H","Object Detection_HSV_Blue2", low_h_b_2);
}
void on_high_h_thresh_trackbar_blue_2(int, void *){
high_h_b_2 = max(high_h_b_2, low_h_b_2+1);
setTrackbarPos("High H", "Object Detection_HSV_Blue2", high_h_b_2);
}
void on_low_s_thresh_trackbar_blue_2(int, void *){
low_s_b_2 = min(high_s_b_2-1, low_s_b_2);
setTrackbarPos("Low S","Object Detection_HSV_Blue2", low_s_b_2);
}
void on_high_s_thresh_trackbar_blue_2(int, void *){
high_s_b_2 = max(high_s_b_2, low_s_b_2+1);
setTrackbarPos("High S", "Object Detection_HSV_Blue2", high_s_b_2);
}
void on_low_v_thresh_trackbar_blue_2(int, void *){
low_v_b_2= min(high_v_b_2-1, low_v_b_2);
setTrackbarPos("Low V","Object Detection_HSV_Blue2", low_v_b_2);
}
void on_high_v_thresh_trackbar_blue_2(int, void *){
high_v_b_2 = max(high_v_b_2, low_v_b_2+1);
setTrackbarPos("High V", "Object Detection_HSV_Blue2", high_v_b_2);
}

// Trackbar for image threshodling in HSV colorspace : Green1
void on_low_h_thresh_trackbar_green_1(int, void *){
low_h_g_1 = min(high_h_g_1-1, low_h_g_1);
setTrackbarPos("Low H","Object Detection_HSV_Green1", low_h_g_1);
}
void on_high_h_thresh_trackbar_green_1(int, void *){
high_h_g_1 = max(high_h_g_1, low_h_g_1+1);
setTrackbarPos("High H", "Object Detection_HSV_Green1", high_h_g_1);
}
void on_low_s_thresh_trackbar_green_1(int, void *){
low_s_g_1 = min(high_s_g_1-1, low_s_g_1);
setTrackbarPos("Low S","Object Detection_HSV_Green1", low_s_g_1);
}
void on_high_s_thresh_trackbar_green_1(int, void *){
high_s_g_1 = max(high_s_g_1, low_s_g_1+1);
setTrackbarPos("High S", "Object Detection_HSV_Green1", high_s_g_1);
}
void on_low_v_thresh_trackbar_green_1(int, void *){
low_v_g_1= min(high_v_g_1-1, low_v_g_1);
setTrackbarPos("Low V","Object Detection_HSV_Green1", low_v_g_1);
}
void on_high_v_thresh_trackbar_green_1(int, void *){
high_v_g_1 = max(high_v_g_1, low_v_g_1+1);
setTrackbarPos("High V", "Object Detection_HSV_Green1", high_v_g_1);
}

// Trackbar for image threshodling in HSV colorspace : Green2
void on_low_h_thresh_trackbar_green_2(int, void *){
low_h_g_2 = min(high_h_g_2-1, low_h_g_2);
setTrackbarPos("Low H","Object Detection_HSV_Green2", low_h_g_2);
}
void on_high_h_thresh_trackbar_green_2(int, void *){
high_h_g_2 = max(high_h_g_2, low_h_g_2+1);
setTrackbarPos("High H", "Object Detection_HSV_Green2", high_h_g_2);
}
void on_low_s_thresh_trackbar_green_2(int, void *){
low_s_g_2 = min(high_s_g_2-1, low_s_g_2);
setTrackbarPos("Low S","Object Detection_HSV_Green2", low_s_g_2);
}
void on_high_s_thresh_trackbar_green_2(int, void *){
high_s_g_2 = max(high_s_g_2, low_s_g_2+1);
setTrackbarPos("High S", "Object Detection_HSV_Green2", high_s_g_2);
}
void on_low_v_thresh_trackbar_green_2(int, void *){
low_v_g_2= min(high_v_g_2-1, low_v_g_2);
setTrackbarPos("Low V","Object Detection_HSV_Green2", low_v_g_2);
}
void on_high_v_thresh_trackbar_green_2(int, void *){
high_v_g_2 = max(high_v_g_2, low_v_g_2+1);
setTrackbarPos("High V", "Object Detection_HSV_Green2", high_v_g_2);
}
//

void ball_detect(){
     //Mat edges;  //assign a memory to save the edge images
     //Mat frame;  //assign a memory to save the images
     Mat frame, frame_1,frame_2, hsv_frame_1, hsv_frame_2,
     hsv_frame_1_red, hsv_frame_2_red, hsv_frame_1_red1, hsv_frame_2_red1, hsv_frame_1_red2, hsv_frame_2_red2,
     hsv_frame_1_red_canny, hsv_frame_2_red_canny,
     hsv_frame_1_red_blur, hsv_frame_2_red_blur,
     hsv_frame_1_blue, hsv_frame_2_blue,
     hsv_frame_1_blue_canny, hsv_frame_2_blue_canny,
     hsv_frame_1_blue_blur, hsv_frame_2_blue_blur,
     hsv_frame_1_green, hsv_frame_2_green,
     hsv_frame_1_green_canny, hsv_frame_2_green_canny,
     hsv_frame_1_green_blur, hsv_frame_2_green_blur,
     result_1, result_2, stereo_1, stereo_2;
     Mat calibrated_frame_1;
     Mat calibrated_frame_2;
     Mat intrinsic_1 = Mat(3,3, CV_32FC1);//change
     Mat intrinsic_2 = Mat(3,3, CV_32FC1);//change
     Mat distCoeffs_1, distCoeffs_2;
     intrinsic_1 = Mat(3, 3, CV_32F, intrinsic_data_1);//change
     intrinsic_2 = Mat(3, 3, CV_32F, intrinsic_data_2);//change
     distCoeffs_1 = Mat(1, 5, CV_32F, distortion_data_1);
     distCoeffs_2 = Mat(1, 5, CV_32F, distortion_data_2);
     vector<Vec4i> hierarchy_r_1;
     vector<Vec4i> hierarchy_r_2;
     vector<Vec4i> hierarchy_b_1;
     vector<Vec4i> hierarchy_g_1;
     vector<Vec4i> hierarchy_b_2;
     vector<Vec4i> hierarchy_g_2;
     vector<vector<Point> > contours_r_1;
     vector<vector<Point> > contours_r_2;
     vector<vector<Point> > contours_b_1;
     vector<vector<Point> > contours_g_1;
     vector<vector<Point> > contours_b_2;
     vector<vector<Point> > contours_g_2;

Mat Rotation_matrix = Mat(3,3, CV_32FC1);
Mat Transfer_matrix;
Rotation_matrix = Mat(3, 3, CV_32F, Rotation_matrix_data);
Transfer_matrix = Mat(3, 1, CV_32F, Transfer_matrix_data);
Mat Camera_2_center = Mat::zeros(3,1, CV_32F);
Mat Camera_1_center = Mat(3,1, CV_32F, Transfer_matrix_data);

     if(buffer.size().width==640){ //if the size of the image is 320x240, then resized it to 640x480
         cv::resize(buffer, frame, cv::Size(1280, 480));
     }
     else{
         frame = buffer;
     }

  frame_1=frame(Range(0,480), Range(0,640));
  frame_2=frame(Range(0,480), Range(640,1280));

  undistort(frame_1, calibrated_frame_1, intrinsic_1, distCoeffs_1);//change
  result_1 = calibrated_frame_1.clone();
  stereo_1 = calibrated_frame_1.clone();
  medianBlur(calibrated_frame_1, calibrated_frame_1, 3);
  cvtColor(calibrated_frame_1, hsv_frame_1, cv::COLOR_BGR2HSV);


  undistort(frame_2, calibrated_frame_2, intrinsic_2, distCoeffs_2);//change
  result_2 = calibrated_frame_2.clone();
  stereo_2 = calibrated_frame_2.clone();
  medianBlur(calibrated_frame_2, calibrated_frame_2, 3);
  cvtColor(calibrated_frame_2, hsv_frame_2, cv::COLOR_BGR2HSV);

  // Detect the object based on RGB and HSV Range Values
  inRange(hsv_frame_1,Scalar(low_h_r_1,low_s_r_1,low_v_r_1),Scalar(high_h_r_1,high_s_r_1,high_v_r_1),hsv_frame_1_red1);
  inRange(hsv_frame_1,Scalar(low_h2_r_1,low_s_r_1,low_v_r_1),Scalar(high_h2_r_1,high_s_r_1,high_v_r_1),hsv_frame_1_red2);
  inRange(hsv_frame_2,Scalar(low_h_r_2,low_s_r_2,low_v_r_2),Scalar(high_h_r_2,high_s_r_2,high_v_r_2),hsv_frame_2_red1);
  inRange(hsv_frame_2,Scalar(low_h2_r_2,low_s_r_2,low_v_r_2),Scalar(high_h2_r_2,high_s_r_2,high_v_r_2),hsv_frame_2_red2);
  inRange(hsv_frame_1,Scalar(low_h_b_1,low_s_b_1,low_v_b_1),Scalar(high_h_b_1,high_s_b_1,high_v_b_1),hsv_frame_1_blue);
  inRange(hsv_frame_2,Scalar(low_h_b_2,low_s_b_2,low_v_b_2),Scalar(high_h_b_2,high_s_b_2,high_v_b_2),hsv_frame_2_blue);
  inRange(hsv_frame_1,Scalar(low_h_g_1,low_s_g_1,low_v_g_1),Scalar(high_h_g_1,high_s_g_1,high_v_g_1),hsv_frame_1_green);
  inRange(hsv_frame_2,Scalar(low_h_g_2,low_s_g_2,low_v_g_2),Scalar(high_h_g_2,high_s_g_2,high_v_g_2),hsv_frame_2_green);
  addWeighted(hsv_frame_1_red1, 1.0, hsv_frame_1_red2, 1.0, 0.0, hsv_frame_1_red);
  addWeighted(hsv_frame_2_red1, 1.0, hsv_frame_2_red2, 1.0, 0.0, hsv_frame_2_red);
  morphOps(hsv_frame_1_red);
  morphOps(hsv_frame_2_red);
  morphOps(hsv_frame_1_blue);
  morphOps(hsv_frame_1_green);
  morphOps(hsv_frame_2_blue);
  morphOps(hsv_frame_2_green);

  //Camera One
  //cout<<"Camera1_line_info"<<endl;
  GaussianBlur(hsv_frame_1_red, hsv_frame_1_red_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_1_blue, hsv_frame_1_blue_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_1_green, hsv_frame_1_green_blur, cv::Size(9, 9), 2, 2);
  Canny(hsv_frame_1_red_blur, hsv_frame_1_red_canny, lowThreshold_r*ratio_r, kernel_size_r);
  Canny(hsv_frame_1_blue_blur, hsv_frame_1_blue_canny, lowThreshold_b*ratio_b, kernel_size_b);
  Canny(hsv_frame_1_green_blur, hsv_frame_1_green_canny, lowThreshold_g*ratio_g, kernel_size_g);
  findContours(hsv_frame_1_red_canny, contours_r_1, hierarchy_r_1, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(hsv_frame_1_blue_canny, contours_b_1, hierarchy_b_1, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(hsv_frame_1_green_canny, contours_g_1, hierarchy_g_1, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<vector<Point> > contours_r_poly_1( contours_r_1.size() );
  vector<vector<Point> > contours_b_poly_1( contours_b_1.size() );
  vector<vector<Point> > contours_g_poly_1( contours_g_1.size() );
  vector<Point2f>center_r_1( contours_r_1.size() );
  vector<Point2f>center_b_1( contours_b_1.size() );
  vector<Point2f>center_g_1( contours_g_1.size() );
  vector<float>radius_r_1( contours_r_1.size() );
  vector<float>radius_b_1( contours_b_1.size() );
  vector<float>radius_g_1( contours_g_1.size() );
  for( size_t i = 0; i < contours_r_1.size(); i++ ){
  approxPolyDP( contours_r_1[i], contours_r_poly_1[i], 3, true );
  minEnclosingCircle( contours_r_poly_1[i], center_r_1[i], radius_r_1[i] );
  }
  for( size_t i = 0; i < contours_b_1.size(); i++ ){
  approxPolyDP( contours_b_1[i], contours_b_poly_1[i], 3, true );
  minEnclosingCircle( contours_b_poly_1[i], center_b_1[i], radius_b_1[i] );
  }
  for( size_t i = 0; i < contours_g_1.size(); i++ ){
  approxPolyDP( contours_g_1[i], contours_g_poly_1[i], 3, true );
  minEnclosingCircle( contours_g_poly_1[i], center_g_1[i], radius_g_1[i] );
  }
  Point2f one_1,two_1;
  float x1_1, x2_1, y1_1, y2_1;
  float r1_1, r2_1;
  float l_1;

  //Red Ball
  size_t contour_r_1= contours_r_1.size();
  vector<Vec3f> slope_r_1;
  vector<Vec3f> base_r_1;
  vector<float>camera1_r_depth;


      for(size_t j=0; j<contour_r_1; j++){
          for(size_t k=0; k<contour_r_1; k++){
              one_1 = center_r_1[j]; two_1 = center_r_1[k];
              r1_1 = radius_r_1[j]; r2_1 = radius_r_1[k];

              x1_1 = one_1.x; y1_1 = one_1.y;
              x2_1 = two_1.x; y2_1 = two_1.y;
              l_1 = sqrt((x1_1-x2_1)*(x1_1-x2_1)+(y1_1-y2_1)*(y1_1-y2_1));

              if(r1_1+r2_1>l_1){
                  if(r1_1>r2_1){
                  radius_r_1.erase(radius_r_1.begin()+k);
                  center_r_1.erase(center_r_1.begin()+k);
                  contours_r_1.erase(contours_r_1.begin()+k);
                  contour_r_1--;
                  if(j>k){
                      j--;
                      k--;
                  }
                  else
                  {
  k-- ;
  }
                  }
              }
          }
      }
  for( size_t i = 0; i< contours_r_1.size(); i++ ){
  if (radius_r_1[i] > iMin_tracking_ball_size){

  Scalar color = Scalar( 0, 0, 255);
  drawContours( hsv_frame_1_red_canny, contours_r_poly_1, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  vector<float> ball_position_r_1;
  ball_position_r_1 = pixel2point_1(center_r_1[i], radius_r_1[i]);
  float ball_position_r_1_float[3];
  ball_position_r_1_float[0]=ball_position_r_1[0];
  ball_position_r_1_float[1]=ball_position_r_1[1];
  ball_position_r_1_float[2]=ball_position_r_1[2];
  Mat ball_position_r_mat_1 = Mat(3,1, CV_32F, ball_position_r_1_float);
  Mat ball_position_r_camera_1 = Rotation_matrix*ball_position_r_mat_1+Transfer_matrix;
  Mat d_r_1 = Camera_1_center - ball_position_r_camera_1;
  Mat p_r_1 = Transfer_matrix;

  float isx_1 = ball_position_r_1[0];
  float isy_1 = ball_position_r_1[1];
  float isz_1 = ball_position_r_1[2];
  float id    = sqrt(isx_1*isx_1+isy_1*isy_1+isz_1*isz_1);
  string sx_1 = floatToString(isx_1);
  string sy_1 = floatToString(isy_1);
  string sz_1 = floatToString(isz_1);
  string d    = floatToString(id);
  text =d+sz_1+"Red Ball:"+sx_1+","+sy_1+","+sz_1;
  putText(result_1, text, center_r_1[i],2,1,Scalar(0,0,255),2);
  circle( result_1, center_r_1[i], (int)radius_r_1[i], color, 2, 8, 0 );

  Vec3f r_s_1;
  r_s_1[0]=d_r_1.at<float>(0,0);
  r_s_1[1]=d_r_1.at<float>(1,0);
  r_s_1[2]=d_r_1.at<float>(2,0);
  slope_r_1.push_back(r_s_1);
  Vec3f r_b_1;
  r_b_1[0]=p_r_1.at<float>(0,0);
  r_b_1[1]=p_r_1.at<float>(1,0);
  r_b_1[2]=p_r_1.at<float>(2,0);
  base_r_1.push_back(r_b_1);
  camera1_r_depth.push_back(id);
  }

  else
  {
      Vec3f r_s_1;
      r_s_1[0]=0;
      r_s_1[1]=0;
      r_s_1[2]=0;
      slope_r_1.push_back(r_s_1);
      Vec3f r_b_1;
      r_b_1[0]=0;
      r_b_1[1]=0;
      r_b_1[2]=0;
      base_r_1.push_back(r_b_1);
  }
  }

  //for(int i=0;i<slope_r_1.size();i++)
  //{
  //    cout<<i<<"_r1_slope:"<<slope_r_1[i]<<endl;
  //    cout<<i<<"_r1_base:"<<base_r_1[i]<<endl;
  //}
  //Blue Ball

  size_t contour_b_1= contours_b_1.size();
  vector<Vec3f> slope_b_1;
  vector<Vec3f> base_b_1;
  vector<float>camera1_b_depth;

      for(size_t j=0; j<contour_b_1; j++){
          for(size_t k=0; k<contour_b_1; k++){
              one_1 = center_b_1[j]; two_1 = center_b_1[k];
              r1_1 = radius_b_1[j]; r2_1 = radius_b_1[k];

              x1_1 = one_1.x; y1_1 = one_1.y;
              x2_1 = two_1.x; y2_1 = two_1.y;
              l_1 = sqrt((x1_1-x2_1)*(x1_1-x2_1)+(y1_1-y2_1)*(y1_1-y2_1));

              if(r1_1+r2_1>l_1){
                  if(r1_1>r2_1){
                  radius_b_1.erase(radius_b_1.begin()+k);
                  center_b_1.erase(center_b_1.begin()+k);
                  contours_b_1.erase(contours_b_1.begin()+k);
                  contour_b_1--;
                  if(j>k){
                      j--;
                      k--;
                  }
                  else
                  {
  k-- ;
  }
                  }
              }
          }
      }
  for( size_t i = 0; i< contours_b_1.size(); i++ ){
  if (radius_b_1[i] > iMin_tracking_ball_size){

  Scalar color = Scalar( 255, 0, 0);
  drawContours( hsv_frame_1_blue_canny, contours_b_poly_1, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  vector<float> ball_position_b_1;
  ball_position_b_1 = pixel2point_1(center_b_1[i], radius_b_1[i]);
  float ball_position_b_1_float[3];
  ball_position_b_1_float[0]=ball_position_b_1[0];
  ball_position_b_1_float[1]=ball_position_b_1[1];
  ball_position_b_1_float[2]=ball_position_b_1[2];
  Mat ball_position_b_mat_1 = Mat(3,1, CV_32F, ball_position_b_1_float);
  Mat ball_position_b_camera_1 = Rotation_matrix*ball_position_b_mat_1+Transfer_matrix;
  Mat d_b_1 = Camera_1_center - ball_position_b_camera_1;
  Mat p_b_1 = Transfer_matrix;

  float isx_1 = ball_position_b_1[0];
  float isy_1 = ball_position_b_1[1];
  float isz_1 = ball_position_b_1[2];
  float id    = sqrt(isx_1*isx_1+isy_1*isy_1+isz_1*isz_1);
  string sx_1 = floatToString(isx_1);
  string sy_1 = floatToString(isy_1);
  string sz_1 = floatToString(isz_1);
  string d    = floatToString(id);
  text =d+sz_1+"Blue Ball:"+sx_1+","+sy_1+","+sz_1;
  putText(result_1, text, center_b_1[i],2,1,Scalar(255,0,0),2);
  circle( result_1, center_b_1[i], (int)radius_b_1[i], color, 2, 8, 0 );

  Vec3f b_s_1;
  b_s_1[0]=d_b_1.at<float>(0,0);
  b_s_1[1]=d_b_1.at<float>(1,0);
  b_s_1[2]=d_b_1.at<float>(2,0);
  slope_b_1.push_back(b_s_1);
  Vec3f b_b_1;
  b_b_1[0]=p_b_1.at<float>(0,0);
  b_b_1[1]=p_b_1.at<float>(1,0);
  b_b_1[2]=p_b_1.at<float>(2,0);
  base_b_1.push_back(b_b_1);
  camera1_b_depth.push_back((id));
  }

  else
  {
      Vec3f b_s_1;
      b_s_1[0]=0;
      b_s_1[1]=0;
      b_s_1[2]=0;
      slope_b_1.push_back(b_s_1);
      Vec3f b_b_1;
      b_b_1[0]=0;
      b_b_1[1]=0;
      b_b_1[2]=0;
      base_b_1.push_back(b_b_1);
  }
  }

  //for(int i=0;i<slope_b_1.size();i++)
  //{
  //    cout<<i<<"_b1_slope:"<<slope_b_1[i]<<endl;
  //    cout<<i<<"_b1_base:"<<base_b_1[i]<<endl;
  //}
  //Green Ball
  size_t contour_g_1= contours_g_1.size();
  vector<Vec3f> slope_g_1;
  vector<Vec3f> base_g_1;
  vector<float>camera1_g_depth;


      for(size_t j=0; j<contour_g_1; j++){
          for(size_t k=0; k<contour_g_1; k++){
              one_1 = center_g_1[j]; two_1 = center_g_1[k];
              r1_1 = radius_g_1[j]; r2_1 = radius_g_1[k];

              x1_1 = one_1.x; y1_1 = one_1.y;
              x2_1 = two_1.x; y2_1 = two_1.y;
              l_1 = sqrt((x1_1-x2_1)*(x1_1-x2_1)+(y1_1-y2_1)*(y1_1-y2_1));

              if(r1_1+r2_1>l_1){
                  if(r1_1>r2_1){
                  radius_g_1.erase(radius_g_1.begin()+k);
                  center_g_1.erase(center_g_1.begin()+k);
                  contours_g_1.erase(contours_g_1.begin()+k);
                  contour_g_1--;
                  if(j>k){
                      j--;
                      k--;
                  }
                  else
                  {
  k-- ;
  }
                  }
              }
          }
      }
  for( size_t i = 0; i< contours_g_1.size(); i++ ){
  if (radius_g_1[i] > iMin_tracking_ball_size){

  Scalar color = Scalar( 0, 255, 0);
  drawContours( hsv_frame_1_green_canny, contours_g_poly_1, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  vector<float> ball_position_g_1;
  ball_position_g_1 = pixel2point_1(center_g_1[i], radius_g_1[i]);
  float ball_position_g_1_float[3];
  ball_position_g_1_float[0]=ball_position_g_1[0];
  ball_position_g_1_float[1]=ball_position_g_1[1];
  ball_position_g_1_float[2]=ball_position_g_1[2];
  Mat ball_position_g_mat_1 = Mat(3,1, CV_32F, ball_position_g_1_float);
  Mat ball_position_g_camera_1 = Rotation_matrix*ball_position_g_mat_1+Transfer_matrix;
  Mat d_g_1 = Camera_1_center - ball_position_g_camera_1;
  Mat p_g_1 = Transfer_matrix;

  float isx_1 = ball_position_g_1[0];
  float isy_1 = ball_position_g_1[1];
  float isz_1 = ball_position_g_1[2];
  float id    = sqrt(isx_1*isx_1+isy_1*isy_1+isz_1*isz_1);
  string sx_1 = floatToString(isx_1);
  string sy_1 = floatToString(isy_1);
  string sz_1 = floatToString(isz_1);
  string d    = floatToString(id);
  text =d+sz_1+"Green Ball:"+sx_1+","+sy_1+","+sz_1;
  putText(result_1, text, center_g_1[i],2,1,Scalar(0,255,0),2);
  circle( result_1, center_g_1[i], (int)radius_g_1[i], color, 2, 8, 0 );

  Vec3f g_s_1;
  g_s_1[0]=d_g_1.at<float>(0,0);
  g_s_1[1]=d_g_1.at<float>(1,0);
  g_s_1[2]=d_g_1.at<float>(2,0);
  slope_g_1.push_back(g_s_1);
  Vec3f g_b_1;
  g_b_1[0]=p_g_1.at<float>(0,0);
  g_b_1[1]=p_g_1.at<float>(1,0);
  g_b_1[2]=p_g_1.at<float>(2,0);
  base_g_1.push_back(g_b_1);
  camera1_g_depth.push_back(id);
  }

  else
  {
      Vec3f g_s_1;
      g_s_1[0]=0;
      g_s_1[1]=0;
      g_s_1[2]=0;
      slope_g_1.push_back(g_s_1);
      Vec3f g_b_1;
      g_b_1[0]=0;
      g_b_1[1]=0;
      g_b_1[2]=0;
      base_g_1.push_back(g_b_1);
  }
  }

  //for(int i=0;i<slope_g_1.size();i++)
  //{
  //    cout<<i<<"_g1_slope:"<<slope_g_1[i]<<endl;
  //    cout<<i<<"_g1_base:"<<base_g_1[i]<<endl;
  //}
  cout<<endl;

  //Camera Two
  //cout<<"Camera2_line_info"<<endl;
  GaussianBlur(hsv_frame_2_red, hsv_frame_2_red_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_2_blue, hsv_frame_2_blue_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_2_green, hsv_frame_2_green_blur, cv::Size(9, 9), 2, 2);
  Canny(hsv_frame_2_red_blur, hsv_frame_2_red_canny, lowThreshold_r*ratio_r, kernel_size_r);
  Canny(hsv_frame_2_blue_blur, hsv_frame_2_blue_canny, lowThreshold_b*ratio_b, kernel_size_b);
  Canny(hsv_frame_2_green_blur, hsv_frame_2_green_canny, lowThreshold_g*ratio_g, kernel_size_g);
  findContours(hsv_frame_2_red_canny, contours_r_2, hierarchy_r_2, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(hsv_frame_2_blue_canny, contours_b_2, hierarchy_b_2, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(hsv_frame_2_green_canny, contours_g_2, hierarchy_g_2, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<vector<Point> > contours_r_poly_2( contours_r_2.size() );
  vector<vector<Point> > contours_b_poly_2( contours_b_2.size() );
  vector<vector<Point> > contours_g_poly_2( contours_g_2.size() );
  vector<Point2f>center_r_2( contours_r_2.size() );
  vector<Point2f>center_b_2( contours_b_2.size() );
  vector<Point2f>center_g_2( contours_g_2.size() );
  vector<float>radius_r_2( contours_r_2.size() );
  vector<float>radius_b_2( contours_b_2.size() );
  vector<float>radius_g_2( contours_g_2.size() );
  for( size_t i = 0; i < contours_r_2.size(); i++ ){
  approxPolyDP( contours_r_2[i], contours_r_poly_2[i], 3, true );
  minEnclosingCircle( contours_r_poly_2[i], center_r_2[i], radius_r_2[i] );
  }
  for( size_t i = 0; i < contours_b_2.size(); i++ ){
  approxPolyDP( contours_b_2[i], contours_b_poly_2[i], 3, true );
  minEnclosingCircle( contours_b_poly_2[i], center_b_2[i], radius_b_2[i] );
  }
  for( size_t i = 0; i < contours_g_2.size(); i++ ){
  approxPolyDP( contours_g_2[i], contours_g_poly_2[i], 3, true );
  minEnclosingCircle( contours_g_poly_2[i], center_g_2[i], radius_g_2[i] );
  }

  Point2f one_2,two_2;
  float x1_2, x2_2, y1_2, y2_2;
  float r1_2, r2_2;
  float l_2;

  //Red ball
  size_t contour_r_2= contours_r_2.size();
  vector<Vec3f>slope_r_2;
  vector<Vec3f>base_r_2;
  vector<float>camera2_r_depth;

      for(size_t j=0; j<contour_r_2; j++){
          for(size_t k=0; k<contour_r_2; k++){
              one_2 = center_r_2[j]; two_2 = center_r_2[k];
              r1_2 = radius_r_2[j]; r2_2 = radius_r_2[k];

              x1_2 = one_2.x; y1_2 = one_2.y;
              x2_2 = two_2.x; y2_2 = two_2.y;
              l_2 = sqrt((x1_2-x2_2)*(x1_2-x2_2)+(y1_2-y2_2)*(y1_2-y2_2));

              if(r1_2+r2_2>l_2){
                  if(r1_2>r2_2){
                  radius_r_2.erase(radius_r_2.begin()+k);
                  center_r_2.erase(center_r_2.begin()+k);
                  contours_r_2.erase(contours_r_2.begin()+k);
                  contour_r_2--;
                  if(j>k){
                      j--;
                      k--;
                  }
                  else
                  {
  k-- ;
  }
                  }
              }
          }
      }

vector<Vec3f> camera2_red;
vector<Vec3f> camera2_blue;
vector<Vec3f> camera2_green;

      for( size_t i = 0; i< contours_r_2.size(); i++ ){
      if (radius_r_2[i] > iMin_tracking_ball_size){

  Scalar color = Scalar( 0, 0, 255);
  drawContours( hsv_frame_2_red_canny, contours_r_poly_2, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  vector<float> ball_position_r_2;
  ball_position_r_2 = pixel2point_2(center_r_2[i], radius_r_2[i]);
  float ball_position_r_2_float[3];
  ball_position_r_2_float[0]=ball_position_r_2[0];
  ball_position_r_2_float[1]=ball_position_r_2[1];
  ball_position_r_2_float[2]=ball_position_r_2[2];
  Vec3f value;
  value[0]=ball_position_r_2[0];
  value[1]=ball_position_r_2[1];
  value[2]=ball_position_r_2[2];
 camera2_red.push_back(value);
  Mat ball_position_r_mat_2 = Mat(3,1, CV_32F, ball_position_r_2_float);

  Mat ball_position_r_camera_2 = ball_position_r_mat_2;
  Mat d_r_2 = Camera_2_center - ball_position_r_camera_2;
  Mat p_r_2 = Camera_2_center;

  float isx_2 = ball_position_r_2[0];
  float isy_2 = ball_position_r_2[1];
  float isz_2 = ball_position_r_2[2];
  float id    = sqrt(isx_2*isx_2+isy_2*isy_2+isz_2*isz_2);
  string sx_2 = floatToString(isx_2);
  string sy_2 = floatToString(isy_2);
  string sz_2 = floatToString(isz_2);
  string d = floatToString(id);
  text =d+sz_2+"Red Ball:"+sx_2+","+sy_2+","+sz_2;
  putText(result_2, text, center_r_2[i],2,1,Scalar(0,0,255),2);
  circle( result_2, center_r_2[i], (int)radius_r_2[i], color, 2, 8, 0 );
  Vec3f r_s_2;
  r_s_2[0]=d_r_2.at<float>(0,0);
  r_s_2[1]=d_r_2.at<float>(1,0);
  r_s_2[2]=d_r_2.at<float>(2,0);
  slope_r_2.push_back(r_s_2);

  Vec3f r_b_2;
  r_b_2[0]=p_r_2.at<float>(0,0);
  r_b_2[1]=p_r_2.at<float>(1,0);
  r_b_2[2]=p_r_2.at<float>(2,0);
  base_r_2.push_back(r_b_2);
  camera2_r_depth.push_back((id));
  }
  else
  {
      Vec3f r_s_2;
      r_s_2[0]=0;
      r_s_2[1]=0;
      r_s_2[2]=0;
      slope_r_2.push_back(r_s_2);
      Vec3f r_b_2;
      r_b_2[0]=0;
      r_b_2[1]=0;
      r_b_2[2]=0;
      base_r_2.push_back(r_b_2);
      Vec3f value;
      value[0]=0;
      value[1]=0;
      value[2]=0;
      camera2_red.push_back(value);
  }
  }


      //for(int i=0;i<slope_r_2.size();i++)
      //{
      //    cout<<i<<"_r2_slope:"<<slope_r_2[i]<<endl;
      //    cout<<i<<"_r2_base:"<<base_r_2[i]<<endl;
      //}
  //Blue Ball
  size_t contour_b_2= contours_b_2.size();
  vector<Vec3f> slope_b_2;
  vector<Vec3f> base_b_2;
  vector<float>camera2_b_depth;

      for(size_t j=0; j<contour_b_2; j++){
          for(size_t k=0; k<contour_b_2; k++){
              one_2 = center_b_2[j]; two_2 = center_b_2[k];
              r1_2 = radius_b_2[j]; r2_2 = radius_b_2[k];

              x1_2 = one_2.x; y1_2 = one_2.y;
              x2_2 = two_2.x; y2_2 = two_2.y;
              l_2 = sqrt((x1_2-x2_2)*(x1_2-x2_2)+(y1_2-y2_2)*(y1_2-y2_2));

              if(r1_2+r2_2>l_2){
                  if(r1_2>r2_2){
                  radius_b_2.erase(radius_b_2.begin()+k);
                  center_b_2.erase(center_b_2.begin()+k);
                  contours_b_2.erase(contours_b_2.begin()+k);
                  contour_b_2--;
                  if(j>k){
                      j--;
                      k--;
                  }
                  else
                  {
  k-- ;
  }
                  }
              }
          }
      }
      for( size_t i = 0; i< contours_b_2.size(); i++ ){
      if (radius_b_2[i] > iMin_tracking_ball_size){
  Scalar color = Scalar( 255, 0,0);
  drawContours( hsv_frame_2_blue_canny, contours_b_poly_2, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  vector<float> ball_position_b_2;
  ball_position_b_2 = pixel2point_2(center_b_2[i], radius_b_2[i]);
  float ball_position_b_2_float[3];
  ball_position_b_2_float[0]=ball_position_b_2[0];
  ball_position_b_2_float[1]=ball_position_b_2[1];
  ball_position_b_2_float[2]=ball_position_b_2[2];
  Vec3f value;
  value[0]=ball_position_b_2[0];
  value[1]=ball_position_b_2[1];
  value[2]=ball_position_b_2[2];
  camera2_blue.push_back(value);
  Mat ball_position_b_mat_2 = Mat(3,1, CV_32F, ball_position_b_2_float);
  Mat ball_position_b_camera_2 = ball_position_b_mat_2;
  Mat d_b_2 = Camera_2_center - ball_position_b_camera_2;
  Mat p_b_2 = Camera_2_center;

  float isx_1 = ball_position_b_2[0];
  float isy_1 = ball_position_b_2[1];
  float isz_1 = ball_position_b_2[2];
  float id    = sqrt(isx_1*isx_1+isy_1*isy_1+isz_1*isz_1);
  string sx_1 = floatToString(isx_1);
  string sy_1 = floatToString(isy_1);
  string sz_1 = floatToString(isz_1);
  string d    = floatToString(id);
  text =d+sz_1+"Blue Ball:"+sx_1+","+sy_1+","+sz_1;
  putText(result_2, text, center_b_2[i],2,1,Scalar(255,0,0),2);
  circle( result_2, center_b_2[i], (int)radius_b_2[i], color, 2, 8, 0 );

  Vec3f b_s_2;
  b_s_2[0]=d_b_2.at<float>(0,0);
  b_s_2[1]=d_b_2.at<float>(1,0);
  b_s_2[2]=d_b_2.at<float>(2,0);
  slope_b_2.push_back(b_s_2);
  Vec3f b_b_2;
  b_b_2[0]=p_b_2.at<float>(0,0);
  b_b_2[1]=p_b_2.at<float>(1,0);
  b_b_2[2]=p_b_2.at<float>(2,0);
  base_b_2.push_back(b_b_2);
  camera2_b_depth.push_back(id);
  }

  else
  {
      Vec3f b_s_2;
      b_s_2[0]=0;
      b_s_2[1]=0;
      b_s_2[2]=0;
      slope_b_2.push_back(b_s_2);
      Vec3f b_b_2;
      b_b_2[0]=0;
      b_b_2[1]=0;
      b_b_2[2]=0;
      base_b_2.push_back(b_b_2);
      Vec3f value;
      value[0]=0;
      value[1]=0;
      value[2]=0;
      camera2_blue.push_back(value);
  }
  }

  //for(int i=0;i<slope_b_2.size();i++)
  //{
  //    cout<<i<<"_b2_slope:"<<slope_b_2[i]<<endl;
  //    cout<<i<<"_b2_base:"<<base_b_2[i]<<endl;
  //}
  //Green Ball
  size_t contour_g_2= contours_g_2.size();
  vector<Vec3f> slope_g_2;
  vector<Vec3f> base_g_2;
  vector<float>camera2_g_depth;


      for(size_t j=0; j<contour_g_2; j++){
          for(size_t k=0; k<contour_g_2; k++){
              one_2 = center_g_2[j]; two_2 = center_g_2[k];
              r1_2 = radius_g_2[j]; r2_2 = radius_g_2[k];

              x1_2 = one_2.x; y1_2 = one_2.y;
              x2_2 = two_2.x; y2_2 = two_2.y;
              l_2 = sqrt((x1_2-x2_2)*(x1_2-x2_2)+(y1_2-y2_2)*(y1_2-y2_2));

              if(r1_2+r2_2>l_2){
                  if(r1_2>r2_2){
                  radius_g_2.erase(radius_g_2.begin()+k);
                  center_g_2.erase(center_g_2.begin()+k);
                  contours_g_2.erase(contours_g_2.begin()+k);
                  contour_g_2--;
                  if(j>k){
                      j--;
                      k--;
                  }
                  else
                  {
  k-- ;
  }
                  }
              }
          }
      }
      for( size_t i = 0; i< contours_g_2.size(); i++ ){
      if (radius_g_2[i] > iMin_tracking_ball_size){
  Scalar color = Scalar( 0, 255, 0);
  drawContours( hsv_frame_2_green_canny, contours_g_poly_2, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  vector<float> ball_position_g_2;
  ball_position_g_2 = pixel2point_2(center_g_2[i], radius_g_2[i]);
  float ball_position_g_2_float[3];
  ball_position_g_2_float[0]=ball_position_g_2[0];
  ball_position_g_2_float[1]=ball_position_g_2[1];
  ball_position_g_2_float[2]=ball_position_g_2[2];
  Vec3f value;
  value[0]=ball_position_g_2[0];
  value[1]=ball_position_g_2[1];
  value[2]=ball_position_g_2[2];
  camera2_green.push_back(value);
  Mat ball_position_g_mat_2 = Mat(3,1, CV_32F, ball_position_g_2_float);
  Mat ball_position_g_camera_2 = ball_position_g_mat_2;
  Mat d_g_2 = Camera_2_center - ball_position_g_camera_2;
  Mat p_g_2 = Camera_2_center;

  float isx_1 = ball_position_g_2[0];
  float isy_1 = ball_position_g_2[1];
  float isz_1 = ball_position_g_2[2];
  float id    = sqrt(isx_1*isx_1+isy_1*isy_1+isz_1*isz_1);
  string sx_1 = floatToString(isx_1);
  string sy_1 = floatToString(isy_1);
  string sz_1 = floatToString(isz_1);
  string d    = floatToString(id);
  text =d+sz_1+"Green Ball:"+sx_1+","+sy_1+","+sz_1;
  putText(result_2, text, center_g_2[i],2,1,Scalar(0,255,0),2);
  circle( result_2, center_g_2[i], (int)radius_g_2[i], color, 2, 8, 0 );

  Vec3f g_s_2;
  g_s_2[0]=d_g_2.at<float>(0,0);
  g_s_2[1]=d_g_2.at<float>(1,0);
  g_s_2[2]=d_g_2.at<float>(2,0);
  slope_g_2.push_back(g_s_2);
  Vec3f g_b_2;
  g_b_2[0]=p_g_2.at<float>(0,0);
  g_b_2[1]=p_g_2.at<float>(1,0);
  g_b_2[2]=p_g_2.at<float>(2,0);
  base_g_2.push_back(g_b_2);
  camera2_g_depth.push_back(id);
  }

  else
  {
      Vec3f g_s_2;
      g_s_2[0]=0;
      g_s_2[1]=0;
      g_s_2[2]=0;
      slope_g_2.push_back(g_s_2);
      Vec3f g_b_2;
      g_b_2[0]=0;
      g_b_2[1]=0;
      g_b_2[2]=0;
      base_g_2.push_back(g_b_2);
      Vec3f value;
      value[0]=0;
      value[1]=0;
      value[2]=0;
      camera2_green.push_back(value);
  }
  }

  //for(int i=0;i<slope_g_2.size();i++)
  //{
  //    cout<<i<<"_g2_slope:"<<slope_g_2[i]<<endl;
  //    cout<<i<<"_g2_base:"<<base_g_2[i]<<endl;
  //}
  cout<<endl;

  //line matching
   //Red ball
   cout<<"line_matching_red"<<endl;
  int red_camera_1=slope_r_1.size();
  int red_camera_2=slope_r_2.size();
  cout<<"Camera1_red_ball_count:"<<red_camera_1<<endl;
  cout<<"Camera2_red_ball_count:"<<red_camera_2<<endl;

  vector<float> red_line_distance;
  vector<Vec3f> red_line_midpoint;
  vector<int> matching_line_number_r;

  if(red_camera_1<red_camera_2){
  for(int i=0;i<slope_r_1.size();i++)
  {
    vector<float> distance;
    vector<Vec3f> midpoint;
    vector<int> matching_number;

    for(int j=0;j<slope_r_2.size();j++)
      {
         float slope_r_1_float[3];
         slope_r_1_float[0]=(slope_r_1[i])[0];
         slope_r_1_float[1]=(slope_r_1[i])[1];
         slope_r_1_float[2]=(slope_r_1[i])[2];
         float slope_r_2_float[3];
         slope_r_2_float[0]=(slope_r_2[j])[0];
         slope_r_2_float[1]=(slope_r_2[j])[1];
         slope_r_2_float[2]=(slope_r_2[j])[2];
         float base_r_1_float[3];
         base_r_1_float[0]=(base_r_1[i])[0];
         base_r_1_float[1]=(base_r_1[i])[1];
         base_r_1_float[2]=(base_r_1[i])[2];
         float base_r_2_float[3];
         base_r_2_float[0]=(base_r_2[j])[0];
         base_r_2_float[1]=(base_r_2[j])[1];
         base_r_2_float[2]=(base_r_2[j])[2];

         Mat d1= Mat(3, 1, CV_32FC1, slope_r_1_float);
         Mat p1= Mat(3, 1, CV_32FC1, base_r_1_float);
         Mat d2= Mat(3, 1, CV_32FC1, slope_r_2_float);
         Mat p2= Mat(3, 1, CV_32FC1, base_r_2_float);
         Mat n=d1.cross(d2);
         Mat n2=d2.cross(n);
         Mat n1=d1.cross(n);
         Mat s=n/sqrt(n.dot(n));
         float k=s.dot(p1-p2);
         float d=sqrt(k*k);
         Mat c1 = p1 + (((p2-p1).dot(n2))/d1.dot(n2))*d1;
         Mat c2 = p2 + (((p1-p2).dot(n1))/d2.dot(n1))*d2;
         Mat c=(c1+c2)/2;
         Vec3f mp;
         float depth = sqrt(c.dot(c));
         mp[0]=c.at<float>(0,0);
         mp[1]=c.at<float>(1,0);
         mp[2]=c.at<float>(2,0);

         if(mp[2]>0 && depth > camera2_r_depth[j]-0.1)
         {

             distance.push_back(d);
             midpoint.push_back(mp);
             matching_number.push_back(j);
         }

      }
    cout<<i<<endl;
    if(distance.size()!=0){
    float min = distance[0];
    Vec3f CP = midpoint[0];
    int pick=matching_number[0];
    for(int k=0; k<distance.size();k++)
      {
        cout<<"distance between two lines:"<<distance[k]<<endl;
       if(distance[k]<=min && distance[k]<0.02 )
         {
             min=distance[k];
             CP=midpoint[k];
             pick =matching_number[k];
         }
      }
    cout << "pick,min:" << pick <<","<<min<< endl;
    matching_line_number_r.push_back(pick);
    red_line_distance.push_back(min);
    red_line_midpoint.push_back(CP);
      }

  }

  }

  else{
      for(int i=0;i<slope_r_2.size();i++)
      {
        vector<float> distance;
        vector<Vec3f> midpoint;
        vector<int> matching_number;
        for(int j=0;j<slope_r_1.size();j++)
          {
             float slope_r_1_float[3];
             slope_r_1_float[0]=(slope_r_1[j])[0];
             slope_r_1_float[1]=(slope_r_1[j])[1];
             slope_r_1_float[2]=(slope_r_1[j])[2];
             float slope_r_2_float[3];
             slope_r_2_float[0]=(slope_r_2[i])[0];
             slope_r_2_float[1]=(slope_r_2[i])[1];
             slope_r_2_float[2]=(slope_r_2[i])[2];
             float base_r_1_float[3];
             base_r_1_float[0]=(base_r_1[j])[0];
             base_r_1_float[1]=(base_r_1[j])[1];
             base_r_1_float[2]=(base_r_1[j])[2];
             float base_r_2_float[3];
             base_r_2_float[0]=(base_r_2[i])[0];
             base_r_2_float[1]=(base_r_2[i])[1];
             base_r_2_float[2]=(base_r_2[i])[2];

             Mat d1= Mat(3, 1, CV_32FC1, slope_r_1_float);
             Mat p1= Mat(3, 1, CV_32FC1, base_r_1_float);
             Mat d2= Mat(3, 1, CV_32FC1, slope_r_2_float);
             Mat p2= Mat(3, 1, CV_32FC1, base_r_2_float);
             Mat n=d1.cross(d2);
             Mat n2=d2.cross(n);
             Mat n1=d1.cross(n);
             Mat s=n/sqrt(n.dot(n));
             float k=s.dot(p1-p2);
             float d=sqrt(k*k);
             Mat c1 = p1 + (((p2-p1).dot(n2))/d1.dot(n2))*d1;
             Mat c2 = p2 + (((p1-p2).dot(n1))/d2.dot(n1))*d2;
             Mat c=(c1+c2)/2;
             float depth = sqrt(c.dot(c));
             Vec3f mp;

             mp[0]=c.at<float>(0,0);
             mp[1]=c.at<float>(1,0);
             mp[2]=c.at<float>(2,0);

             if(mp[2]>0 && depth > camera2_r_depth[i]-0.1)
              {

                 distance.push_back(d);
                 midpoint.push_back(mp);
                 matching_number.push_back(j);
              }
          }
        cout << i << endl;
        if(distance.size()!=0)
        {
        float min = distance[0];
        Vec3f CP = midpoint[0];
        int pick=matching_number[0] ;
        for(int k=0; k<distance.size();k++)
          {
            cout<<"distance between two lines:"<<distance[k]<<endl;
            if(distance[k]<=min && distance[k]<0.02 )
             {
                 min=distance[k];
                 CP=midpoint[k];
                 pick=matching_number[k];
             }
        }
        cout << "pick,min:" << pick <<","<<min<< endl;
        matching_line_number_r.push_back(pick);
        red_line_distance.push_back(min);
        red_line_midpoint.push_back(CP);
        }
    }
  }
//cout<<endl;
  //cout<<"stereo_matching_red"<<endl;
  float pi=3.141592;
  float degree=19.9;
  float translation_data[3];
  translation_data[0]=Transfer_matrix_data[0]/2;
  translation_data[1]=0;
  translation_data[2]=-0.3;
  float error_data[3]={0,-0.02,0};
  Mat error = Mat(3,1, CV_32F, error_data);
  Mat t = Mat(3,1, CV_32F, translation_data);
  Mat translation =t+error;
  float rotation_data[9]={1,0,0,0,-sin((degree*pi)/180),cos((degree*pi)/180),0,-cos((degree*pi)/180),-sin((degree*pi)/180)};
  Mat rotation = Mat(3, 3, CV_32F, rotation_data);
  float cx,cy,cz,fx,fy,fz,r;

  if(red_camera_1<red_camera_2){
  for(int l=0;l<red_line_midpoint.size();l++){

      Scalar color = Scalar( 0, 0, 255);
      float r_x = red_line_midpoint[l][0];
      float r_y = red_line_midpoint[l][1];
      float r_z = red_line_midpoint[l][2];
      float r_d = sqrt(r_x*r_x+r_y*r_y+r_z*r_z);
      float box[3] = {r_x,r_y,r_z};
      Mat Non_real_axis=Mat(3, 1, CV_32F, box);
      Mat real_axis = rotation*Non_real_axis-translation;
      fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
      fy = real_axis.at<float>(0,1);
      fz = real_axis.at<float>(0,2);
      fx =  roundf(1000*fx)/1000;
      fy =  roundf(1000*fy)/1000;
      fz =  roundf(1000*fz)/1000;
      string x = floatToString(fx);
      string y = floatToString(fy);
      string z = floatToString(fz);

      string m = floatToString(matching_line_number_r[l]);
      text =x+","+y;
      putText(stereo_1, text, center_r_1[l],2,1,Scalar(0,0,255),2);
      circle( stereo_1, center_r_1[l], (int)5, color, 2, 8, 0 );
      putText(stereo_2, text, center_r_2[matching_line_number_r[l]],2,1,Scalar(0,0,255),2);
      circle( stereo_2, center_r_2[matching_line_number_r[l]], (int)5, color, 2, 8, 0 );
      //cout<<"line_matching:(" <<l<<","<< matching_line_number_r[l]<<")"<< endl;
      //cout<<"red_line_centerpoint=" << red_line_midpoint[l]<< endl;
      //cout<<"stereo_depth:"<<r_d<<endl;
  }
  }
  else
  {
       for(int l=0;l<red_line_midpoint.size();l++){

           Scalar color = Scalar(0,0,255);
           float r_x = red_line_midpoint[l][0];
           float r_y = red_line_midpoint[l][1];
           float r_z = red_line_midpoint[l][2];
           float r_d = sqrt(r_x*r_x+r_y*r_y+r_z*r_z);
           float box[3] = {r_x,r_y,r_z};
           Mat Non_real_axis=Mat(3, 1, CV_32F, box);
           Mat real_axis = rotation*Non_real_axis-translation;
           fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
           fy = real_axis.at<float>(0,1);
           fz = real_axis.at<float>(0,2);
           fx =  roundf(1000*fx)/1000;
           fy =  roundf(1000*fy)/1000;
           fz =  roundf(1000*fz)/1000;
           string x = floatToString(fx);
           string y = floatToString(fy);
           string z = floatToString(fz);
           string m = floatToString(matching_line_number_r[l]);
           text =x+","+y;
           putText(stereo_1, text, center_r_1[matching_line_number_r[l]],2,1,Scalar(0,0,255),2);
           circle( stereo_1, center_r_1[matching_line_number_r[l]], (int)5, color, 2, 8, 0 );
           putText(stereo_2, text, center_r_2[l],2,1,Scalar(0,0,255),2);
           circle( stereo_2, center_r_2[l], (int)5, color, 2, 8, 0 );
           //cout<<"line_matching:(" <<matching_line_number_r[l]<<","<< l<<")"<< endl;
           //cout<<"red_line_centerpoint=" << red_line_midpoint[l]<< endl;
           //cout<<"matching_depth:"<<r_d<<endl;
       }

  }
  cout<<endl;
  cout<<"line_matching_blue"<<endl;
  //Blue ball
  int blue_camera_1=slope_b_1.size();
  int blue_camera_2=slope_b_2.size();
  cout<<"blue_ball_count1:"<<blue_camera_1<<endl;
  cout<<"blue_ball_count2:"<<blue_camera_2<<endl;

  vector<float> blue_line_distance;
  vector<Vec3f> blue_line_midpoint;
  vector<int> matching_line_number_b;
  if(blue_camera_1<blue_camera_2){
  for(int i=0;i<slope_b_1.size();i++)
  {
   vector<float> distance;
   vector<Vec3f> midpoint;
   vector<int> matching_number;
   for(int j=0;j<slope_b_2.size();j++)
     {
        float slope_b_1_float[3];
        slope_b_1_float[0]=(slope_b_1[i])[0];
        slope_b_1_float[1]=(slope_b_1[i])[1];
        slope_b_1_float[2]=(slope_b_1[i])[2];
        float slope_b_2_float[3];
        slope_b_2_float[0]=(slope_b_2[j])[0];
        slope_b_2_float[1]=(slope_b_2[j])[1];
        slope_b_2_float[2]=(slope_b_2[j])[2];
        float base_b_1_float[3];
        base_b_1_float[0]=(base_b_1[i])[0];
        base_b_1_float[1]=(base_b_1[i])[1];
        base_b_1_float[2]=(base_b_1[i])[2];
        float base_b_2_float[3];
        base_b_2_float[0]=(base_b_2[j])[0];
        base_b_2_float[1]=(base_b_2[j])[1];
        base_b_2_float[2]=(base_b_2[j])[2];

        Mat d1= Mat(3, 1, CV_32FC1, slope_b_1_float);
        Mat p1= Mat(3, 1, CV_32FC1, base_b_1_float);
        Mat d2= Mat(3, 1, CV_32FC1, slope_b_2_float);
        Mat p2= Mat(3, 1, CV_32FC1, base_b_2_float);
        Mat n=d1.cross(d2);
        Mat n2=d2.cross(n);
        Mat n1=d1.cross(n);
        Mat s=n/sqrt(n.dot(n));
        float k=s.dot(p1-p2);
        float d=sqrt(k*k);
        Mat c1 = p1 + (((p2-p1).dot(n2))/d1.dot(n2))*d1;
        Mat c2 = p2 + (((p1-p2).dot(n1))/d2.dot(n1))*d2;
        Mat c=(c1+c2)/2;
        Vec3f mp;
        float depth = sqrt(c.dot(c));
        mp[0]=c.at<float>(0,0);
        mp[1]=c.at<float>(1,0);
        mp[2]=c.at<float>(2,0);
        if(mp[2]>0 && depth > camera2_b_depth[j]-0.1)
        {
            distance.push_back(d);
            midpoint.push_back(mp);
            matching_number.push_back(j);
        }
     }
   cout << i << endl;
   if(distance.size()!=0){
   float min = distance[0];
   Vec3f CP = midpoint[0];
   int pick=matching_number[0];
   for(int k=0; k<distance.size();k++)
     {
      cout<<"distance between two lines:"<<distance[k]<<endl;
      if(distance[k]<=min && distance[k]<0.02 )
        {
            min=distance[k];
            CP=midpoint[k];
            pick = matching_number[k];
        }
     }
   cout << "pick,min:" << pick<<","<<min<<endl;
  matching_line_number_b.push_back(pick);
  blue_line_distance.push_back(min);
  blue_line_midpoint.push_back(CP);
   }
  }

}

  else{
     for(int i=0;i<slope_b_2.size();i++)
     {
       vector<float> distance;
       vector<Vec3f> midpoint;
       vector<int> matching_number;
       for(int j=0;j<slope_b_1.size();j++)
         {
            float slope_b_1_float[3];
            slope_b_1_float[0]=(slope_b_1[j])[0];
            slope_b_1_float[1]=(slope_b_1[j])[1];
            slope_b_1_float[2]=(slope_b_1[j])[2];
            float slope_b_2_float[3];
            slope_b_2_float[0]=(slope_b_2[i])[0];
            slope_b_2_float[1]=(slope_b_2[i])[1];
            slope_b_2_float[2]=(slope_b_2[i])[2];
            float base_b_1_float[3];
            base_b_1_float[0]=(base_b_1[j])[0];
            base_b_1_float[1]=(base_b_1[j])[1];
            base_b_1_float[2]=(base_b_1[j])[2];
            float base_b_2_float[3];
            base_b_2_float[0]=(base_b_2[i])[0];
            base_b_2_float[1]=(base_b_2[i])[1];
            base_b_2_float[2]=(base_b_2[i])[2];

            Mat d1= Mat(3, 1, CV_32FC1, slope_b_1_float);
            Mat p1= Mat(3, 1, CV_32FC1, base_b_1_float);
            Mat d2= Mat(3, 1, CV_32FC1, slope_b_2_float);
            Mat p2= Mat(3, 1, CV_32FC1, base_b_2_float);
            Mat n=d1.cross(d2);
            Mat n2=d2.cross(n);
            Mat n1=d1.cross(n);
            Mat s=n/sqrt(n.dot(n));
            float k=s.dot(p1-p2);
            float d=sqrt(k*k);
            Mat c1 = p1 + (((p2-p1).dot(n2))/d1.dot(n2))*d1;
            Mat c2 = p2 + (((p1-p2).dot(n1))/d2.dot(n1))*d2;
            Mat c=(c1+c2)/2;
            Vec3f mp;
            float depth = sqrt(c.dot(c));
            mp[0]=c.at<float>(0,0);
            mp[1]=c.at<float>(1,0);
            mp[2]=c.at<float>(2,0);
            if(mp[2]>0 && depth > camera2_b_depth[i]-0.1)
             {
                distance.push_back(d);
                midpoint.push_back(mp);
                matching_number.push_back(j);
             }



         }
            cout << i << endl;
            if(distance.size()!=0)
            {
       float min = distance[0];
       Vec3f CP = midpoint[0];
       int pick=matching_number[0] ;
       for(int k=0; k<distance.size();k++)
         {
           cout<<"distance between two lines:"<<distance[k]<<endl;
           if(distance[k]<=min && distance[k]<0.02 )
            {
                min=distance[k];
                CP=midpoint[k];
                pick=matching_number[k];
            }
       }
       cout << "pick,min:" << pick<<","<<min<<endl;
       matching_line_number_b.push_back(pick);
       blue_line_distance.push_back(min);
       blue_line_midpoint.push_back(CP);
      }
  }
}
cout<<endl;
  //cout<<"stereo_matching_blue"<<endl;
  if(blue_camera_1<blue_camera_2){
  for(int l=0;l<blue_line_midpoint.size();l++){

     Scalar color = Scalar( 255, 0, 0);
     float b_x = blue_line_midpoint[l][0];
     float b_y = blue_line_midpoint[l][1];
     float b_z = blue_line_midpoint[l][2];
     float b_d = sqrt(b_x*b_x+b_y*b_y+b_z*b_z);
     float box[3] = {b_x,b_y,b_z};
     Mat Non_real_axis=Mat(3, 1, CV_32F, box);
     Mat real_axis = rotation*Non_real_axis-translation;
     fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
     fy = real_axis.at<float>(0,1);
     fz = real_axis.at<float>(0,2);
     fx =  roundf(1000*fx)/1000;
     fy =  roundf(1000*fy)/1000;
     fz =  roundf(1000*fz)/1000;
     string x = floatToString(fx);
     string y = floatToString(fy);
     string z = floatToString(fz);

     string m = floatToString(matching_line_number_b[l]);
     text =x+","+y;
     putText(stereo_1, text, center_b_1[l],2,1,Scalar(255,0,0),2);
     circle( stereo_1, center_b_1[l], (int)5, color, 2, 8, 0 );
     putText(stereo_2, text, center_b_2[matching_line_number_b[l]],2,1,Scalar(255,0,0),2);
     circle( stereo_2, center_b_2[matching_line_number_b[l]], (int)5, color, 2, 8, 0 );
     //cout<<"line_matching:(" <<l<<","<<matching_line_number_b[l]<<")"<< endl;
     //cout<<"blue_line_centerpoint=" << blue_line_midpoint[l]<< endl;
     //cout<<"matching_depth:"<<b_d<<endl;
  }
  }
  else
  {
      for(int l=0;l<blue_line_midpoint.size();l++){

          Scalar color = Scalar( 255, 0, 0);
          float b_x = blue_line_midpoint[l][0];
          float b_y = blue_line_midpoint[l][1];
          float b_z = blue_line_midpoint[l][2];
          float b_d = sqrt(b_x*b_x+b_y*b_y+b_z*b_z);
          float box[3] = {b_x,b_y,b_z};
          Mat Non_real_axis=Mat(3, 1, CV_32F, box);
          Mat real_axis = rotation*Non_real_axis-translation;
          fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
          fy = real_axis.at<float>(0,1);
          fz = real_axis.at<float>(0,2);
          fx =  roundf(1000*fx)/1000;
          fy =  roundf(1000*fy)/1000;
          fz =  roundf(1000*fz)/1000;
          string x = floatToString(fx);
          string y = floatToString(fy);
          string z = floatToString(fz);

          string m = floatToString(matching_line_number_b[l]);
          text =x+","+y;
          putText(stereo_1, text, center_b_1[matching_line_number_b[l]],2,1,Scalar(255,0,0),2);
          circle( stereo_1, center_b_1[matching_line_number_b[l]], (int)5, color, 2, 8, 0 );
          putText(stereo_2, text, center_b_2[l],2,1,Scalar(255,0,0),2);
          circle( stereo_2, center_b_2[l], (int)5, color, 2, 8, 0 );
          //cout<<"line_matching:(" <<matching_line_number_b[l]<<","<< l<<")"<< endl;
          //cout<<"blue_line_centerpoint=" << blue_line_midpoint[l]<< endl;
          //cout<<"matching_depth:"<<b_d<<endl;
      }

  }
  cout<<endl;
cout<<"line_matching_green"<<endl;
  //Green ball
  int green_camera_1=slope_g_1.size();
  int green_camera_2=slope_g_2.size();
  cout<<"green_ball_count1:"<<green_camera_1<<endl;
  cout<<"green_ball-count2:"<<green_camera_2<<endl;

  vector<float> green_line_distance;
  vector<Vec3f> green_line_midpoint;
  vector<int> matching_line_number_g;
  if(green_camera_1<green_camera_2){
  for(int i=0;i<slope_g_1.size();i++)
  {
   vector<float> distance;
   vector<Vec3f> midpoint;
   vector<int> matching_number;
   for(int j=0;j<slope_g_2.size();j++)
     {
        float slope_g_1_float[3];
        slope_g_1_float[0]=(slope_g_1[i])[0];
        slope_g_1_float[1]=(slope_g_1[i])[1];
        slope_g_1_float[2]=(slope_g_1[i])[2];
        float slope_g_2_float[3];
        slope_g_2_float[0]=(slope_g_2[j])[0];
        slope_g_2_float[1]=(slope_g_2[j])[1];
        slope_g_2_float[2]=(slope_g_2[j])[2];
        float base_g_1_float[3];
        base_g_1_float[0]=(base_g_1[i])[0];
        base_g_1_float[1]=(base_g_1[i])[1];
        base_g_1_float[2]=(base_g_1[i])[2];
        float base_g_2_float[3];
        base_g_2_float[0]=(base_g_2[j])[0];
        base_g_2_float[1]=(base_g_2[j])[1];
        base_g_2_float[2]=(base_g_2[j])[2];

        Mat d1= Mat(3, 1, CV_32FC1, slope_g_1_float);
        Mat p1= Mat(3, 1, CV_32FC1, base_g_1_float);
        Mat d2= Mat(3, 1, CV_32FC1, slope_g_2_float);
        Mat p2= Mat(3, 1, CV_32FC1, base_g_2_float);
        Mat n=d1.cross(d2);
        Mat n2=d2.cross(n);
        Mat n1=d1.cross(n);
        Mat s=n/sqrt(n.dot(n));
        float k=s.dot(p1-p2);
        float d=sqrt(k*k);
        Mat c1 = p1 + (((p2-p1).dot(n2))/d1.dot(n2))*d1;
        Mat c2 = p2 + (((p1-p2).dot(n1))/d2.dot(n1))*d2;
        Mat c=(c1+c2)/2;
        Vec3f mp;
        float depth = sqrt(c.dot(c));
        mp[0]=c.at<float>(0,0);
        mp[1]=c.at<float>(1,0);
        mp[2]=c.at<float>(2,0);

        if(mp[2]>0 && depth > camera2_g_depth[j]-0.1)
        {
            distance.push_back(d);
            midpoint.push_back(mp);
            matching_number.push_back(j);
        }


     }
   cout << i << endl;
   if(distance.size()!=0){
   float min = distance[0];
   Vec3f CP = midpoint[0];
   int pick=matching_number[0];
   for(int k=0; k<distance.size();k++)
     {
      cout<<"distance between two lines:"<<distance[k]<<endl;
      if(distance[k]<=min && distance[k]<0.02 )
        {
            min=distance[k];
            CP=midpoint[k];
            pick = matching_number[k];
        }
     }
   cout << "pick,min:" << pick<<","<<min<<endl;
   matching_line_number_g.push_back(pick);
   green_line_distance.push_back(min);
   green_line_midpoint.push_back(CP);
     }

  }
}

  else{
     for(int i=0;i<slope_g_2.size();i++)
     {
       vector<float> distance;
       vector<Vec3f> midpoint;
       vector<int> matching_number;
       for(int j=0;j<slope_g_1.size();j++)
         {
            float slope_g_1_float[3];
            slope_g_1_float[0]=(slope_g_1[j])[0];
            slope_g_1_float[1]=(slope_g_1[j])[1];
            slope_g_1_float[2]=(slope_g_1[j])[2];
            float slope_g_2_float[3];
            slope_g_2_float[0]=(slope_g_2[i])[0];
            slope_g_2_float[1]=(slope_g_2[i])[1];
            slope_g_2_float[2]=(slope_g_2[i])[2];
            float base_g_1_float[3];
            base_g_1_float[0]=(base_g_1[j])[0];
            base_g_1_float[1]=(base_g_1[j])[1];
            base_g_1_float[2]=(base_g_1[j])[2];
            float base_g_2_float[3];
            base_g_2_float[0]=(base_g_2[i])[0];
            base_g_2_float[1]=(base_g_2[i])[1];
            base_g_2_float[2]=(base_g_2[i])[2];

            Mat d1= Mat(3, 1, CV_32FC1, slope_g_1_float);
            Mat p1= Mat(3, 1, CV_32FC1, base_g_1_float);
            Mat d2= Mat(3, 1, CV_32FC1, slope_g_2_float);
            Mat p2= Mat(3, 1, CV_32FC1, base_g_2_float);
            Mat n=d1.cross(d2);
            Mat n2=d2.cross(n);
            Mat n1=d1.cross(n);
            Mat s=n/sqrt(n.dot(n));
            float k=s.dot(p1-p2);
            float d=sqrt(k*k);
            Mat c1 = p1 + (((p2-p1).dot(n2))/d1.dot(n2))*d1;
            Mat c2 = p2 + (((p1-p2).dot(n1))/d2.dot(n1))*d2;
            Mat c=(c1+c2)/2;
            Vec3f mp;
            float depth = sqrt(c.dot(c));
            mp[0]=c.at<float>(0,0);
            mp[1]=c.at<float>(1,0);
            mp[2]=c.at<float>(2,0);

            if(mp[2]>0 && depth > camera2_g_depth[i]-0.1)
             {

                distance.push_back(d);
                midpoint.push_back(mp);
                matching_number.push_back(j);
             }

         }
        cout<<i<<endl;
       if(distance.size()!=0)
       {
       float min = distance[0];
       Vec3f CP = midpoint[0];
       int pick=matching_number[0] ;
       for(int k=0; k<distance.size();k++)
         {
           cout<<"distance between two lines:"<<distance[k]<<endl;
           if(distance[k]<=min && distance[k]<0.02 )
            {
                min=distance[k];
                CP=midpoint[k];
                pick=matching_number[k];
            }
         }
       cout << "pick,min:" << pick<<","<<min<<endl;
       matching_line_number_g.push_back(pick);
       green_line_distance.push_back(min);
       green_line_midpoint.push_back(CP);
       }
     }
  }
  cout<<endl;
  //cout<<"stereo_matching_green"<<endl;
  if(green_camera_1<green_camera_2){
  for(int l=0;l<green_line_midpoint.size();l++){

     Scalar color = Scalar( 0, 255, 0);
     float g_x = green_line_midpoint[l][0];
     float g_y = green_line_midpoint[l][1];
     float g_z = green_line_midpoint[l][2];
     float g_d = sqrt(g_x*g_x+g_y*g_y+g_z*g_z);
     float box[3] = {g_x,g_y,g_z};
     Mat Non_real_axis=Mat(3, 1, CV_32F, box);
     Mat real_axis = rotation*Non_real_axis-translation;
     fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
     fy = real_axis.at<float>(0,1);
     fz = real_axis.at<float>(0,2);
     fx =  roundf(1000*fx)/1000;
     fy =  roundf(1000*fy)/1000;
     fz =  roundf(1000*fz)/1000;
     string x = floatToString(fx);
     string y = floatToString(fy);
     string z = floatToString(fz);

     string m = floatToString(matching_line_number_g[l]);
     text =x+","+y;


     putText(stereo_1, text, center_g_1[l],2,1,Scalar(0,255,0),2);
     circle( stereo_1, center_g_1[l], (int)5, color, 2, 8, 0 );
     putText(stereo_2, text, center_g_2[matching_line_number_g[l]],2,1,Scalar(0,255,0),2);
     circle( stereo_2, center_g_2[matching_line_number_g[l]], (int)5, color, 2, 8, 0 );
     //cout<<"line_matching:(" <<l<<","<<matching_line_number_g[l]<<")"<< endl;
     //cout<<"green_line_centerpoint=" << green_line_midpoint[l]<< endl;
     //cout<<"matching_depth:"<<g_d<<endl;
  }
  }
  else
  {

      for(int l=0;l<green_line_midpoint.size();l++){

          Scalar color = Scalar( 0, 255, 0);
          float g_x = green_line_midpoint[l][0];
          float g_y = green_line_midpoint[l][1];
          float g_z = green_line_midpoint[l][2];
          float g_d = sqrt(g_x*g_x+g_y*g_y+g_z*g_z);
          float box[3] = {g_x,g_y,g_z};
          Mat Non_real_axis=Mat(3, 1, CV_32F, box);
          Mat real_axis = rotation*Non_real_axis-translation;
          fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
          fy = real_axis.at<float>(0,1);
          fz = real_axis.at<float>(0,2);
          fx =  roundf(1000*fx)/1000;
          fy =  roundf(1000*fy)/1000;
          fz =  roundf(1000*fz)/1000;
          string x = floatToString(fx);
          string y = floatToString(fy);
          string z = floatToString(fz);

          string m = floatToString(matching_line_number_g[l]);
          text =x+","+y;

          putText(stereo_1, text, center_g_1[matching_line_number_g[l]],2,1,Scalar(0,255,0),2);
          circle( stereo_1, center_g_1[matching_line_number_g[l]], (int)5, color, 2, 8, 0 );
          putText(stereo_2, text, center_g_2[l],2,1,Scalar(0,255,0),2);
          circle( stereo_2, center_g_2[l], (int)5, color, 2, 8, 0 );
          //cout<<"line_matching:(" <<matching_line_number_g[l]<<","<< l<<")"<< endl;
          //cout<<"green_line_centerpoint=" << green_line_midpoint[l]<< endl;
          //cout<<"matching_depth:"<<g_d<<endl;
      }
  }
  cout<<endl;
  cout<<"x,y from Frame"<<endl;

     core_msgs::ball_pos msg;  //create a message for ball positions

     msg.b_size = blue_line_midpoint.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msg.b_img_x.resize(blue_line_midpoint.size());  //adjust the size of array
     msg.b_img_y.resize(blue_line_midpoint.size());  //adjust the size of array
     msg.g_size = green_line_midpoint.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msg.g_img_x.resize(green_line_midpoint.size());  //adjust the size of array
     msg.g_img_y.resize(green_line_midpoint.size());  //adjust the size of array
     msg.r_size = red_line_midpoint.size();
     msg.r_img_x.resize(red_line_midpoint.size());
     msg.r_img_y.resize(red_line_midpoint.size());
     msg.r1=red_camera_1;
     msg.r2=red_camera_2;
     msg.g1=green_camera_1;
     msg.g2=green_camera_2;
     //msg.b_size = camera2_blue.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     //msg.b_img_x.resize(camera2_blue.size());  //adjust the size of array
     //msg.b_img_y.resize(camera2_blue.size());  //adjust the size of array
     //msg.g_size = camera2_green.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     //msg.g_img_x.resize(camera2_green.size());  //adjust the size of array
     //msg.g_img_y.resize(camera2_green.size());  //adjust the size of array
     //msg.r_size = camera2_red.size();
     //msg.r_img_x.resize(camera2_red.size());
     //msg.r_img_y.resize(camera2_red.size());
    visualization_msgs::Marker ball_list;  //declare marker
   	ball_list.header.frame_id = "base_link";  //set the frame
  	ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
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

   Vec3f b_params; //assign a memory to save the information of circles
   Vec3f g_params;
   Vec3f r_params;
   Vec3f d_params;

 for(int k=0;k<red_line_midpoint.size();k++)
     {

         r_params = red_line_midpoint[k];  //the information of k-th circle
         //r_params = camera2_red[k];
         cx=r_params[0];  //x position of k-th circle
         cy=r_params[1];  //y position
         cz=r_params[2];  //z position
         float box[3] = {cx,cy,cz};
         Mat Non_real_axis=Mat(3, 1, CV_32F, box);
         //r=cvRound(params[2]); //radius
         // 원 출력을 위한 원 중심 생성
         //Point center(cx,cy);  //declare a Point
         //circle(frame,center,r,Scalar(0,0,255),10); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
         Mat real_axis = rotation*Non_real_axis-translation;
         fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
         fy = real_axis.at<float>(0,1);
         fz = real_axis.at<float>(0,2);
         msg.r_img_x[k]=fx;  //input the x position of the ball to the message
         msg.r_img_y[k]=fy;

        geometry_msgs::Point p;
	      p.x = fx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
	      p.y = fy;
	      p.z = fz;
	      ball_list.points.push_back(p);

	      std_msgs::ColorRGBA c;
	      c.r = 1.0;  //set the color of the balls. You can set it respectively.
	      c.g = 0.0;
	      c.b = 0.0;
	      c.a = 1.0;
	      ball_list.colors.push_back(c);
       printf("red_ball_%d:(%f,%f,%f)\n",k,fx,fy,fz);
     }

  for(int k=0;k<blue_line_midpoint.size();k++)
       {
           b_params = blue_line_midpoint[k];  //the information of k-th circle
           //b_params = camera2_blue[k];

           cx=b_params[0];  //x position of k-th circle
           cy=b_params[1];  //y position
           cz=b_params[2];  //z position
           float box[3] = {cx,cy,cz};
           Mat Non_real_axis=Mat(3, 1, CV_32F, box);
           //r=cvRound(params[2]); //radius
           // 원 출력을 위한 원 중심 생성
           //Point center(cx,cy);  //declare a Point
           //circle(frame,center,r,Scalar(0,0,255),10); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
           Mat real_axis = rotation*Non_real_axis-translation;
           fx = real_axis.at<float>(0,0);    //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
           fy = real_axis.at<float>(0,1);
           fz = real_axis.at<float>(0,2);
           msg.b_img_x[k]=fx; //input the x position of the ball to the message
           msg.b_img_y[k]=fy;

  	      geometry_msgs::Point p;
  	      p.x = fx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
  	      p.y = fy;
  	      p.z = fz;
  	      ball_list.points.push_back(p);

  	      std_msgs::ColorRGBA c;
  	      c.r = 0.0;  //set the color of the balls. You can set it respectively.
  	      c.g = 0.0;
  	      c.b = 1.0;
  	      c.a = 1.0;
  	      ball_list.colors.push_back(c);
          printf("blue_ball_%d:(%f,%f,%f)\n",k,fx,fy,fz);
       }


       for(int k=0;k<green_line_midpoint.size();k++)
         {
             g_params = green_line_midpoint[k];  //the information of k-th circle
             //g_params = camera2_green[k];
             cx=g_params[0];  //x position of k-th circle
             cy=g_params[1];  //y position
             cz=g_params[2];  //z position
             float box[3] = {cx,cy,cz};
             Mat Non_real_axis=Mat(3, 1, CV_32F, box);
             //r=cvRound(params[2]); //radius
             // 원 출력을 위한 원 중심 생성
             //Point center(cx,cy);  //declare a Point
             //circle(frame,center,r,Scalar(0,0,255),10); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
             Mat real_axis = rotation*Non_real_axis-translation;
             fx = real_axis.at<float>(0,0);   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
             fy = real_axis.at<float>(0,1);
             fz = real_axis.at<float>(0,2);
             msg.g_img_x[k]=fx; //input the x position of the ball to the message
             msg.g_img_y[k]=fy;

            geometry_msgs::Point p;
            p.x = fx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
            p.y = fy;
            p.z = fz;
            ball_list.points.push_back(p);

            std_msgs::ColorRGBA c;
            c.r = 0.0;  //set the color of the balls. You can set it respectively.
            c.g = 1.0;
            c.b = 0.0;
            c.a = 1.0;
            ball_list.colors.push_back(c);
            printf("green_ball_%d:(%f,%f,%f)\n",k,fx,fy,fz);
         }
         cout<<endl;
  //cv::imshow("view", frame);  //show the image with a window
  //cv::waitKey(1);
  // Show the frames


  imshow("Object Detection_HSV_Red1",hsv_frame_1_red);
  imshow("Object Detection_HSV_Red2",hsv_frame_2_red);
  imshow("Object Detection_HSV_Blue1",hsv_frame_1_blue);
  imshow("Object Detection_HSV_Blue2",hsv_frame_2_blue);
  imshow("Object Detection_HSV_Green1",hsv_frame_1_green);
  imshow("Object Detection_HSV_Green2",hsv_frame_2_green);
  Mat stereo, Result,Final;
  cv::hconcat(stereo_1,stereo_2,stereo);
  cv::hconcat(result_1,result_2,Result);
  cv::vconcat(stereo,Result,Final);
  imshow("Final",Final);

     (char)waitKey(1)!='q';
     pub.publish(msg);  //publish a message
     pub_markers.publish(ball_list);  //publish a marker message
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

   if(msg->height==480&&buffer.size().width==640){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
	std::cout<<"resized"<<std::endl;
	cv::resize(buffer,buffer,cv::Size(1280,480));

}
   else{
	//do nothing!
   }

   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer

   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

   }

   ball_detect(); //proceed ball detection
}


int main(int argc, char **argv)
{

  namedWindow("Result1", WINDOW_NORMAL);
  namedWindow("Result2", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Red1", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Red2", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Blue1", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Blue2", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Green1", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Green2", WINDOW_NORMAL);
  namedWindow("Stereo_camera1", WINDOW_NORMAL);
  namedWindow("Stereo_camera2", WINDOW_NORMAL);
  namedWindow("Final",WINDOW_NORMAL);

  moveWindow("Object Detection_HSV_Red1", 50,370);
  moveWindow("Object Detection_HSV_Red2", 470,370);
  moveWindow("Object Detection_HSV_Blue1", 50,370);
  moveWindow("Object Detection_HSV_Blue2", 470,370);
  moveWindow("Object Detection_HSV_Green1", 50,370);
  moveWindow("Object Detection_HSV_Green2", 470,370);
  moveWindow("Result1", 50, 730);
  moveWindow("Result2", 470, 730);
  moveWindow("Stereo_camera1", 890, 730);
  moveWindow("Stereo_camera2", 1310, 730);

  // Trackbars to set thresholds for HSV values : Red ball
  createTrackbar("Low H","Object Detection_HSV_Red1", &low_h_r_1, 180, on_low_h_thresh_trackbar_red_1);
  createTrackbar("High H","Object Detection_HSV_Red1", &high_h_r_1, 180, on_high_h_thresh_trackbar_red_1);
  createTrackbar("Low H2","Object Detection_HSV_Red1", &low_h2_r_1, 180, on_low_h2_thresh_trackbar_red_1);
  createTrackbar("High H2","Object Detection_HSV_Red1", &high_h2_r_1, 180, on_high_h2_thresh_trackbar_red_1);
  createTrackbar("Low S","Object Detection_HSV_Red1", &low_s_r_1, 255, on_low_s_thresh_trackbar_red_1);
  createTrackbar("High S","Object Detection_HSV_Red1", &high_s_r_1, 255, on_high_s_thresh_trackbar_red_1);
  createTrackbar("Low V","Object Detection_HSV_Red1", &low_v_r_1, 255, on_low_v_thresh_trackbar_red_1);
  createTrackbar("High V","Object Detection_HSV_Red1", &high_v_r_1, 255, on_high_v_thresh_trackbar_red_1);

  createTrackbar("Low H","Object Detection_HSV_Red2", &low_h_r_2, 180, on_low_h_thresh_trackbar_red_2);
  createTrackbar("High H","Object Detection_HSV_Red2", &high_h_r_2, 180, on_high_h_thresh_trackbar_red_2);
  createTrackbar("Low H2","Object Detection_HSV_Red2", &low_h2_r_2, 180, on_low_h2_thresh_trackbar_red_2);
  createTrackbar("High H2","Object Detection_HSV_Red2", &high_h2_r_2, 180, on_high_h2_thresh_trackbar_red_2);
  createTrackbar("Low S","Object Detection_HSV_Red2", &low_s_r_2, 255, on_low_s_thresh_trackbar_red_2);
  createTrackbar("High S","Object Detection_HSV_Red2", &high_s_r_2, 255, on_high_s_thresh_trackbar_red_2);
  createTrackbar("Low V","Object Detection_HSV_Red2", &low_v_r_2, 255, on_low_v_thresh_trackbar_red_2);
  createTrackbar("High V","Object Detection_HSV_Red2", &high_v_r_2, 255, on_high_v_thresh_trackbar_red_2);

  // Trackbars to set thresholds for HSV values : Blue ball
  createTrackbar("Low H","Object Detection_HSV_Blue1", &low_h_b_1, 180, on_low_h_thresh_trackbar_blue_1);
  createTrackbar("High H","Object Detection_HSV_Blue1", &high_h_b_1, 180, on_high_h_thresh_trackbar_blue_1);
  createTrackbar("Low S","Object Detection_HSV_Blue1", &low_s_b_1, 255, on_low_s_thresh_trackbar_blue_1);
  createTrackbar("High S","Object Detection_HSV_Blue1", &high_s_b_1, 255, on_high_s_thresh_trackbar_blue_1);
  createTrackbar("Low V","Object Detection_HSV_Blue1", &low_v_b_1, 255, on_low_v_thresh_trackbar_blue_1);
  createTrackbar("High V","Object Detection_HSV_Blue1", &high_v_b_1, 255, on_high_v_thresh_trackbar_blue_1);

  createTrackbar("Low H","Object Detection_HSV_Blue2", &low_h_b_2, 180, on_low_h_thresh_trackbar_blue_2);
  createTrackbar("High H","Object Detection_HSV_Blue2", &high_h_b_2, 180, on_high_h_thresh_trackbar_blue_2);
  createTrackbar("Low S","Object Detection_HSV_Blue2", &low_s_b_2, 255, on_low_s_thresh_trackbar_blue_2);
  createTrackbar("High S","Object Detection_HSV_Blue2", &high_s_b_2, 255, on_high_s_thresh_trackbar_blue_2);
  createTrackbar("Low V","Object Detection_HSV_Blue2", &low_v_b_2, 255, on_low_v_thresh_trackbar_blue_2);
  createTrackbar("High V","Object Detection_HSV_Blue2", &high_v_b_2, 255, on_high_v_thresh_trackbar_blue_2);

  // Trackbars to set thresholds for HSV values : Green ball
  createTrackbar("Low H","Object Detection_HSV_Green1", &low_h_g_1, 180, on_low_h_thresh_trackbar_green_1);
  createTrackbar("High H","Object Detection_HSV_Green1", &high_h_g_1, 180, on_high_h_thresh_trackbar_green_1);
  createTrackbar("Low S","Object Detection_HSV_Green1", &low_s_g_1, 255, on_low_s_thresh_trackbar_green_1);
  createTrackbar("High S","Object Detection_HSV_Green1", &high_s_g_1, 255, on_high_s_thresh_trackbar_green_1);
  createTrackbar("Low V","Object Detection_HSV_Green1", &low_v_g_1, 255, on_low_v_thresh_trackbar_green_1);
  createTrackbar("High V","Object Detection_HSV_Green1", &high_v_g_1, 255, on_high_v_thresh_trackbar_green_1);

  createTrackbar("Low H","Object Detection_HSV_Green2", &low_h_g_2, 180, on_low_h_thresh_trackbar_green_2);
  createTrackbar("High H","Object Detection_HSV_Green2", &high_h_g_2, 180, on_high_h_thresh_trackbar_green_2);
  createTrackbar("Low S","Object Detection_HSV_Green2", &low_s_g_2, 255, on_low_s_thresh_trackbar_green_2);
  createTrackbar("High S","Object Detection_HSV_Green2", &high_s_g_2, 255, on_high_s_thresh_trackbar_green_2);
  createTrackbar("Low V","Object Detection_HSV_Green2", &low_v_g_2, 255, on_low_v_thresh_trackbar_green_2);
  createTrackbar("High V","Object Detection_HSV_Green2", &high_v_g_2, 255, on_high_v_thresh_trackbar_green_2);

   ros::init(argc, argv, "ball_detect_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber

   pub = nh.advertise<core_msgs::ball_pos>("/position", 100); //setting publisher
   pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

   ros::spin(); //spin.

   return 0;
}
