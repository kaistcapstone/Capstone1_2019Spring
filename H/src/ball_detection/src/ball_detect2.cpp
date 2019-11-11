#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;


ros::Publisher pub;
ros::Publisher pub_markers;


void morphOps(Mat &thresh);
vector<float> pixel2point(Point center, int radius);
string floatToString(float f);
string intToString(int n);


Mat buffer(320,240,CV_8UC1);
Mat frame, calibrated_frame, bgr_frame, hsv_frame,
hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue,
hsv_frame_red_blur, hsv_frame_blue_blur,
hsv_frame_red_canny, hsv_frame_blue_canny, result;
Mat edges;
String text, text1, text2;
vector<Vec4i> hierarchy_r;
vector<Vec4i> hierarchy_b;
vector<vector<Point> > contours_r;
vector<vector<Point> > contours_b;


//camera calibration data
float intrinsic_data[9] = {644.790478, 0, 295.692810, 0, 647.510977, 224.472524, 0, 0, 1};
float distortion_data[5] = {0.019196, -0.171574, 0.011015, -0.011519, 0};
Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

//inrage thresholds
int low_h2_r=153, high_h2_r=180;
int low_h_r=0, low_s_r=234, low_v_r=123;
int high_h_r=5, high_s_r=255, high_v_r=220;
int low_h_b=101, low_s_b=190, low_v_b=129;
int high_h_b=111, high_s_b=255, high_v_b=191;

//canny edge thresholds
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;

int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

//draw contour threshold
int iMin_tracking_ball_size = 10;

// pixet2point data
float fball_radius = 0.037; // meter
float camera_height = 0.38;



void ball_detect(){
  if(buffer.size().width==320){
    cv::resize(buffer, frame, cv::Size(640, 480));
  }
  else{
    frame = buffer;
  }

  undistort(frame, calibrated_frame, intrinsic, distCoeffs);
  result = calibrated_frame.clone();

  medianBlur(calibrated_frame, calibrated_frame, 3);

  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
  inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
  inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

  morphOps(hsv_frame_red);
  morphOps(hsv_frame_blue);

  GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);

  Canny(hsv_frame_red_blur,hsv_frame_red_canny, lowThreshold_r,lowThreshold_r*ratio_r, kernel_size_r);
  Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

  findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP,CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<vector<Point> > contours_r_poly( contours_r.size() );
  vector<vector<Point> > contours_b_poly( contours_b.size() );
  vector<Point2f>center_r( contours_r.size() );
  vector<Point2f>center_b( contours_b.size() );
  vector<Point2f>bc_r( contours_r.size() );
  vector<Point2f>bc_b( contours_b.size() );
  vector<Point2f>br_r( contours_r.size() );
  vector<Point2f>br_b( contours_r.size() );
  vector<float>radius_r( contours_r.size() );
  vector<float>radius_b( contours_b.size() );
  vector<Point2f>text_center_r1( contours_r.size() ), text_center_r2( contours_r.size() );
  vector<Point2f>text_center_b( contours_b.size() );
  for( size_t i = 0; i < contours_r.size(); i++ ){
    approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
    minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );

  }

  for( size_t i = 0; i < contours_b.size(); i++ ){
    approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
    minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i< contours_r.size(); i++ ){
    if (radius_r[i] > iMin_tracking_ball_size){
      Scalar color = Scalar( 0, 0, 255);
      drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );

      float px_r = {center_r[i].x};
      float py_r = {center_r[i].y};
      vector<float> ball_position_r;
      text_center_r1[i] = center_r[i]+Point2f(20,-30);
      text_center_r2[i] = center_r[i]+Point2f(20, 30);
      ball_position_r = pixel2point(center_r[i], radius_r[i]);
      float isx = ball_position_r[0];
      float isy = ball_position_r[1];
      float isz = ball_position_r[2];
      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);
      string sx_round = intToString(cvRound(isx*100));
      string sy_round = intToString(cvRound(isy*100));
      string sz_round = intToString(cvRound(isz*100));

      float bc_r_y = {33+423.9519594*exp(-0.0097096738*py_r)};
      float bc_r_x = {15*(px_r-320)/(-0.0000221681072401222*pow(bc_r_y,3)+0.01032654*pow(bc_r_y,2)-1.8187695951*bc_r_y+165.6656346749)};
      string bc_r_y_string = intToString(cvRound(bc_r_y));
      string bc_r_x_string = intToString(cvRound(bc_r_x));
      //vector<Point2f>bc_r[i] = {bc_r_y[i], bc_r_x[i]};
      //string bc_r_x_round[i] = floatToString(cvRound(bc_r_x[i]*100));
      //string bc_r_y_round[i] = floatToString(cvRound(bc_r_y[i]*100));
      float br_r_x = isx*100;
      float br_r_y = 100*(pow(pow(isz,2)+pow(isy,2)-pow(camera_height-fball_radius,2),0.5)-0.095);
      string br_r_y_string = intToString(cvRound(br_r_y));
      string br_r_x_string = intToString(cvRound(br_r_x));
      //string br_r_x_round[i] = floatToString(cvRound(br_r_x[i]*100));
      //string br_r_y_round[i] = floatToString(cvRound(br_r_y[i]*100));
      //text = sx_round + "," + sy_round + "," + sz_round;
      text1 = "bc:" + bc_r_x_string + "," + bc_r_y_string;
      text2 = "br:" + br_r_x_string + "," + br_r_y_string;
      putText(result, text1, text_center_r1[i],1,1.5,Scalar(0,255,0),2);
      putText(result, text2, text_center_r2[i],1,1.5,Scalar(0,255,0),2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
      //  cout<<"x"<<center_r[0]<<endl;
          }

  }

  for( size_t i = 0; i< contours_b.size(); i++ ){
    if(radius_b[i] > iMin_tracking_ball_size){
      Scalar color = Scalar( 255, 0, 0);
      drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );

      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      vector<float> ball_position_b;
      text_center_b[i] = center_b[i]+Point2f(20,-20);
      ball_position_b = pixel2point(center_b[i], radius_b[i]);
      float isx = ball_position_b[0];
      float isy = ball_position_b[1];
      float isz = ball_position_b[2];
      string sx = floatToString(isx*100);
      string sy = floatToString(isy*100);
      string sz = floatToString(isz*100);
      string sx_round = intToString(cvRound(isx*100));
      string sy_round = intToString(cvRound(isy*100));
      string sz_round = intToString(cvRound(isz*100));
      text = "";
      //putText(result, text, text_center_b[i],2,1.5,Scalar(0,255,0),2);
      //circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
    }
  }

  // //---------------------------------------------------------------------------------------
  //
  // vector<Vec3f> circles; //assign a memory to save the result of circle detection
  // vector<Vec3f> circlesb;
  //
  // HoughCircles(hsv_frame_red_canny,circles,HOUGH_GRADIENT, 1,50 , 300, 10, 3, 80); //proceed circle detection
  // HoughCircles(hsv_frame_blue_canny,circlesb,HOUGH_GRADIENT, 1, 50, 300, 10, 3, 80);//initvalue 3, 80 for minmaxrad and 50 for mindistbetadjballs
  // //second edit 1, 100, 200, 20, 20, 150
  // Vec3f params; //assign a memory to save the information of circles
  // float cx,cy,cz,r;//edited by minki at 05:00
  // //cout<<"circles.size="<<circles.size()<<endl;  //print the number of circles detected
  //
  // core_msgs::ball_position msg;  //create a message for ball positions
  // msg.size =circles.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  // msg.img_x.resize(circles.size());  //adjust the size of array
  // msg.img_y.resize(circles.size());  //adjust the size of array
  //
  // visualization_msgs::Marker ball_list;  //declare marker
  // ball_list.header.frame_id = "/camera_link";  //set the frame
  // ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
  // ball_list.ns = "balls";   //name of markers
  // ball_list.action = visualization_msgs::Marker::ADD;
  // ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
  // ball_list.pose.position.y=0;
  // ball_list.pose.position.z=0;
  // ball_list.pose.orientation.x=0;
  // ball_list.pose.orientation.y=0;
  // ball_list.pose.orientation.z=0;
  // ball_list.pose.orientation.w=1.0;
  //
  // ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
  // ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker
  //
  // double radius = 0.10;
  // ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
  // ball_list.scale.y=radius;
  // ball_list.scale.z=radius;
  //
  // for(int k=0;k<circles.size();k++){
  //   params = circles[k];  //the information of k-th circle
  //   cx=cvRound(params[0]);  //x position of k-th circle
  //   cy=cvRound(params[1]);  //y position
  //   r=cvRound(params[2]); //radius
  //   // 원 출력을 위한 원 중심 생성
  //   Point center(cx,cy);  //declare a Point
  //   string num =intToString(k+1);
  //   float ry = (-4.0*0.000001*cy*cy*cy+0.005*cy*cy-2.3996*cy+430.89)*0.95;
  //   float rx = 0.4;
  //   float rz = 0.5;
  //   string sx = floatToString(cx);//mo
  //   string sy = floatToString(ry);//mod, isy
  //   string sz = floatToString(cy);
  //
  //   //added 05:00 minki
  //   // vector<float> ball_position_r;
  //   // ball_position_r = pixel2point(center, r);
  //   // float isx = ball_position_r[0];
  //   // float isy = ball_position_r[1];
  //   // float isz = ball_position_r[2];
  //   // string sx = floatToString(cx);//mo
  //   // string sy = floatToString(cy);//mod, isy
  //   // string sz = floatToString(isz);
  //   text =  "R" + num;//modified
  //   // putText(calibrated_frame, text, center,2,1,Scalar(0,0,255),2);
  //   //circle(calibrated_frame,center,r,Scalar(0,0,255),3);
  //   // cx=isx;
  //   // cy=isy;
  //   // cz=isz;
  //   // //////////////////////////////////////////////////////////////
  //   //
  //   // //circle(calibrated_frame,center,r,Scalar(0,0,255),3); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
  //   // //cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
  //   // //cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);
  //   // //cout<<"cx="<<cx<<"cy="<<cy<<endl;
  //   //
  //   // cout<<"cz="<<cz<<endl;
  //   // msg.img_x[k]=cx;  //input the x position of the ball to the message
  //   // msg.img_y[k]=cy;
  //   //
  //   // geometry_msgs::Point p;
  //   // p.x = cx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
  //   // p.y = cy;
  //   // //p.z = 0.1;
  //   // p.z = cz; //edited by minki at 05:00
  //   // ball_list.points.push_back(p);
  //   //
  //   // std_msgs::ColorRGBA c;
  //   // c.r = 1.0;  //set the color of the balls. You can set it respectively.
  //   // c.g = 0.0;
  //   // c.b = 0.0;
  //   // c.a = 1.0;
  //   // ball_list.colors.push_back(c);
  //
  // }
  //
  // for(int k=0;k<circlesb.size();k++){
  //   params = circlesb[k];  //the information of k-th circle
  //   cx=cvRound(params[0]);  //x position of k-th circle
  //   cy=cvRound(params[1]);  //y position
  //   r=cvRound(params[2]); //radius
  //   // 원 출력을 위한 원 중심 생성
  //   Point center(cx,cy);  //declare a Point
  //   string num = intToString(k+1);
  //
  //   //  circle(calibrated_frame,center,r,Scalar(255,0 ,0),3); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
  //   text = "B" +num;
  //   //putText(calibrated_frame, text, center,2,1,Scalar(255, 0,0),2);
  //   vector<float> ball_position_r;
  //   ball_position_r = pixel2point(center, r);
  //   float isx = ball_position_r[0];
  //   float isy = ball_position_r[1];
  //   float isz = ball_position_r[2];
  //   string sx = floatToString(isx);
  //   string sy = floatToString(isy);
  //   string sz = floatToString(isz);
  //   // text = "B"+ num + ":" + sx + "," + sy + "," + sz;
  //   // putText(calibrated_frame, text, center,2,1,Scalar(255,0,0),2);
  //   //       cx=isx;
  //   //       cy=isy;
  //   //       cz=isz;
  //   //
  //   //       cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
  //   //       cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);
  //   //
  //   //       msg.img_x[k]=cx;  //input the x position of the ball to the message
  //   //       msg.img_y[k]=cy;
  //   //
  //   // geometry_msgs::Point p;
  //   // p.x = cx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
  //   // p.y = cy;
  //   // p.z = 0.1;
  //   // ball_list.points.push_back(p);
  //   //
  //   // std_msgs::ColorRGBA c;
  //   // c.r = 0.0;  //set the color of the balls. You can set it respectively.
  //   // c.g = 1.0;
  //   // c.b = 0.0;
  //   // c.a = 1.0;
  //   // ball_list.colors.push_back(c);
  // }

  //Trackbar
  void on_low_h_thresh_trackbar_red(int, void *);
  void on_high_h_thresh_trackbar_red(int, void *);
  void on_low_h2_thresh_trackbar_red(int, void *);
  void on_high_h2_thresh_trackbar_red(int, void *);
  void on_low_s_thresh_trackbar_red(int, void *);
  void on_high_s_thresh_trackbar_red(int, void *);
  void on_low_v_thresh_trackbar_red(int, void *);
  void on_high_v_thresh_trackbar_red(int, void *);
  void on_low_h_thresh_trackbar_blue(int, void *);
  void on_high_h_thresh_trackbar_blue(int, void *);
  void on_low_s_thresh_trackbar_blue(int, void *);
  void on_high_s_thresh_trackbar_blue(int, void *);
  void on_low_v_thresh_trackbar_blue(int, void *);
  void on_high_v_thresh_trackbar_blue(int, void *);
  void on_canny_edge_trackbar_red(int, void *);
  void on_canny_edge_trackbar_blue(int, void *);

  namedWindow("Video Capture", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);
  namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
  namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
  namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
  namedWindow("Result", WINDOW_NORMAL);
  moveWindow("Video Capture",50, 0);
  moveWindow("Object Detection_HSV_Red", 50,370);
  moveWindow("Object Detection_HSV_Blue",470,370);
  moveWindow("Canny Edge for Red Ball", 50,730);
  moveWindow("Canny Edge for Blue Ball", 470,730);
  moveWindow("Result", 470, 0);

  // Trackbars to set thresholds for HSV values : Red ball
  createTrackbar("Low H","Object Detection_HSV_Red", &low_h_r, 180, on_low_h_thresh_trackbar_red);
  createTrackbar("High H","Object Detection_HSV_Red", &high_h_r, 180, on_high_h_thresh_trackbar_red);
  createTrackbar("Low H2","Object Detection_HSV_Red", &low_h2_r, 180, on_low_h2_thresh_trackbar_red);
  createTrackbar("High H2","Object Detection_HSV_Red", &high_h2_r, 180, on_high_h2_thresh_trackbar_red);
  createTrackbar("Low S","Object Detection_HSV_Red", &low_s_r, 255, on_low_s_thresh_trackbar_red);
  createTrackbar("High S","Object Detection_HSV_Red", &high_s_r, 255, on_high_s_thresh_trackbar_red);
  createTrackbar("Low V","Object Detection_HSV_Red", &low_v_r, 255, on_low_v_thresh_trackbar_red);
  createTrackbar("High V","Object Detection_HSV_Red", &high_v_r, 255, on_high_v_thresh_trackbar_red);
  // Trackbars to set thresholds for HSV values : Blue ball
  createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180, on_low_h_thresh_trackbar_blue);
  createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180, on_high_h_thresh_trackbar_blue);
  createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255, on_low_s_thresh_trackbar_blue);
  createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255, on_high_s_thresh_trackbar_blue);
  createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255, on_low_v_thresh_trackbar_blue);
  createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255, on_high_v_thresh_trackbar_blue);
  // Trackbar to set parameter for Canny Edge
  createTrackbar("Min Threshold:","Canny Edge for Red Ball", &lowThreshold_r,100, on_canny_edge_trackbar_red);
  createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);

  cv::imshow("view", result);  //show the image with a window
  cv::imshow("Video Capture",calibrated_frame);
  cv::imshow("Object Detection_HSV_Red",hsv_frame_red);
  cv::imshow("Object Detection_HSV_Blue",hsv_frame_blue);
  cv::imshow("Canny Edge for Red Ball", hsv_frame_red_canny);
  cv::imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
  cv::imshow("Result", result);

  cv::waitKey(10);
  //pub.publish(msg);  //publish a message
  //pub_markers.publish(ball_list);  //publish a marker message



}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if(msg->height==480&&buffer.size().width==320){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
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
  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber

  pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
  pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

  ros::spin(); //spin.
  return 0;
}


// Set functions
void morphOps(Mat &thresh){
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
  erode(thresh,thresh,erodeElement);
  erode(thresh,thresh,erodeElement);
  dilate(thresh,thresh,dilateElement);
  dilate(thresh,thresh,dilateElement);
}

string floatToString(float f){
  ostringstream buffer;
  buffer << f;
  return buffer.str();
}

string intToString(int n){
  stringstream s;
  s << n;
  return s.str();
}

vector<float> pixel2point(Point center, int radius){
  vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = center.x;//.x;// .at(0);
  y = center.y;//.y;//
  u = (x-intrinsic_data[2])/intrinsic_data[0];
  v = (y-intrinsic_data[5])/intrinsic_data[4];
  Zc = (intrinsic_data[0]*fball_radius)/((float)radius) ;
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

// Trackbar for cannyedge
void on_canny_edge_trackbar_red(int, void *){
  setTrackbarPos("Min Threshold", "Canny Edge for Red Ball",lowThreshold_r);
}
void on_canny_edge_trackbar_blue(int, void *){
  setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball",lowThreshold_b);
}
