#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
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

Mat rframe, frame;

// Declaration of trackbar functions to set HSV colorspace's parameters
// void on_low_h_thresh_trackbar_red(int, void *);
// void on_high_h_thresh_trackbar_red(int, void *);
// void on_low_h2_thresh_trackbar_red(int, void *);
// void on_high_h2_thresh_trackbar_red(int, void *);
// void on_low_s_thresh_trackbar_red(int, void *);
// void on_high_s_thresh_trackbar_red(int, void *);
// void on_low_v_thresh_trackbar_red(int, void *);
// void on_high_v_thresh_trackbar_red(int, void *);
// int low_h2_r=160, high_h2_r=180;
// int low_h_r=0, low_s_r=150, low_v_r=100;
// int high_h_r=10, high_s_r=255, high_v_r=150;

void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);
int low_h_b=100, low_s_b=62, low_v_b=100; //low_S_b=75
int high_h_b=140, high_s_b=255, high_v_b=240; //high_v_b=150

void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);
int low_h_g=45, low_s_g=86, low_v_g=35;
int high_h_g=90, high_s_g=255, high_v_g=255;

// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);

// Declaration of trackbars function that set canny edge's parameters
// void on_canny_edge_trackbar_red(int, void *);
// int lowThreshold_r = 100;
// int ratio_r = 3;
// int kernel_size_r = 3;

void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

void on_canny_edge_trackbar_green(int, void *);
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

// Initialization of variable for dimension of the target
float fball_radius = 0.073 ; // ball diameter, unit: meter

// Initialization of variable for camera calibration paramters
Mat distCoeffs;
// 640 x 360
float intrinsic_data[9] = {466.384228, 0.000000, 308.807845,
0.000000, 464.875680, 173.215221,
0.000000, 0.000000, 1.000000
};

float distortion_data[5] = {0.021261, -0.090254, 0.000665, -0.002808, 0.000000};

// Initialization of variable for text drawing
double fontScale = 1;
int thickness = 1;
String text ;
int iMin_tracking_ball_size = 1;

// Mat buffer(240,320,CV_8UC1);
ros::Publisher pub;
void ball_detect(){
    Mat hsv_frame,
    hsv_frame_red, hsv_frame_blue, hsv_frame_green, hsv_frame_red2, hsv_frame_red1,
    hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_green_blur,
    hsv_frame_red_canny, hsv_frame_blue_canny, hsv_frame_green_canny,
    result;

    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);
    // vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    // vector<Vec4i> hierarchy_g;
    // vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;
    // vector<vector<Point> > contours_g;


	undistort(frame, calibrated_frame, intrinsic, distCoeffs);
	result = calibrated_frame.clone();
	medianBlur(calibrated_frame, calibrated_frame, 3);
	cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

	// Detect the object based on RGB and HSV Range Values
  // inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
  // inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
  inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
  // inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);

  // addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);
	// morphOps(hsv_frame_red);
	morphOps(hsv_frame_blue);
	// morphOps(hsv_frame_green);

	// GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
	GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
	// GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);

	// Canny(hsv_frame_red_blur,hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
	Canny(hsv_frame_blue_blur,hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
	// Canny(hsv_frame_green_blur,hsv_frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);

	// findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
	findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
	// findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

	// vector<vector<Point> > contours_r_poly( contours_r.size() );
	vector<vector<Point> > contours_b_poly( contours_b.size() );
	// vector<vector<Point> > contours_g_poly( contours_g.size() );

	// vector<Point2f>center_r( contours_r.size() );
	vector<Point2f>center_b( contours_b.size() );
	// vector<Point2f>center_g( contours_g.size() );

	// vector<float>radius_r( contours_r.size() );
	vector<float>radius_b( contours_b.size() );
	// vector<float>radius_g( contours_g.size() );

	for( size_t i = 0; i < contours_b.size(); i++ ){
		approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
		minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
	}

        float l, r1, r2;
        Point2f one, two;
        float x1, x2, y1, y2;

        size_t contour_b=contours_b.size();
        for (size_t i=0; i< contour_b; i++){
            if (radius_b[i] > iMin_tracking_ball_size){
                for (size_t j=0; j<contour_b; j++){
                    for (size_t k=0; k<contour_b; k++){
                        one=center_b[j];
                        two=center_b[k];
                        r1=radius_b[j];
                        r2=radius_b[k];

                        x1=one.x;
                        y1=one.y;
                        x2=two.x;
                        y2=two.y;
                        l=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

                        if (r1+r2>l){
                            if (r1>r2){
                                radius_b.erase(radius_b.begin()+k);
                                center_b.erase(center_b.begin()+k);
                                contours_b.erase(contours_b.begin()+k);
                                contour_b--;
                                j--;
                            }
                        }
                    }
                }
            }
        }

        // size_t contour_g=contours_g.size();
        // for (size_t i=0; i< contour_g; i++){
        //     if (radius_g[i] > iMin_tracking_ball_size){
        //         for (size_t j=0; j<contour_g; j++){
        //             for (size_t k=0; k<contour_g; k++){
        //                 one=center_g[j];
        //                 two=center_g[k];
        //                 r1=radius_g[j];
        //                 r2=radius_g[k];
        //
        //                 x1=one.x;
        //                 y1=one.y;
        //                 x2=two.x;
        //                 y2=two.y;
        //                 l=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        //
        //                 if (r1+r2>l){
        //                     if (r1>r2){
        //                         radius_g.erase(radius_g.begin()+k);
        //                         center_g.erase(center_g.begin()+k);
        //                         contours_g.erase(contours_g.begin()+k);
        //                         contour_g--;
        //                         j--;
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }

   core_msgs::ball_position msg;  //create a message for ball positions
   msg.size =contours_b.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
   msg.img_x.resize(contours_b.size());  //adjust the size of array
   msg.img_y.resize(contours_b.size());  //adjust the size of array

   // msg.size3 = contours_g.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
   // msg.img_x3.resize(contours_g.size());  //adjust the size of array
   // msg.img_y3.resize(contours_g.size());  //adjust the size of array

	for( size_t i = 0; i< contours_b.size(); i++ ){
		if(radius_b[i] > iMin_tracking_ball_size){
			Scalar color = Scalar( 255, 0, 0);
			vector<float> ball_position_b;
      ball_position_b = pixel2point(center_b[i], radius_b[i]);
			// drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			float isx = -(320-center_b[i].x);
			float isy = (360-center_b[i].y);
			// float isz = ball_position_b[2];
			string sx = floatToString(isx);
			string sy = floatToString(isy);
			// string sz = floatToString(isz);
      msg.img_x[i] = -(320-center_b[i].x);
      msg.img_y[i] = (360-center_b[i].y);

			text = "B:" + sx + "," + sy;
			putText(result, text, center_b[i],2,1,Scalar(0,255,0),2);
			circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
		}
	}

  cout<<"blue size : "<<contours_b.size()<<endl;

  // for( size_t i = 0; i< contours_g.size(); i++ ){
	// 	if(radius_g[i] > iMin_tracking_ball_size){
	// 		Scalar color = Scalar( 0, 255, 0);
	// 		vector<float> ball_position_g;
  //     ball_position_g = pixel2point(center_g[i], radius_g[i]);
	// 		drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	// 		float isx = -(320-center_g[i].x);
	// 		float isy = (360-center_g[i].y);
	// 		float isz = ball_position_g[2];
	// 		string sx = floatToString(isx);
	// 		string sy = floatToString(isy);
  //     string sz = floatToString(isz);
  //     // cout<<"green size"<<contours_g.size()<<endl;
  //     // cout<<"green center"<<center_g[i]<<endl;
  //                       msg.img_x3[i] = -(320-center_g[i].x);
  //                       msg.img_y3[i] = (360-center_g[i].y);
  //
	// 		text = "Green:" + sx  + "," + sy;
	// 		putText(result, text, center_g[i],2,1,Scalar(0,255,0),2);
	// 		circle( result, center_g[i], (int)radius_g[i], color, 2 , 8, 0 );
	// 	}
	// }

  // cv::waitKey(1);
	// imshow("Object Detection_HSV_Blue",hsv_frame_blue);
	// imshow("Result", result);

	pub.publish(msg);
}

int main(int argc, char **argv)
{
	VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
    // namedWindow("Result", WINDOW_NORMAL);
    // namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
    // moveWindow("Object Detection_HSV_Blue",470,370);
    // moveWindow("Result", 470, 0);
    // //
    // createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180, on_low_h_thresh_trackbar_blue);
    // createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180, on_high_h_thresh_trackbar_blue);
    // createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255, on_low_s_thresh_trackbar_blue);
    // createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255, on_high_s_thresh_trackbar_blue);
    // createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255, on_low_v_thresh_trackbar_blue);
    // createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255, on_high_v_thresh_trackbar_blue);

   ros::init(argc, argv, "ball_detect_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler

   pub = nh.advertise<core_msgs::ball_position>("/position123", 1000); //setting publisher

   int t=0;
   int i = 0;
   while (ros::ok()){
     cap>>frame;
     ball_detect();
     // printf("%d\n", i);
     // i++;
   }
   return 0;
}



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
    erode(thresh,thresh,erodeElement); //remove blur noise, shrinked ball size
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement); //original ball size
    dilate(thresh,thresh,dilateElement);
}
vector<float> pixel2point(Point center, int radius){
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
