#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "core_msgs/ball_position_back.h"

using namespace cv;
using namespace std;

Mat frame;

// Declaration of trackbar functions to set HSV colorspace's parameters
void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);
int low_h_g=45, low_s_g=80, low_v_g=70;
int high_h_g=90, high_s_g=255, high_v_g=255;

// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);

// Declaration of trackbars function that set canny edge's parameters
void on_canny_edge_trackbar_green(int, void *);
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

// Initialization of variable for dimension of the target
float fball_radius = 0.073 ; // ball diameter, unit: meter

// Initialization of variable for camera calibration paramters
Mat distCoeffs;
// 640 x 480

float intrinsic_data[9] = {466.384228, 0.000000, 308.807845,
0.000000, 464.875680, 173.215221,
0.000000, 0.000000, 1.000000
};

float distortion_data[5] = {0.021261, -0.090254, 0.000665, -0.002808, 0.000000};

// Initialization of variable for text drawing
double fontScale = 1;
int thickness = 1;
String text ;
int iMin_tracking_ball_size = 5;

// Mat buffer(240,320,CV_8UC1);
ros::Publisher pub;
void ball_detect(){
    Mat hsv_frame,
    hsv_frame_green,
    hsv_frame_green_blur,
    hsv_frame_green_canny,
    result;

    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);
    vector<Vec4i> hierarchy_g;
    vector<vector<Point> > contours_g;

	undistort(frame, calibrated_frame, intrinsic, distCoeffs);
	result = calibrated_frame.clone();
	medianBlur(calibrated_frame, calibrated_frame, 3);
	cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

	// Detect the object based on RGB and HSV Range Values
  inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);
	morphOps(hsv_frame_green);
	GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);
	Canny(hsv_frame_green_blur,hsv_frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);
	findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point> > contours_g_poly( contours_g.size() );
	vector<Point2f>center_g( contours_g.size() );
	vector<float>radius_g( contours_g.size() );


	for( size_t i = 0; i < contours_g.size(); i++ ){
		approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
		minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
	}

        float l, r1, r2;
        Point2f one, two;
        float x1, x2, y1, y2;

        size_t contour_g=contours_g.size();
        for (size_t i=0; i< contour_g; i++){
            if (radius_g[i] > iMin_tracking_ball_size){
                for (size_t j=0; j<contour_g; j++){
                    for (size_t k=0; k<contour_g; k++){
                        one=center_g[j];
                        two=center_g[k];
                        r1=radius_g[j];
                        r2=radius_g[k];

                        x1=one.x;
                        y1=one.y;
                        x2=two.x;
                        y2=two.y;
                        l=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

                        if (r1+r2>l){
                            if (r1>r2){
                                radius_g.erase(radius_g.begin()+k);
                                center_g.erase(center_g.begin()+k);
                                contours_g.erase(contours_g.begin()+k);
                                contour_g--;
                                j--;
                            }
                        }
                    }
                }
            }
        }

        // int upper_limit = 450;
        // int lower_limit = 30;
        //
        // size_t contour_g2 = contours_g.size();
        // for (size_t i=0; i<contour_g2; i++){
        //    one=center_g[i];
        //    x1=one.x;
        //    y1=one.y;
        //    if (y1>upper_limit){
        //        radius_g.erase(radius_g.begin()+i);
        //        center_g.erase(center_g.begin()+i);
        //        contours_g.erase(contours_g.begin()+i);
        //        contour_g2--;
        //        i--;
        //    }
        //    if(y1<lower_limit){
        //         radius_g.erase(radius_g.begin()+i);
        //         center_g.erase(center_g.begin()+i);
        //         contours_g.erase(contours_g.begin()+i);
        //         contour_g2--;
        //         i--;
        //    }
        // }

   core_msgs::ball_position_back msg;  //create a message for ball positions


   msg.size3 = contours_g.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
   msg.img_x3.resize(contours_g.size());  //adjust the size of array
   msg.img_y3.resize(contours_g.size());  //adjust the size of array

	for( size_t i = 0; i< contours_g.size(); i++ ){
		if(radius_g[i] > iMin_tracking_ball_size){
			Scalar color = Scalar( 0, 255, 0);
			vector<float> ball_position_g;
      ball_position_g = pixel2point(center_g[i], radius_g[i]);
			drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			float isx = -(320-center_g[i].x);
			float isy = (360-center_g[i].y);
			float isz = ball_position_g[2];
			string sx = floatToString(isx);
			string sy = floatToString(isy);
      string sz = floatToString(isz);
      // cout<<"green center : "<<isx<<", "<<isy<<endl;
                        msg.img_x3[i] = -(320-center_g[i].x);
                        msg.img_y3[i] = (360-center_g[i].y);

			text = "Green:" + sx  + "," + sy;
			putText(result, text, center_g[i],2,1,Scalar(0,255,0),2);
			circle( result, center_g[i], (int)radius_g[i], color, 2 , 8, 0 );
		}
	}
  cout<<"green size : "<<contours_g.size()<<endl;
  // cv::waitKey(1);
	// imshow("Object Detection_HSV_Green",hsv_frame_green);
	// imshow("Result", result);

	pub.publish(msg);
}

int main(int argc, char **argv)
{
	VideoCapture cap(2);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
    // namedWindow("Object Detection_HSV_Green", WINDOW_NORMAL);
    // namedWindow("Result", WINDOW_NORMAL);
    // moveWindow("Object Detection_HSV_Green", 890,370);
    // moveWindow("Result", 470, 0);
    // //
    // createTrackbar("Low H","Object Detection_HSV_Green", &low_h_g, 180, on_low_h_thresh_trackbar_green);
    // createTrackbar("High H","Object Detection_HSV_Green", &high_h_g, 180, on_high_h_thresh_trackbar_green);
    // createTrackbar("Low S","Object Detection_HSV_Green", &low_s_g, 255, on_low_s_thresh_trackbar_green);
    // createTrackbar("High S","Object Detection_HSV_Green", &high_s_g, 255, on_high_s_thresh_trackbar_green);
    // createTrackbar("Low V","Object Detection_HSV_Green", &low_v_g, 255, on_low_v_thresh_trackbar_green);
    // createTrackbar("High V","Object Detection_HSV_Green", &high_v_g, 255, on_high_v_thresh_trackbar_green);

   ros::init(argc, argv, "ball_detect_back2_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler

   pub = nh.advertise<core_msgs::ball_position_back>("/position_back", 100); //setting publisher

   int t=0;
   while (ros::ok()){
     cap>>frame;
     ball_detect();
     // ros::Duration(0.01).sleep();
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

// Trackbar for image threshodling in HSV colorspace : Green
void on_low_h_thresh_trackbar_green(int, void *){
    low_h_g = min(high_h_g-1, low_h_g);
    setTrackbarPos("Low H","Object Detection_HSV_Green", low_h_g);
}
void on_high_h_thresh_trackbar_green(int, void *){
    high_h_g = max(high_h_g, low_h_g+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_g);
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
    low_v_g= min(high_v_g-1, low_v_g);
    setTrackbarPos("Low V","Object Detection_HSV_Green", low_v_g);
}
void on_high_v_thresh_trackbar_green(int, void *){
    high_v_g = max(high_v_g, low_v_g+1);
    setTrackbarPos("High V", "Object Detection_HSV_Green", high_v_g);
}
void on_canny_edge_trackbar_green(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Green Ball", lowThreshold_g);
}
