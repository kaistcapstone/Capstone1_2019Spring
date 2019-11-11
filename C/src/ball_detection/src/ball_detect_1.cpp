#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <core_msgs/ball_position.h>
#include <std_msgs/ColorRGBA.h>
//#include <visualization_msgs/Marker.h>
using namespace std;
using namespace cv;
/*
pro1
low_h_r : 0
high_h_r : 6
low_h2_r : 178
high_h2_r : 180
low_s_r : 189
high_s_r : 255
low_v_r : 73
high_v_r : 255

low_h_b : 98
high_h_b : 117
low_s_b : 130
high_s_b : 255
low_v_b : 70
high_v_b : 255

pro2 - in lab
low_h_r : 0
high_h_r : 6
low_h2_r : 178
high_h2_r : 180
low_s_r : 168
high_s_r : 255
low_v_r : 70
high_v_r : 255

low_h_b : 98
high_h_b : 117
low_s_b : 130
high_s_b : 255
low_v_b : 70
high_v_b : 255

low_h_g : 51
high_h_g : 82
low_s_g : 60
high_s_g : 255
low_v_g : 76
high_v_g : 255

*/
// Declaration of trackbar functions to set HSV colorspace's parameters
void on_low_h_thresh_trackbar_red(int, void *);
void on_high_h_thresh_trackbar_red(int, void *);
void on_low_h2_thresh_trackbar_red(int, void *);
void on_high_h2_thresh_trackbar_red(int, void *);
void on_low_s_thresh_trackbar_red(int, void *);
void on_high_s_thresh_trackbar_red(int, void *);
void on_low_v_thresh_trackbar_red(int, void *);
void on_high_v_thresh_trackbar_red(int, void *);
int low_h2_r=178, high_h2_r=180;
int low_h_r=0, low_s_r=207, low_v_r=87;
int high_h_r=6, high_s_r=255, high_v_r=255;


void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);
int low_h_b=98, low_s_b=200, low_v_b=70;
int high_h_b=120, high_s_b=255, high_v_b=240;


void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);
int low_h_g=51, low_s_g=60, low_v_g=100;
int high_h_g=82, high_s_g=245, high_v_g=253;


// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);

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
float fball_radius = 0.074 ; // meter

// Initialization of variable for camera calibration paramters
Mat distCoeffs;
float intrinsic_data[9] = {637.593481, 0, 315.209216, 0, 641.348267, 251.475646, 0, 0, 1};
float distortion_data[5] = {0.029069, -0.112136, 0.013558, -0.008788, 0};

// Initialization of variable for text drawing
double fontScale = 2;
int thickness = 3;
String text ;
/*
typedef struct {
		int red_cnt = 0;
		int blue_cnt = 0;
		int green_cnt = 0;
		float *imgx_red;
		float *imgx_blue;
		float *imgx_green;
		float *imgy_red;
		float *imgy_blue;
		float *imgy_green;
}pos;

*/
int iMin_tracking_ball_size = 9;
int iMin_tracking_green_ball_size = 6;
core_msgs::ball_position ball_detect(Mat frame){

	Mat bgr_frame, hsv_frame, hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue, hsv_frame_green, hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_green_blur, hsv_frame_red_canny, hsv_frame_blue_canny, hsv_frame_green_canny, result;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    Mat eq_red, eq_blue, eq_green;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);
    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
	vector<Vec4i> hierarchy_g;
    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;
	vector<vector<Point> > contours_g;
	
	
	//pos res;

    //VideoCapture cap_1(0);
    

    // Trackbars to set thresholds for HSV values : Red ball
    createTrackbar("Low H","Object Detection_HSV_Red_1", &low_h_r, 180, on_low_h_thresh_trackbar_red);
    createTrackbar("High H","Object Detection_HSV_Red_1", &high_h_r, 180, on_high_h_thresh_trackbar_red);
    createTrackbar("Low H2","Object Detection_HSV_Red_1", &low_h2_r, 180, on_low_h2_thresh_trackbar_red);
    createTrackbar("High H2","Object Detection_HSV_Red_1", &high_h2_r, 180, on_high_h2_thresh_trackbar_red);
    createTrackbar("Low S","Object Detection_HSV_Red_1", &low_s_r, 255, on_low_s_thresh_trackbar_red);
    createTrackbar("High S","Object Detection_HSV_Red_1", &high_s_r, 255, on_high_s_thresh_trackbar_red);
    createTrackbar("Low V","Object Detection_HSV_Red_1", &low_v_r, 255, on_low_v_thresh_trackbar_red);
    createTrackbar("High V","Object Detection_HSV_Red_1", &high_v_r, 255, on_high_v_thresh_trackbar_red);

    // Trackbars to set thresholds for HSV values : Blue ball
    createTrackbar("Low H","Object Detection_HSV_Blue_1", &low_h_b, 180, on_low_h_thresh_trackbar_blue);
    createTrackbar("High H","Object Detection_HSV_Blue_1", &high_h_b, 180, on_high_h_thresh_trackbar_blue);
    createTrackbar("Low S","Object Detection_HSV_Blue_1", &low_s_b, 255, on_low_s_thresh_trackbar_blue);
    createTrackbar("High S","Object Detection_HSV_Blue_1", &high_s_b, 255, on_high_s_thresh_trackbar_blue);
    createTrackbar("Low V","Object Detection_HSV_Blue_1", &low_v_b, 255, on_low_v_thresh_trackbar_blue);
    createTrackbar("High V","Object Detection_HSV_Blue_1", &high_v_b, 255, on_high_v_thresh_trackbar_blue);
	
	// Trackbars to set thresholds for HSV values : Green ball
    createTrackbar("Low H","Object Detection_HSV_Green_1", &low_h_g, 180, on_low_h_thresh_trackbar_green);
    createTrackbar("High H","Object Detection_HSV_Green_1", &high_h_g, 180, on_high_h_thresh_trackbar_green);
    createTrackbar("Low S","Object Detection_HSV_Green_1", &low_s_g, 255, on_low_s_thresh_trackbar_green);
    createTrackbar("High S","Object Detection_HSV_Green_1", &high_s_g, 255, on_high_s_thresh_trackbar_green);
    createTrackbar("Low V","Object Detection_HSV_Green_1", &low_v_g, 255, on_low_v_thresh_trackbar_green);
    createTrackbar("High V","Object Detection_HSV_Green_1", &high_v_g, 255, on_high_v_thresh_trackbar_green);


    // Trackbar to set parameter for Canny Edge
    createTrackbar("Min Threshold:","Canny Edge for Red Ball_1", &lowThreshold_r, 100, on_canny_edge_trackbar_red);
    createTrackbar("Min Threshold:","Canny Edge for Blue Ball_1", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);
	createTrackbar("Min Threshold:","Canny Edge for Green Ball_1", &lowThreshold_g, 100, on_canny_edge_trackbar_green);

   // while((char)waitKey(1)!='q'){
        //cap_1>>frame;

        //if(frame.empty()) break;

        undistort(frame, calibrated_frame, intrinsic, distCoeffs);
        result = calibrated_frame.clone();
        medianBlur(calibrated_frame, calibrated_frame, 3);
        cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

        // Detect the object based on RGB and HSV Range Values
        inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
        inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
        inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
		inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);
        addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);
        morphOps(hsv_frame_red);
        morphOps(hsv_frame_blue);
		morphOps(hsv_frame_green);

        GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
        GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
		GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);

        equalizeHist(hsv_frame_red_blur, eq_red);
        equalizeHist(hsv_frame_blue_blur, eq_blue);
		equalizeHist(hsv_frame_green_blur, eq_green);

        Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
        Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
		Canny(hsv_frame_green_blur, hsv_frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);


        findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

        vector<vector<Point> > contours_r_poly( contours_r.size() );
        vector<vector<Point> > contours_b_poly( contours_b.size() );
		vector<vector<Point> > contours_g_poly( contours_g.size() );
        vector<Point2f>center_r( contours_r.size() );
        vector<Point2f>center_b( contours_b.size() );
		vector<Point2f>center_g( contours_g.size() );
        vector<float>radius_r( contours_r.size() );
        vector<float>radius_b( contours_b.size() );
		vector<float>radius_g( contours_g.size() );

		//Determine circle for red ball
        for( size_t i = 0; i < contours_r.size(); i++ ){
            approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
            minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
        }
        //cout << contours_r.size() << endl;

		//If small ball is inside of bigger ball, erase small ball
        if(contours_r.size() > 1){

            for ( size_t i = 0; i < contours_r.size(); i++ ){
                for ( size_t j = i + 1; j < contours_r.size();j++){
                    float dis = sqrt((center_r[i].x - center_r[j].x)*(center_r[i].x - center_r[j].x) + (center_r[i].y - center_r[j].y)*(center_r[i].y - center_r[j].y));

                    if(radius_r[i] > radius_r[j]){
                        if(dis < radius_r[i]){
                            contours_r.erase(contours_r.begin() + j);
                            contours_r_poly.erase(contours_r_poly.begin() + j);
                            center_r.erase(center_r.begin() + j);
                            radius_r.erase(radius_r.begin() + j);
                            j-=1;
                        }
                    }

                    else if(radius_r[j] > radius_r[i]){
                        if(dis < radius_r[j]){
                            contours_r.erase(contours_r.begin() + i);
                            contours_r_poly.erase(contours_r_poly.begin() + i);
                            center_r.erase(center_r.begin() + i);
                            radius_r.erase(radius_r.begin() + i);
                            j-=1;
                        }
                    }
                }
            }
        }
       // cout << contours_r.size() << endl << "----------" << endl;

		//Determine circle for blue ball
        for( size_t i = 0; i < contours_b.size(); i++ ){
            approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
            minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
        }

		if(contours_b.size() > 1){

            for ( size_t i = 0; i < contours_b.size(); i++ ){
                for ( size_t j = i + 1; j < contours_b.size();j++){
                    float dis = sqrt((center_b[i].x - center_b[j].x)*(center_b[i].x - center_b[j].x) + (center_b[i].y - center_b[j].y)*(center_b[i].y - center_b[j].y));

                    if(radius_b[i] > radius_b[j]){
                        if(dis < radius_b[i]){
                            contours_b.erase(contours_b.begin() + j);
                            contours_b_poly.erase(contours_b_poly.begin() + j);
                            center_b.erase(center_b.begin() + j);
                            radius_b.erase(radius_b.begin() + j);
                            j-=1;
                        }
                    }

                    else if(radius_b[j] > radius_b[i]){
                        if(dis < radius_b[j]){
                            contours_b.erase(contours_b.begin() + i);
                            contours_b_poly.erase(contours_b_poly.begin() + i);
                            center_b.erase(center_b.begin() + i);
                            radius_b.erase(radius_b.begin() + i);
                            j-=1;
                        }
                    }
                }
            }
        }

		//Determine circle for green ball
		for( size_t i = 0; i < contours_g.size(); i++ ){
            approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
            minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
        }

		if(contours_g.size() > 1){

            for ( size_t i = 0; i < contours_g.size(); i++ ){
                for ( size_t j = i + 1; j < contours_g.size();j++){
                    float dis = sqrt((center_g[i].x - center_g[j].x)*(center_g[i].x - center_g[j].x) + (center_g[i].y - center_g[j].y)*(center_g[i].y - center_g[j].y));

                    if(radius_g[i] > radius_g[j]){
                        if(dis < radius_g[i]){
                            contours_g.erase(contours_g.begin() + j);
                            contours_g_poly.erase(contours_g_poly.begin() + j);
                            center_g.erase(center_g.begin() + j);
                            radius_g.erase(radius_g.begin() + j);
                            j-=1;
                        }
                    }

                    else if(radius_g[j] > radius_g[i]){
                        if(dis < radius_g[j]){
                            contours_g.erase(contours_g.begin() + i);
                            contours_g_poly.erase(contours_g_poly.begin() + i);
                            center_g.erase(center_g.begin() + i);
                            radius_g.erase(radius_g.begin() + i);
                            j-=1;
                        }
                    }
                }
            }
		}
        

		float red_cnt = 0, blue_cnt = 0, green_cnt = 0;
		float red_imgx, red_imgy, blue_imgx, blue_imgy, green_imgx, green_imgy;
		float cam_height = 0.45, y_offset = 0;
		core_msgs::ball_position out;

		//Determine the number of each ball
		for(size_t i = 0; i<contours_r.size();i++){
			if(radius_r[i] > iMin_tracking_ball_size) red_cnt += 1;
		}
		
		for(size_t i = 0; i<contours_b.size();i++){
			if(radius_b[i] > iMin_tracking_ball_size) blue_cnt += 1;
		}

		for(size_t i = 0; i<contours_g.size();i++){
			if(radius_g[i] > iMin_tracking_green_ball_size) green_cnt += 1;
		}
		
		//Pick most large ball
		if(contours_r.size() > 1){
			//Fill 0 index			
			float max = 0;
			size_t max_i = 0;
			for(size_t i =0; i<contours_r.size();i++){
				if(radius_r[i] > max){
					max = radius_r[i];
					max_i = i;
				}
			}
			vector<Point2f>temp_r(1);
			temp_r[0] = center_r[max_i];
			
			radius_r[max_i] = radius_r[0];
			center_r[max_i] = center_r[0];

			radius_r[0] = max;
			center_r[0] = temp_r[0];
			
			//Fill 1, 2 index
			if(contours_r.size() > 2){
				if(radius_r[1] < radius_r[2]){
					temp_r[0] = center_r[1];
					center_r[1] = center_r[2];
					center_r[2] = temp_r[0];

					float temp_radius = radius_r[1];
					radius_r[1] = radius_r[2];
					radius_r[2] = temp_radius;
				}
			}

		}

		
			
		if(contours_b.size() > 1){
			float max = 0;
			size_t max_i = 0;
			for(size_t i =0; i<contours_b.size();i++){
				if(radius_b[i] > max){
					max = radius_b[i];
					max_i = i;
				}
			}
			vector<Point2f>temp_b(1);
			temp_b[0] = center_b[max_i];
			
			radius_b[max_i] = radius_b[0];
			center_b[max_i] = center_b[0];

			radius_b[0] = max;
			center_b[0] = temp_b[0];

			//Fill 1, 2 index
			if(contours_b.size() > 2){
				if(radius_b[1] < radius_b[2]){
					temp_b[0] = center_b[1];
					center_b[1] = center_b[2];
					center_b[2] = temp_b[0];

					float temp_radius = radius_b[1];
					radius_b[1] = radius_b[2];
					radius_b[2] = temp_radius;
				}
			}

		}
		//if(contours_b.size() >= 1) ROS_INFO("Most Big Blue : center(%f, %f), radius(%f)",center_b[0].x, center_b[0].y, radius_b[0]);

		if(contours_g.size() > 1){
			float max = 0;
			size_t max_i = 0;
			for(size_t i =0; i<contours_g.size();i++){
				if(radius_g[i] > max){
					max = radius_g[i];
					max_i = i;
				}
			}
			vector<Point2f>temp_g(1);
			temp_g[0] = center_g[max_i];
			
			radius_g[max_i] = radius_g[0];
			center_g[max_i] = center_g[0];

			radius_g[0] = max;
			center_g[0] = temp_g[0];
			
			//Fill 1, 2 index
			if(contours_g.size() > 2){
				if(radius_g[1] < radius_g[2]){
					temp_g[0] = center_g[1];
					center_g[1] = center_g[2];
					center_g[2] = temp_g[0];

					float temp_radius = radius_g[1];
					radius_g[1] = radius_g[2];
					radius_g[2] = temp_radius;
				}
			}


		}
		//if(contours_g.size() >= 1) ROS_INFO("Most Big Green : center(%f, %f), radius(%f)",center_g[0].x, center_g[0].y, radius_g[0]);
		

		out.size_red = red_cnt;
		out.size_blue = blue_cnt;
		out.size_green = green_cnt;

		out.img_x_red.resize( red_cnt );
		out.img_x_blue.resize( blue_cnt );
		out.img_x_green.resize( green_cnt );

		out.img_y_red.resize( red_cnt );
		out.img_y_blue.resize( blue_cnt );
		out.img_y_green.resize( green_cnt );
		
		//Calculate ball position
		size_t j = 0;
        for( size_t i = 0; i< contours_r.size(); i++ ){
            if (radius_r[i] > iMin_tracking_ball_size){
				
                
                Scalar color = Scalar( 0, 0, 255);
                drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                vector<float> ball_position_r;
				float ratio_r;
				
				
				ratio_r = 0.0111*radius_r[i] + 0.43;

				if(ratio_r <= 0.56) ratio_r = 0.56;
				if(ratio_r >= 1) ratio_r = 1;
				radius_r[i] *= ratio_r;
				//ROS_INFO("RED BALL, center : (%f, %f), radius : %f", center_r[i].x, center_r[i].y, radius_r[i]);
                ball_position_r = pixel2point(center_r[i], radius_r[i]);

                float isx = ball_position_r[0];
                float isy = ball_position_r[1];
                float isz = ball_position_r[2];

                string sx = floatToString(isx);
                string sy = floatToString(isy);
                string sz = floatToString(isz);
				string index = intToString(j);
				out.img_x_red[j] = isx;
				out.img_y_red[j] = sqrt(isz*isz + isy*isy - cam_height * cam_height) + y_offset;
                text = "<" + index + ">, " + "x : " + floatToString(isx) + "," + "y : " + floatToString(out.img_y_red[j]);
				
                Point2f text_loc(center_r[i].x - 200, center_r[i].y);
                putText(result, text, text_loc,2,1,Scalar(0,255,0),2);
                circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
				
				
				
				
				//ROS_INFO("RED BALL, x : %f, y : %f", out.img_x_red[j], out.img_y_red[j]);
				j += 1;
            }
        }
		
		j = 0;
        for( size_t i = 0; i< contours_b.size(); i++ ){
            if(radius_b[i] > iMin_tracking_ball_size){
				               
				Scalar color = Scalar( 255, 0, 0);
                drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                vector<float> ball_position_b;
				float ratio_b;
				

				ratio_b = 0.0099*radius_b[i] + 0.49;
				//ROS_INFO("ratio_b : %f", ratio_b);
				if(ratio_b <= 0.58) ratio_b = 0.58;
				if(ratio_b >= 1) ratio_b = 1;
				radius_b[i] *= ratio_b;
                ball_position_b = pixel2point(center_b[i], radius_b[i]);

                float isx = ball_position_b[0];
                float isy = ball_position_b[1];
                float isz = ball_position_b[2];

                string sx = floatToString(isx);
                string sy = floatToString(isy);
                string sz = floatToString(isz);
				string index = intToString(j);
				out.img_x_blue[j] = isx;
				out.img_y_blue[j] = sqrt(isz*isz + isy*isy - cam_height * cam_height) + y_offset;
                text = "<" + index + ">, " + "x : " + floatToString(isx) + "," + "y : " + floatToString(out.img_y_blue[j]);
				Point2f text_loc(center_b[i].x - 200, center_b[i].y);
                putText(result, text, text_loc ,2,1,Scalar(0,255,0),2);
                circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );

				
				
				//ROS_INFO("BLUE BALL, x : %f, y : %f", out.img_x_blue[j], out.img_y_blue[j]);
				j += 1;

            }
        }

		j = 0;
		for( size_t i = 0; i< contours_g.size(); i++ ){
            if(radius_g[i] > iMin_tracking_green_ball_size){
				
                Scalar color = Scalar( 0, 255, 0);
                drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                vector<float> ball_position_g;
				float ratio_g;
				
				
				ratio_g = 0.0093*radius_g[i] + 0.53;
				if(ratio_g <= 0.59) ratio_g = 0.59;
				if(ratio_g >= 1) ratio_g = 1;
				radius_g[i] *= ratio_g;              
				//ROS_INFO("GREEN BALL, center : (%f, %f), radius : %f", center_g[i].x, center_g[i].y, radius_g[i]);
				ball_position_g = pixel2point(center_g[i], radius_g[i]);

                float isx = ball_position_g[0];
                float isy = ball_position_g[1];
                float isz = ball_position_g[2];

                string sx = floatToString(isx);
                string sy = floatToString(isy);
                string sz = floatToString(isz);
				string index = intToString(j);
				out.img_x_green[j] = isx;
				out.img_y_green[j] = sqrt(isz*isz + isy*isy - cam_height * cam_height) + y_offset;
                text = "<" + index + ">, " + "x : " + floatToString(isx) + "," + "y : " + floatToString(out.img_y_green[j]);
				Point2f text_loc(center_g[i].x - 200, center_g[i].y);
                putText(result, text, text_loc,2,1,Scalar(0,255,0),2);
                circle( result, center_g[i], (int)radius_g[i], color, 2, 8, 0 );

				
				
				ROS_INFO("GREEN BALL, x : %f, y : %f", out.img_x_green[j], out.img_y_green[j]);
				j += 1;

            }
        }

        // Show the frames
      /*  imshow("Video Capture_1",calibrated_frame);
		imshow("hsv frame",hsv_frame);
		imshow("after equalizer", eq_red);
		imshow("before equalizer", hsv_frame_red_blur);
        
		
		imshow("Object Detection_HSV_Red_1",hsv_frame_red);
        imshow("Object Detection_HSV_Blue_1",hsv_frame_blue);
		
        imshow("Canny Edge for Blue Ball_1", hsv_frame_blue_canny);		
		imshow("Canny Edge for Red Ball_1", hsv_frame_red_canny);		
		
		imshow("Object Detection_HSV_Green_1",hsv_frame_green);
		imshow("Canny Edge for Green Ball_1", hsv_frame_green_canny);*/
        imshow("Result_1", result);
   // }
	
	waitKey(1);
//No garbage collection now (free the allocated memory)

	if(red_cnt == 0){
		out.img_x_red.resize(1);
		out.img_y_red.resize(1);
		out.img_x_red[0] = 0;
		out.img_y_red[0] = 0;
	}

	if(blue_cnt == 0){
		out.img_x_blue.resize(1);
		out.img_y_blue.resize(1);
		out.img_x_blue[0] = 0;
		out.img_y_blue[0] = 0;
	}

	if(green_cnt == 0){
		out.img_x_green.resize(1);
		out.img_y_green.resize(1);
		out.img_x_green[0] = 0;
		out.img_y_green[0] = 0;
	}

	return out;


}

class pass{
	private :
		ros::NodeHandle nh;
		ros::Publisher pub;
		image_transport::Subscriber sub;
	//	ros::Publisher pub_markers;
		

	public :
		pass(){
			image_transport::ImageTransport it(nh);
			pub = nh.advertise<core_msgs::ball_position>("/position_1", 100);
		//	pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);
			sub = it.subscribe("camera/image_1", 1, &pass::imageCallback, this);

		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg){
			Mat received;
			Mat new_rec;			
			try{
				received = cv_bridge::toCvShare(msg, "bgr8")->image;
			}

			catch(cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
			}
			
			if(received.size().width == 320) cv::resize(received, new_rec, cv::Size(640, 480));	
			pub.publish(ball_detect(new_rec));
			//ROS_INFO("send position");

		}

};

int main(int argc, char **argv){
	
	ros::init(argc, argv, "ball_detect_1");

	pass PASS;

/*	namedWindow("Video Capture_1", WINDOW_NORMAL);
    
    
	namedWindow("hsv frame",WINDOW_NORMAL);
	namedWindow("after equalizer", WINDOW_NORMAL);
	namedWindow("before equalizer", WINDOW_NORMAL);
      
	
	namedWindow("Canny Edge for Blue Ball_1", WINDOW_NORMAL);	
	namedWindow("Object Detection_HSV_Blue_1", WINDOW_NORMAL);	
		
	namedWindow("Object Detection_HSV_Red_1", WINDOW_NORMAL);
	namedWindow("Canny Edge for Red Ball_1", WINDOW_NORMAL);  
	namedWindow("Object Detection_HSV_Green_1", WINDOW_NORMAL);
    	
	namedWindow("Canny Edge for Green Ball_1", WINDOW_NORMAL);	*/
	namedWindow("Result_1", WINDOW_NORMAL);
	
/*    moveWindow("Video Capture_1", 50, 0);
    
    
	moveWindow("Object Detection_HSV_Green_1",890,370);
    moveWindow("hsv frame",50, 370);
	moveWindow("after equalizer", 470, 0);
	moveWindow("before equalizer", 470, 370);
    moveWindow("Canny Edge for Blue Ball_1", 470,730);	
	moveWindow("Object Detection_HSV_Blue_1",470,370);	
	    
	moveWindow("Object Detection_HSV_Red_1", 50,370);
	
	moveWindow("Canny Edge for Green Ball_1", 890,730);*/
	//moveWindow("Canny Edge for Red Ball_1", 50,730);	
	moveWindow("Result_1", 50, 0);
	
	
	ros::spin();

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
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
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


// Trackbar for image threshodling in HSV colorspace : Red
void on_low_h_thresh_trackbar_red(int, void *){
    low_h_r = min(high_h_r-1, low_h_r);
    setTrackbarPos("Low H","Object Detection_HSV_Red_1", low_h_r);
}

void on_high_h_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red_1", high_h_r);
}

void on_low_h2_thresh_trackbar_red(int, void *){
    low_h2_r = min(high_h2_r-1, low_h2_r);
    setTrackbarPos("Low H2","Object Detection_HSV_Red_1", low_h2_r);
}

void on_high_h2_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red_1", high_h_r);
}

void on_low_s_thresh_trackbar_red(int, void *){
    low_s_r = min(high_s_r-1, low_s_r);
    setTrackbarPos("Low S","Object Detection_HSV_Red_1", low_s_r);
}

void on_high_s_thresh_trackbar_red(int, void *){
    high_s_r = max(high_s_r, low_s_r+1);
    setTrackbarPos("High S", "Object Detection_HSV_Red_1", high_s_r);
}

void on_low_v_thresh_trackbar_red(int, void *){
    low_v_r= min(high_v_r-1, low_v_r);
    setTrackbarPos("Low V","Object Detection_HSV_Red_1", low_v_r);
}

void on_high_v_thresh_trackbar_red(int, void *){
    high_v_r = max(high_v_r, low_v_r+1);
    setTrackbarPos("High V", "Object Detection_HSV_Red_1", high_v_r);
}

// Trackbar for image threshodling in HSV colorspace : Blue
void on_low_h_thresh_trackbar_blue(int, void *){
    low_h_b = min(high_h_b-1, low_h_b);
    setTrackbarPos("Low H","Object Detection_HSV_Blue_1", low_h_b);
}

void on_high_h_thresh_trackbar_blue(int, void *){
    high_h_b = max(high_h_b, low_h_b+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue_1", high_h_b);
}

void on_low_s_thresh_trackbar_blue(int, void *){
    low_s_b = min(high_s_b-1, low_s_b);
    setTrackbarPos("Low S","Object Detection_HSV_Blue_1", low_s_b);
}

void on_high_s_thresh_trackbar_blue(int, void *){
    high_s_b = max(high_s_b, low_s_b+1);
    setTrackbarPos("High S", "Object Detection_HSV_Blue_1", high_s_b);
}

void on_low_v_thresh_trackbar_blue(int, void *){
    low_v_b= min(high_v_b-1, low_v_b);
    setTrackbarPos("Low V","Object Detection_HSV_Blue_1", low_v_b);
}

void on_high_v_thresh_trackbar_blue(int, void *){
    high_v_b = max(high_v_b, low_v_b+1);
    setTrackbarPos("High V", "Object Detection_HSV_Blue_1", high_v_b);
}

// Trackbar for image threshodling in HSV colorspace : Green
void on_low_h_thresh_trackbar_green(int, void *){
    low_h_g = min(high_h_g-1, low_h_g);
    setTrackbarPos("Low H","Object Detection_HSV_Green_1", low_h_g);
}

void on_high_h_thresh_trackbar_green(int, void *){
    high_h_g = max(high_h_g, low_h_g+1);
    setTrackbarPos("High H", "Object Detection_HSV_Green_1", high_h_g);
}

void on_low_s_thresh_trackbar_green(int, void *){
    low_s_g = min(high_s_g-1, low_s_g);
    setTrackbarPos("Low S","Object Detection_HSV_Green_1", low_s_g);
}

void on_high_s_thresh_trackbar_green(int, void *){
    high_s_g = max(high_s_g, low_s_g+1);
    setTrackbarPos("High S", "Object Detection_HSV_Green_1", high_s_g);
}

void on_low_v_thresh_trackbar_green(int, void *){
    low_v_g= min(high_v_g-1, low_v_g);
    setTrackbarPos("Low V","Object Detection_HSV_Green_1", low_v_g);
}

void on_high_v_thresh_trackbar_green(int, void *){
    high_v_g = max(high_v_g, low_v_g+1);
    setTrackbarPos("High V", "Object Detection_HSV_Green_1", high_v_g);
}


// Trackbar for Canny edge algorithm
void on_canny_edge_trackbar_red(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Red Ball_1", lowThreshold_r);
}

void on_canny_edge_trackbar_blue(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball_1", lowThreshold_b);
}

void on_canny_edge_trackbar_green(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Green Ball_1", lowThreshold_g);
}


