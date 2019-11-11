//version 5.17 pm23:17 by inchul + JW
//version 5.19 pm20:45 by JW (green ball detect code included && need to change getting angle at path planning from acos to atan2
//version 5.24 pm23:44 by JW (Done except overlap)
//version 5.25 23:02 by inchul pathgenmode
//version 5.28 18:37 by JW avoid red ball while return to release balls
/*version 5.30 03:00 by JW HS GJ
	1. pick up avoid range change
	2. angle1 align
	3. avoid red ball : different avoiding direction depend on red ball y distance.
	4. additional rotate very slow (after avoid-blue ball align, angle1~3 align, green ball align)
*/
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"

//----------declare functions------------
void avoid_RedBall();
void no_red_ball();
void releaseAlign();

int lineRed = 70;		// distance(cm) of starting avoiding red ball
int lineBlue = 50;	// distance(cm) of starting moving forward forcefully to pickup blue ball.
int cenB = 0;
int minR = 0;
int minB = 0;
int rTheta = 0;
int lTheta = 0;
int findGreen = 0;		//increment when rotated to find green balls

//----------variavles for data_integation---------
#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress

using namespace std;

boost::mutex map_mutex;
int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int red_ball_number;
int blue_ball_number;
int green_ball_number;
float r_ball_X[20];
float r_ball_Y[20];
float b_ball_X[20];
float b_ball_Y[20];
float g_ball_X[20];
float g_ball_Y[20];
float ball_distance[20];
int near_ball;

int collect=1;
int rotated=0;

int pathmode = -1; // 0 : 1st ball coverde, 1: 2nd ball covered, 2: normal path
int ballcovered = -1; // -1:default, 0: firstball covering other, 1: second ball covered

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];

//----------variavles for pathgen---------
int check = -1;

#define PI 3.14159265

int carx = 30, cary = 40;  // car size
double saferange = 30;

double tcost[6] = {0, };

double t_max = 100000;
double straight = 20;   // cm/s
double rotate_cost = 10;     // degree/s
double diagonal = 10;   // cm/s
double stood = 1;
double dtod = 3;

double base_x = 0, base_y = 0;

int size_r=3, size_b=3, size_g=2;
double b_ball_x[3] ={0, }, r_ball_x[3] ={0, }, g_ball_x[2] ={0, };
double b_ball_y[3] ={0, }, r_ball_y[3] ={0, }, g_ball_y[2] ={0, };

double angle1; //base to b1;
double angle2; //b1 to b2;
double angle3; //b2 to b3;
double angle4; //b3 to base;
double angle5; //base to goal;

double prep = 0;


string filePath = "/home/capstonea/result.txt";
ofstream txtFile(filePath);

void dataInit()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	data[6] = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
	data[7] = 0; //GamepadStickLength(_dev, STICK_RIGHT);
	data[8] = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
	data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
	data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
	data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
	data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
	data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
	data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
	data[15] = 0; //GamepadButtonDown(_dev, BUTTON_B);
	data[16] = 0; //GamepadButtonDown(_dev, BUTTON_X);
	data[17] = 0; //GamepadButtonDown(_dev, BUTTON_Y);
	data[18] = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
	data[19] = 0; //GamepadButtonDown(_dev, BUTTON_START);
	data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
	data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
	data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
	data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
}

void lift(){
	data[17]=1;
	cout<<"lift"<<endl;
	txtFile << "lift" << endl;
}

void down(){
	data[17]=0;
	cout<<"down"<<endl;
	txtFile<<"down"<<endl;
}

void turn_right()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0.6;
  data[5] = 0;
	cout<<"turn right"<<endl;
	txtFile<<"turn_right"<<endl;
}

void turn_right_medium()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0.3;
  data[5] = 0;
	cout<<"turn right"<<endl;
	txtFile<<"turn_right_medium"<<endl;

}

void turn_right_slow(){
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0.2;
	data[5] = 0;
	cout<<"turn right"<<endl;
	txtFile<<"turn_right_slow"<<endl;

}

void turn_right_very_slow(){
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0.1;
	data[5] = 0;
	cout<<"turn right"<<endl;
	txtFile<<"turn_right_very_slow"<<endl;

}

void turn_left()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = -0.6;
  data[5] = 0;
	cout<<"turn left"<<endl;
	txtFile<<"turn_left"<<endl;

}

void turn_left_medium()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = -0.3;
  data[5] = 0;
	cout<<"turn left"<<endl;
	txtFile<<"turn_left_medium"<<endl;

}

void turn_left_slow()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = -0.2;
  data[5] = 0;
	cout<<"turn left"<<endl;
	txtFile<<"turn_left_slow"<<endl;

}

void turn_left_very_slow()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = -0.1;
  data[5] = 0;
	cout<<"turn left"<<endl;
	txtFile<<"turn_left_very_slow"<<endl;

}

void move_forward()
{
  data[0] = 0;
  data[1] = 1;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"move forward"<<endl;
}

void move_forward_medium()
{
  data[0] = 0;
  data[1] = 0.6;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"move forward"<<endl;
}

void move_forward_slow()
{
  data[0] = 0;
  data[1] = 0.3;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"move forward"<<endl;
}

void stop()
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"stop"<<endl;
	txtFile << "stop" << endl;
}

void move_back(){
  data[0] = 0;
  data[1] = -1;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"move backward"<<endl;
	txtFile << "move backward" << endl;
}

void move_left(){
	data[0] = -1;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"move left"<<endl;
}

void move_left_medium(){
	data[0] = -0.2;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"move left medium"<<endl;
}

void move_left_slow(){
	data[0] = -0.1;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"left"<<endl;
}

void move_right(){
	data[0] = 1;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout << "move right" << endl;
}

void move_right_medium(){
	data[0] = 0.2;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout << "move right medium" << endl;
}

void move_right_slow(){
	data[0] = 0.1;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"right"<<endl;
}

void roller_back(){
	data[14]=1;
	data[17]=1;
	cout << "Drop ball" << endl;
	txtFile << "drop ball" << endl;
}

void forward_left(){
	data[0] = -0.55;
  data[1] = 0.7;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"forward left slow"<<endl;
}

void left_forward(){
	data[0] = -0.7;
  data[1] = 0.55;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"left forward slow"<<endl;
}

void forward_right(){
	data[0] = 0.55;
  data[1] = 0.7;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"forward right slow"<<endl;
}

void right_forward(){
	data[0] = 0.7;
  data[1] = 0.55;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
	cout<<"right forward slow"<<endl;
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int r_count = position->r_size;
		//red_ball_number = r_count;
		int error_r=0;
		int b_count = position->b_size;
		//blue_ball_number = b_count;
		int error_b=0;
		int g_count = position->g_size;
		//green_ball_number = g_count;
		int error_g=0;

    for(int r = 0; r < r_count; r++)
    {
			if( (position->r_img_y[r])== -100 )
			{
				error_r++;
			}
			else
			{
				r_ball_X[r-error_r] = position->r_img_x[r];
        r_ball_Y[r-error_r] = position->r_img_y[r];
			}
    }
		red_ball_number = r_count-error_r;

    for(int b = 0; b < b_count; b++)
    {
			if( (position->b_img_y[b]) > 500 || (position->b_img_y[b]) < 32)
			{
				error_b++;
			}
			else
			{
        b_ball_X[b-error_b] = position->b_img_x[b];
        b_ball_Y[b-error_b] = position->b_img_y[b];
			}
    }
		blue_ball_number = b_count-error_b;


		for(int g = 0; g < g_count; g++)
    {
			if( (position->g_img_y[g])== -100 )
			{
				error_g++;
			}
			else
			{
				g_ball_X[g-error_g] = position->g_img_x[g];
        g_ball_Y[g-error_g] = position->g_img_y[g];
			}
    }
		green_ball_number = g_count-error_g;

		//-----------rank of closest red ball
		minR = 0;
		for(int i=0; i<red_ball_number; i++){
			if(r_ball_Y[minR] > r_ball_Y[i]){
				minR = i;
			}
		}
		//------------rank of blue ball closest to center
		cenB = 0;
		for(int i=0; i<blue_ball_number; i++){
			if(abs(b_ball_X[cenB]) > abs(b_ball_X[i])){
				cenB = i;
			}
		}
		//------------rank of closest blue ball
		minB = 0;
		for(int i=0; i<blue_ball_number; i++){
			if(b_ball_Y[minB] > b_ball_Y[i]){
				minB = i;
			}
		}
}

void cor_show()
{
	cout << "cor show"<< endl;

	for(int r = 0; r < red_ball_number; r++)
	{
		cout << "red_ball_X : "<< r_ball_X[r];
		cout << "red_ball_Y : "<< r_ball_Y[r]<<std::endl;
	}
	for(int b = 0; b < blue_ball_number; b++)
	{
		cout << "blue_ball_X : "<< b_ball_X[b];
		cout << "blue_ball_Y : "<< b_ball_Y[b]<<std::endl;
	}
	for(int g = 0; g < green_ball_number; g++)
	{
		cout << "green_ball_X : "<< g_ball_X[g];
		cout << "green_ball_Y : "<< g_ball_Y[g]<<std::endl;
	}
}

void init_move()
{
	cout<<"init_move start"<<endl;
	move_forward();
						cout << "check it"<< endl;
	// write(c_socket, data, sizeof(data));
	// ros::Duration(2).sleep();

	stop();
	// write(c_socket, data, sizeof(data));
	// ros::Duration(0.1).sleep();
	cout<<"init_move end"<<endl;
}

void cor_init()
{
	cout<<"cor_init start"<<endl;

	// b_ball_x[0] = -80; b_ball_x[1] = 5;   b_ball_x[2] = 85;
	// b_ball_y[0] = 345; b_ball_y[1] = 335; b_ball_y[2] = 270;
	//
	// r_ball_x[0] = -75; r_ball_x[1] = 0;   r_ball_x[2] = 75;
	// r_ball_y[0] = 270; r_ball_y[1] = 270; r_ball_y[2] = 340;

	int cnt_b2 = 0;
	int cnt_b3 = 0;
	int cnt_b_error = 0;
	int previous_b = -1;

	int cnt_r3 =0;
	int cnt_r2 =0;

  int cnt_r[4] = {0, };
	int previous_r = -1;


	double b_temp_x[3] = {0, };
	double b_temp_y[3] = {0, };
	double r_temp_x[3] = {0, };
	double r_temp_y[3] = {0, };

  while(true)
	{
		ros::Duration(0.05).sleep();
		ros::spinOnce();
    //cor_show();

		if(blue_ball_number==3 && ballcovered==-1)
		{
			cout << "normal ball detection : 3b" <<endl;
			if(cnt_b2!=0) continue;

			cnt_b3 ++;
			previous_b = 3;

			for(int b = 0; b < blue_ball_number; b++)
			{
					b_temp_x[b] += b_ball_X[b];
					b_temp_y[b] += b_ball_Y[b];
			}
		}
		else if(blue_ball_number==2 && ballcovered!=-1)
		{
			// one ball covered

			cout << " one ball covered : 2b" <<endl;
			cnt_b2++;

			if(previous_b!=2)
			{
				for(int b = 0; b < blue_ball_number; b++)
				{
						b_temp_x[b] = b_ball_X[b];
						b_temp_y[b] = b_ball_Y[b];
				}
				b_temp_x[2] = 0;
				b_temp_y[2] = 0;
			}
			else
			{
				for(int b = 0; b < blue_ball_number; b++)
				{
						b_temp_x[b] += b_ball_X[b];
						b_temp_y[b] += b_ball_Y[b];
				}
				b_temp_x[2] = 0;
				b_temp_y[2] = 0;
			}

			previous_b=2;

		}
		else
		{
			// error
			cout << "no 3 blue ball" << endl;
			cnt_b_error++;
			// if(cnt_b_error ==20 && cnt_b2 == 0 && cnt_b3 ==0)
			// {
			// 	cout << "emergency escape1" << endl;
			// 	previous_b =-1;
			// 	break;
			// }
			// if(cnt_b_error == 1000)
			// {
			// 	cout << "emergency escape2" << endl;
			// 	previous_b =-1;
			// 	break;
			// }
			// continue;
		}



		if(red_ball_number>=3)
		{
			cout << "more then 3 red ball" << endl;
			cnt_r[3] ++;

			for(int r = 0; r < 3; r++)
			{
					r_temp_x[r] += r_ball_X[r];
					r_temp_y[r] += r_ball_Y[r];
			}
			previous_r = 3;
		}
		else if(red_ball_number==2 && cnt_r[3]==0)
		{
			cout << "2 red ball" << endl;
			cnt_r[2]++;
			for(int r = 0; r < red_ball_number; r++)
			{
					r_temp_x[r] += r_ball_X[r];
					r_temp_y[r] += r_ball_Y[r];
			}

			r_temp_x[2] = 0;
			r_temp_y[2] = -100;

			previous_r = 2;
		}
		else if(red_ball_number==1 && cnt_r[3]==0 && cnt_r[2]==0)
		{
			cout << "1 red ball" << endl;
			cnt_r[1]++;
			for(int r = 0; r < red_ball_number; r++)
			{
					r_temp_x[r] += r_ball_X[r];
					r_temp_y[r] += r_ball_Y[r];
			}

			r_temp_x[1] = 0;
			r_temp_y[1] = -100;
			r_temp_x[2] = 0;
			r_temp_y[2] = -100;

			previous_r = 1;
		}
		else if(red_ball_number==0 && cnt_r[3]==0 && cnt_r[2]==0 && cnt_r[1]==0)
		{
			cnt_r[0]++;

			r_temp_x[0] = 0;
			r_temp_y[0] = -100;
			r_temp_x[1] = 0;
			r_temp_y[1] = -100;
			r_temp_x[2] = 0;
			r_temp_y[2] = -100;

			previous_r = 0;
		}
		else
		{
			cout << "red ball out of range" << endl;
			continue;
		}


		if( (cnt_b2>3 || (cnt_b2==0 && cnt_b3>3)) && cnt_r[previous_r]>3) break;

	}




	for(int r = 0; r < previous_r; r++)
	{
			r_ball_x[r] = r_temp_x[r]/cnt_r[previous_r];
			r_ball_y[r] = r_temp_y[r]/cnt_r[previous_r];
	}


  if(previous_b == 3)
	{
		for(int b = 0; b < 3; b++)
		{
				b_ball_x[b] = b_temp_x[b]/cnt_b3;
				b_ball_y[b] = b_temp_y[b]/cnt_b3;
		}
		pathmode = 2;
	}
	else if(previous_b == 2)
	{
		for(int b = 0; b < 2; b++)
		{
				b_ball_x[b] = b_temp_x[b]/cnt_b2;
				b_ball_y[b] = b_temp_y[b]/cnt_b2;
		}
		b_ball_x[2] = 0;
		b_ball_y[2] = 0;

		pathmode = ballcovered;
	}
	else if(previous_b == -1)
	{
		for(int b = 0; b < 3; b++)
		{
				b_ball_x[b] = -100;
				b_ball_y[b] = -100;
		}
		cout << "can't find blue ball position" << endl;
	}
	else
	{
		cout << "unknown error" << endl;
	}


	cout << "ball position (x, y) [cm]" << endl;
	cout << "blue 0 = ( "<< b_ball_x[0] << ", " << b_ball_y[0] << " )" <<endl;
	cout << "blue 1 = ( "<< b_ball_x[1] << ", " << b_ball_y[1] << " )" <<endl;
	cout << "blue 2 = ( "<< b_ball_x[2] << ", " << b_ball_y[2] << " )" <<endl;

	cout << "red  0 = ( "<< r_ball_x[0] << ", " << r_ball_y[0] << " )" <<endl;
	cout << "red  1 = ( "<< r_ball_x[1] << ", " << r_ball_y[1] << " )" <<endl;
	cout << "red  2 = ( "<< r_ball_x[2] << ", " << r_ball_y[2] << " )" <<endl;


	cout<<"cor_init end"<<endl;
}

double pdis(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

double inner(double x1, double y1, double x2, double y2)
{
	return x1*x2 + y1*y2;
}

double mindis(double xa, double ya, double xb, double yb, double xp, double yp)
{
	//cout<<xa<<","<<ya<<"    "<<xb<<","<<yb<<"    "<<xp<<","<<yp<<endl;
	double xap = xp-xa;  double yap = yp-ya;
	double xab = xb-xa;  double yab = yb-ya;
	double dap = pdis(0, 0, xap, yap); double dab = pdis(0, 0, xab, yab);
	double cos = inner(xap, yap, xab, yab)/dap/dab;

	if(cos<0) return dap;
	else if(cos*dap>dab) return pdis(xp, yp, xb, yb);
	else return sqrt(1-pow(cos, 2))*dap;
}

bool b_ball_in_path(int b1, int b2, int b3)
{
	double dis = mindis(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2]);
	if(dis < saferange) { /*cout<<dis<<"----1"<<endl;*/ return true; }

	dis = mindis(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b3], b_ball_y[b3]);
	if(dis < saferange) { /*cout<<dis<<"----2"<<endl;*/ return true; }

	dis = mindis(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3]);
	if(dis < saferange) { /*cout<<dis<<"----3"<<endl;*/ return true; }

	return false;
}

double angle(double x1, double y1, double x2, double y2, double x3, double y3)
{
	// double in = inner(x2-x1, y2-y1, x3-x2, y3-y2);
	// double dis12 = pdis(x1, y1, x2, y2);
	// double dis23 = pdis(x2, y2, x3, y3);
	// double theta = acos(in/dis12/dis23)*180/PI;

	double the12 = atan2(y2-y1, x2-x1);
	double the23 = atan2(y3-y2, x3-x2);
	double theta = (the23-the12)*180/M_PI;
	if(-theta>180){
		theta = theta + 360;
	}
	else if(-theta < -180){
		theta = theta - 360;
	}
	return -theta;    // ccw : minus, cw : plus
}

double pathgen(int b1, int b2, int b3)
{
	cout << "pathgen_start" << endl;
	double t = 0;

	if(b_ball_in_path(b1, b2, b3))
	{	t = t_max;
		return t;	}

	t+=pdis(base_x, base_y, b_ball_x[b1], b_ball_y[b1])/straight;
	t+=angle(base_x, base_y-10, base_x, base_y, b_ball_x[b1], b_ball_y[b1])/rotate_cost;
	for(int i=0; i<3; i++)
	{
		double dis = mindis(base_x, base_y, b_ball_x[b1], b_ball_y[b1], r_ball_x[i], r_ball_y[i]);
		if(dis<saferange)
		{
			double diff = saferange-dis;
			t+=diff*2*sqrt(2)/diagonal;
			t-=diff*2/straight;
			t+=stood*2;
			t+=dtod;
		}
	}

	t+=pdis(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2])/straight;
	t+=angle(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2])/rotate_cost;
	for(int i=0; i<3; i++)
	{
		double dis = mindis(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2], r_ball_x[i], r_ball_y[i]);
		if(dis<saferange)
		{
			double diff = saferange-dis;
			t+=diff*2*sqrt(2)/diagonal;
			t-=diff*2/straight;
			t+=stood*2;
			t+=dtod;
		}
	}

	t+=pdis(b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3])/straight;
	t+=angle(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3])/rotate_cost;
	for(int i=0; i<3; i++)
	{
		double dis = mindis(b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3], r_ball_x[i], r_ball_y[i]);
		if(dis<saferange)
		{
			double diff = saferange-dis;
			t+=diff*2*sqrt(2)/diagonal;
			t-=diff*2/straight;
			t+=stood*2;
			t+=dtod;
		}
	}

	t+=pdis(b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep)/straight;
	t+=angle(b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep)/rotate_cost;
	for(int i=0; i<3; i++)
	{
		double dis = mindis(b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep, r_ball_x[i], r_ball_y[i]);
		if(dis<saferange)
		{
			double diff = saferange-dis;
			t+=diff*2*sqrt(2)/diagonal;
			t-=diff*2/straight;
			t+=stood*2;
			t+=dtod;
		}
	}

	t+=angle(b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep, base_x, base_y-10)/rotate_cost;

	return t;

}

void pathgen2()
{
	cout << "pathgen for covered blue ball" << endl;
	int b1, b2, b3;
	if(pathmode==0)
	{
		b1 = 0; b2 = 2; b3 = 1;
	}
	else
	{
		b1 = 1; b2 = 2; b3 = 0;
	}

	angle1 = angle(base_x, base_y-10, base_x, base_y, b_ball_x[b1], b_ball_y[b1]);
	angle2 = 0;
	angle3 = angle(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b3], b_ball_y[b3]);
	if(angle3>=0) {angle3=1000;} else {angle3=-1000;}
	if(b_ball_x[b1]>b_ball_x[b3]) {angle4 = -1000;} else{angle4 = 1000;}
	angle5 = angle(b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep, base_x, base_y-10);

	cout << endl;
	cout << "angle1 : " << angle1 << endl;
	cout << "angle2 : " << angle2 << endl;
	cout << "angle3 : " << angle3 << endl;
	cout << "angle4 : " << angle4 << endl;
	cout << "angle5 : " << angle5 << endl;
}

int tcal()
{
	tcost[0] = pathgen(0, 1, 2);
	tcost[1] = pathgen(0, 2, 1);
	tcost[2] = pathgen(1, 0, 2);
	tcost[3] = pathgen(1, 2, 0);
	tcost[4] = pathgen(2, 0, 1);
	tcost[5] = pathgen(2, 1, 0);

  cout << endl;
	cout << "tcost per path" << endl;
	cout << "0, 1, 2 : t = "<< tcost[0] << endl;
	cout << "0, 2, 1 : t = "<< tcost[1] << endl;
	cout << "1, 0, 2 : t = "<< tcost[2] << endl;
	cout << "1, 2, 0 : t = "<< tcost[3] << endl;
	cout << "2, 0, 1 : t = "<< tcost[4] << endl;
	cout << "2, 1, 0 : t = "<< tcost[5] << endl;

	double tmin = tcost[0];
	int imin = 0;
	for(int i=1; i<6; i++)
	{
		if(tcost[i]<tmin)
		{
			imin = i;
			tmin = tcost[i];
		}
	}
	return imin;
}

void pathgen_call()
{
	check = 0;

	if(pathmode!=2){cout<<"moving to pathgen2"<<endl; pathgen2(); return;}
	int path = tcal();
	cout << endl;
	cout << "best path" << endl;

	int b1, b2, b3;
	switch(path)
	{
		case 0:
			b1 = 0; b2 = 1; b3 = 2;
			break;
		case 1:
			b1 = 0; b2 = 2; b3 = 1;
			break;
		case 2:
			b1 = 1; b2 = 0; b3 = 2;
			break;
		case 3:
			b1 = 1; b2 = 2; b3 = 0;
			break;
		case 4:
			b1 = 2; b2 = 0; b3 = 1;
			break;
		case 5:
			b1 = 2; b2 = 1; b3 = 0;
			break;
	}
	cout << b1 << "("<< b_ball_x[b1] << ", " << b_ball_y[b1] << ") => " ;
	cout << b2 << "("<< b_ball_x[b2] << ", " << b_ball_y[b2] << ") => " ;
	cout << b3 << "("<< b_ball_x[b3] << ", " << b_ball_y[b3] << ")" << endl;

	angle1 = angle(base_x, base_y-10, base_x, base_y, b_ball_x[b1], b_ball_y[b1]);
	angle2 = angle(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2]);
	angle3 = angle(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3]);
	angle4 = angle(b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3], base_x, base_y);
	angle5 = angle(b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep, base_x, base_y-10);

	if(angle1<5 && angle1>-5)
	{
		angle1 =0;
	}


	double tan1 = atan2(b_ball_y[b1], b_ball_x[b1]);
	double tan2 = atan2(b_ball_y[b2], b_ball_x[b2]);
	double tan3 = atan2(b_ball_y[b3], b_ball_x[b3]);

	if(tan1<tan2 && tan1<tan3)
	{
		angle1+=5;
	}
	else if(tan1>tan2 && tan1>tan3)
	{
		angle1-=5;
	}
	else if(angle1<7 && angle1>-7)
	{
		angle1 = 0;
	}

  cout << endl;
	cout << "angle1 : " << angle1 << endl;
	cout << "angle2 : " << angle2 << endl;
	cout << "angle3 : " << angle3 << endl;
	cout << "angle4 : " << angle4 << endl;
	cout << "angle5 : " << angle5 << endl;
	txtFile << "angle1 : " << angle1 << endl;
	txtFile << "angle2 : " << angle2 << endl;
	txtFile << "angle3 : " << angle3 << endl;
	txtFile << "angle4 : " << angle4 << endl;
	txtFile << "angle5 : " << angle5 << endl;
	txtFile << "b1_x : " << b_ball_x[b1] << "\t" << "b1_y : " << b_ball_y[b1] << endl;
	txtFile << "b2_x : " << b_ball_x[b2] << "\t" << "b2_y : " << b_ball_y[b2] << endl;
	txtFile << "b3_x : " << b_ball_x[b3] << "\t" << "b3_y : " << b_ball_y[b3] << endl;
}


void rotate(int angle){
	if(rotated==0){		//rotate angle1
		if(angle>0){
			for(int i=0; i<(angle*0.25+0.5)*3; i++){		//myRIO : max 30 -> 90'=138 || max 50 -> 150'=138
				turn_right_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		else if(angle<0){
			for(int i=0; i<(-angle*0.25+0.5)*3; i++){
				turn_left_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		else{
		}
	}
	else{		//rotate angle2~4
		if(angle>0){
			for(int i=0; i<angle*0.25 + 0.5 + lTheta/3 - rTheta/3; i++){		//myRIO : max 30 -> 90'=138 || max 50 -> 150'=138
				turn_right();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		else{
			for(int i=0; i< -angle*0.25 +0.5 - lTheta/3 + rTheta/3; i++){
				turn_left();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
	}
	lTheta=0;
	rTheta=0;
	stop();
	write(c_socket, data, sizeof(data));
	ros::Duration(0.05).sleep();
	ros::spinOnce();
	//
	// while(1)
	// {
	// 	ros::spinOnce();
	// 	if(blue_ball_)
	// }

	cout << "stop" << endl;
	rotated++;
}

void releaseAlign(){
	double x1;
	double x2;
	double y1;
	double y2;
	int lineGreen = 300;
	txtFile << "\nCallback" << endl;


	if(g_ball_X[0]<g_ball_X[1]){
		x1 = g_ball_X[0];
		x2 = g_ball_X[1];
		y1 = g_ball_Y[0];
		y2 = g_ball_Y[1];
	}
	else{
		x1 = g_ball_X[1];
		x2 = g_ball_X[0];
		y1 = g_ball_Y[1];
		y2 = g_ball_Y[0];
	}

	if(green_ball_number==0){
		// when both of green balls are not detected
		if(findGreen <= 136){
			turn_right_medium();
			cout<<"1"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
			findGreen++;
		}
		else if(findGreen > 136){
			turn_right_slow();
			cout<<"1"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
			findGreen++;
		}
	}

	// when there is red ball on the path, go to avoid RedBall.
	else if(red_ball_number!=0 && r_ball_X[minR]<10 && r_ball_X[minR]>-10 && r_ball_Y[minR] < lineRed && y1+y2 > lineGreen){
		avoid_RedBall();
	}

	else if(green_ball_number==1){
		if(g_ball_X[0]<-40){
			turn_left_slow();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
		else if(g_ball_X[0]>40){
			turn_right_slow();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
		else{
			move_forward();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
	}

	else if(y1+y2>lineGreen){		//>1.5m
		if(y1>y2){
			if(x1<-25){
				turn_left_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
			else if(x1>25){
				turn_right_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
			else{
				move_forward();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		else if(y1<y2){
			if(x2<-25){
				turn_left_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
			else if(x2>25){
				turn_right_slow();
				cout<<"3"<<endl;
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
			else{
				move_forward();
				cout<<"11"<<endl;
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
	}


	else if(y1+y2<lineGreen && y1+y2>120){			// 0.6m~1.5m
		cout<<"120~300"<<endl;
		if(y1>y2+3){
			turn_right_very_slow();
			cout<<"7"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
		else if(y2>y1+3){
			turn_left_very_slow();
			cout<<"8"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}

		else if(x1+x2<-4){
			move_left_medium();
			cout<<"5"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
		else if(x1+x2>4){
			move_right_medium();
			cout<<"6"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
		else if(x1+x2<=4 && x1+x2>=-4 && y1+y2>120){
			move_forward_medium();
			cout<<"over 120 move forward"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
	}

	else{			//drop ball
		txtFile<<"left green ball x : " << x1 << "\t" << "left green ball y : " << y1 << endl;
		txtFile<<"right green ball x : " << x2 << "\t" << "right green ball y : " << y2 << endl;

		lift();
		stop();
		write(c_socket, data, sizeof(data));
		ros::Duration(0.7).sleep();

		cout<<"<120"<<endl;
		for(int i=0; i<37; i++){
			move_forward_slow();
			cout<<"forcefully move"<<endl;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
		}
		stop();
		write(c_socket, data, sizeof(data));
		ros::Duration(0.05).sleep();

		for(int i=0; i<80; i++){
			roller_back();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
		}
		collect++;
	}
}

void no_red_ball(){         // Define if there is no red ball
	if(blue_ball_number==0){
		cout<<"no blue ball"<<endl;
		turn_left();
		txtFile << "no blue ball, turn left" << endl;
		write(c_socket, data, sizeof(data));
	  ros::Duration(0.05).sleep();
	  ros::spinOnce();
		return no_red_ball();
	}

	// move closer to the blue ball
	if(-15 <= b_ball_X[cenB] && b_ball_X[cenB]<= 15 && b_ball_Y[cenB]> 100){
	  move_forward();
		cout << "move closer to the blue ball" << endl;
		txtFile << "blue ball detected & move forward" << endl;
	  write(c_socket, data, sizeof(data));
	  ros::Duration(0.05).sleep();
	  ros::spinOnce();
	}

  // move left if blue ball is on left side
  else if(b_ball_X[cenB] < -15 && b_ball_Y[cenB] > 100){
		cout<<"b_ball_X[cenB] = " << b_ball_X[cenB] <<endl;
    move_left();
		txtFile << "blue ball move left" << endl;
    write(c_socket, data, sizeof(data));
    ros::Duration(0.05).sleep();
    ros::spinOnce();
  }

  // move right if blue ball is on right side
  else if(b_ball_X[cenB] > 15 && b_ball_Y[cenB] > 100){
		cout<<"b_ball_X[cenB] = " << b_ball_X[cenB] <<endl;
    move_right();
		txtFile << "blue ball move right" << endl;
    write(c_socket, data, sizeof(data));
    ros::Duration(0.05).sleep();
    ros::spinOnce();
	}

  // when close enough, pick up blue ball
  if(b_ball_Y[cenB] < 100){
		if(b_ball_X[cenB] < -7){
			cout<<"b_ball_X[cenB] = " << b_ball_X[cenB] <<endl;
	    move_left_medium();
			txtFile << "blue ball move left (close)" << endl;
	    write(c_socket, data, sizeof(data));
	    ros::Duration(0.05).sleep();
	    ros::spinOnce();
	  }

	  // move right if blue ball is on right side
	  else if(b_ball_X[cenB] > 7){
			cout<<"b_ball_X[cenB] = " << b_ball_X[cenB] <<endl;
	    move_right_medium();
			txtFile << "blue ball move right (close)" << endl;
	    write(c_socket, data, sizeof(data));
	    ros::Duration(0.05).sleep();
	    ros::spinOnce();
		}
		else if(b_ball_X[cenB] > -7 && b_ball_X[cenB] < 7 && b_ball_Y[cenB] > lineBlue){
			move_forward();
			txtFile << "blue ball on center(move forward)" << endl;
	    write(c_socket, data, sizeof(data));
	    ros::Duration(0.05).sleep();
	    ros::spinOnce();
		}
		else if(b_ball_X[cenB] > -7 && b_ball_X[cenB] < 7 && b_ball_Y[cenB] < lineBlue && b_ball_Y[cenB] > 0){
			txtFile << "blue ball x : " << b_ball_X[cenB] << "\t" << "blue ball y : " << b_ball_Y[cenB] << endl;
			for(int i=0; i<13; i++){
				cout << "move forcefully" << endl;
				cout << "b_ball_Y[cenB]" << b_ball_Y[cenB] << endl;
				txtFile << "move forcefully to eat blue ball" << endl;
				move_forward();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
			cout << "collect complete"<< endl;
			collect++;
			txtFile << "collect = " << collect << endl;
		}

		if(collect==4){
			rotate(angle4);
			ros::Duration(0.5).sleep();
			ros::spinOnce();
		}
  }
}

void avoid_RedBall(){       //Condition: closest red ball is already on center line.
	cout << "Avoid Redball Initiate"<< endl;

  //----------red ball is not close enough, move closer
	if(blue_ball_number==0 && collect!=4){
		no_red_ball();
		return;
	}

  if(r_ball_Y[minR] > lineRed){
    move_forward();
		txtFile << "red ball faraway && move forward" << endl;
    write(c_socket, data, sizeof(data));
    ros::Duration(0.05).sleep();
    ros::spinOnce();
		return avoid_RedBall();
  }

  //----------left avoid (move left, move forward, move back right
  else if(r_ball_X[minR] > 0 && r_ball_X[minR] < 30 && r_ball_Y[minR] <= lineRed){
		txtFile << "avoid red ball address" << red_ball_number<< endl;
		txtFile << "red ball x : " << r_ball_X[minR] << "\t" << "red ball y : " << r_ball_Y[minR] << endl;

		if(r_ball_Y[minR]<50){
			txtFile << "under 50" << endl;
			for(int i=0; i<32; i++){
				left_forward();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		else{
			txtFile << "over 50" << endl;
			for(int i=0; i<43; i++){
				forward_left();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}


		if(collect==4){
			return releaseAlign();
		}

		for(int i=0; i<2; i++){
			turn_right_slow();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
			rTheta+=2;
		}

		if(blue_ball_number==0){
			return no_red_ball();
		}

		while(1){
			int i=0;
			while(i<blue_ball_number){
				if(b_ball_X[i]<0){
					i++;
				}
				else{
					break;
				}
			}
			if(b_ball_X[i] > 7){
				turn_right_very_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
				rTheta++;
			}
			else{
				break;
			}
		}
		stop();
		write(c_socket, data, sizeof(data));
		ros::Duration(0.05).sleep();
		ros::spinOnce();

		txtFile << "after avoid, blue ball address" << endl;
		txtFile << "blue ball x : " << b_ball_X[cenB] << "\t" << "blue ball y : " << b_ball_Y[cenB] << endl;
    no_red_ball();
	}

    //----------right avoid (move right, move forward, move back left
  else if(r_ball_X[minR] < 0 && r_ball_X[minR] > -30 && r_ball_Y[minR] <= lineRed){
		txtFile << "avoid red ball address" << red_ball_number<< endl;
		txtFile << "red ball x : " << r_ball_X[minR] << "\t" << "red ball y : " << r_ball_Y[minR] << endl;

		if(r_ball_Y[minR]<50){
			txtFile << "under 50" << endl;
			for(int i=0; i<32; i++){
				right_forward();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		else{
			txtFile << "over 50" << endl;
			for(int i=0; i<43; i++){
				forward_right();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
			}
		}
		if(collect==4){
			return releaseAlign();
		}

		for(int i=0; i<2; i++){
			turn_left_slow();
			write(c_socket, data, sizeof(data));
			ros::Duration(0.05).sleep();
			ros::spinOnce();
			lTheta+=2;
		}

		if(blue_ball_number==0){
			return no_red_ball();
		}

		while(1){
			int i=0;
			while(i<blue_ball_number){
				if(b_ball_X[i]>0){
					i++;
				}
				else{
					break;
				}
			}
			if(b_ball_X[i] < -7){
				turn_left_very_slow();
				write(c_socket, data, sizeof(data));
				ros::Duration(0.05).sleep();
				ros::spinOnce();
				lTheta++;
			}
			else{
				break;
			}
		}
		stop();
		write(c_socket, data, sizeof(data));
		ros::Duration(0.05).sleep();
		ros::spinOnce();

		txtFile << "after avoid, blue ball address" << endl;
		txtFile << "blue ball x : " << b_ball_X[cenB] << "\t" << "blue ball y : " << b_ball_Y[cenB] << endl;
    no_red_ball();
  }
}

void PickUp(){
	if(collect==4){
		releaseAlign();
	}
	else{
		if(collect==rotated){
			cout << "PickUp Initiate"<< endl;
			cout << "blue ball number = " << blue_ball_number << endl;
			cout << "red ball number = " << red_ball_number << endl;
			txtFile << "blue ball number = " << blue_ball_number << endl;
			txtFile << "red ball number = " << red_ball_number << endl;

			if(red_ball_number!=0 && r_ball_Y[minR] < b_ball_Y[minB]){
				if(r_ball_X[minR] < 21 && r_ball_X[minR] > -21){
					avoid_RedBall();
				}
				else{
					no_red_ball();
				}
			}
			else{
				no_red_ball();
			}
		}

		else{
			if(rotated==0){
				rotate(angle1);
				ros::Duration(0.1).sleep();
				ros::spinOnce();
				cout<<"rotate(angle1)"<<endl;
				txtFile << "rotate(angle1) = " << angle1 << endl;
				ros::Duration(0.05).sleep();
				ros::spinOnce();
				txtFile << " blue ball number : " << blue_ball_number << endl;
				txtFile << "[cenB] : " << cenB << endl;
				txtFile << "b_ball_X[cenB] : " << b_ball_X[cenB] << endl;
				txtFile << "b_ball_Y[cenB] = " << b_ball_Y[cenB] << endl;

				while(1){
					if(blue_ball_number==0){
						return no_red_ball();
					}
					else{
						break;
					}

					//--------align first blue ball

					// else if(b_ball_X[cenB]>7){
					// 	turn_right_very_slow();
					// 	write(c_socket, data, sizeof(data));
					// 	ros::Duration(0.05).sleep();
					// 	ros::spinOnce();
					// }
					// else if(b_ball_X[cenB]<-7){
					// 	turn_left_very_slow();
					// 	write(c_socket, data, sizeof(data));
					// 	ros::Duration(0.05).sleep();
					// 	ros::spinOnce();
					// }
					// else{
					// 	break;
					// }
				}
			}
			else if(rotated==1){
				rotate(angle2);
				ros::Duration(0.1).sleep();
				ros::spinOnce();
				cout<<"rotate(angle2)"<<endl;
				txtFile << "rotate(angle2) = " << angle2 << endl;
				ros::Duration(0.05).sleep();
				ros::spinOnce();
				txtFile << " blue ball number : " << blue_ball_number << endl;
				txtFile << "[cenB] : " << cenB << endl;
				txtFile << "b_ball_X[cenB] : " << b_ball_X[cenB] << endl;
				txtFile << "b_ball_Y[cenB] = " << b_ball_Y[cenB] << endl;

				while(1){
					if(blue_ball_number==0){
						return no_red_ball();
					}
					else if(b_ball_X[cenB]>7){
						turn_right_very_slow();
						write(c_socket, data, sizeof(data));
						ros::Duration(0.05).sleep();
						ros::spinOnce();
					}
					else if(b_ball_X[cenB]<-7){
						turn_left_very_slow();
						write(c_socket, data, sizeof(data));
						ros::Duration(0.05).sleep();
						ros::spinOnce();
					}
					else{
						break;
					}
				}
			}
			else if(rotated==2){
				rotate(angle3);
				ros::Duration(0.1).sleep();
				ros::spinOnce();
				cout<<"rotate(angle3)"<<endl;
				txtFile << "rotate(angle3) = " << angle3 << endl;
				ros::Duration(0.05).sleep();
				ros::spinOnce();
				txtFile << " blue ball number : " << blue_ball_number << endl;
				txtFile << "[cenB] : " << cenB << endl;
				txtFile << "b_ball_X[cenB] : " << b_ball_X[cenB] << endl;
				txtFile << "b_ball_Y[cenB] = " << b_ball_Y[cenB] << endl;

				while(1){
					if(blue_ball_number==0){
						return no_red_ball();
					}
					else if(b_ball_X[cenB]>7){
						turn_right_very_slow();
						write(c_socket, data, sizeof(data));
						ros::Duration(0.05).sleep();
						ros::spinOnce();
					}
					else if(b_ball_X[cenB]<-7){
						turn_left_very_slow();
						write(c_socket, data, sizeof(data));
						ros::Duration(0.05).sleep();
						ros::spinOnce();
					}
					else{
						break;
					}
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
	  cout << "main start"<< endl;
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    // ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);

		dataInit();


		int i=1;
		while(i==1)
		{
			ros::Duration(0.05).sleep();
			ros::spinOnce();
			// cor_show();
			if(blue_ball_number>0) break;
		}
		cor_init();

		cout<< "moving to pathgen_call" <<endl;
		pathgen_call();

		if(check == -1){
			cout << "pathgen_error" << endl;
			return -1;
		}

		c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
        printf("Failed to connect\n");
        close(c_socket);
        return -1;
    }

		while(ros::ok)
		{


			PickUp();
			// releaseAlign();
			// avoid_RedBall();
			cout<<"greenball number = " << green_ball_number << endl;
			cout<<"collect = " << collect << endl;
			txtFile<<"collect = " << collect << endl;

			if(collect==5){
				cout<<"collect = 5"<<endl;
				break;
			}
		}
		txtFile.close();
		return 0;
}
