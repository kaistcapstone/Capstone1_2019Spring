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
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"

using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress

boost::mutex map_mutex;

// long distance camera
int ball_number_1_red;
int ball_number_1_blue;
int ball_number_1_green;
float ball_X_1_red[20];
float ball_Y_1_red[20];
float ball_X_1_blue[20];
float ball_Y_1_blue[20];
float ball_X_1_green[20];
float ball_Y_1_green[20];
float ball_distance_1_red[20];
float ball_distance_1_blue[20];
float ball_distance_1_green[20];

// short distance camera
int ball_number_2_red;
int ball_number_2_blue;
int ball_number_2_green;
float ball_X_2_red[20];
float ball_Y_2_red[20];
float ball_X_2_blue[20];
float ball_Y_2_blue[20];
float ball_X_2_green[20];
float ball_Y_2_green[20];
float ball_distance_2_red[20];
float ball_distance_2_blue[20];
float ball_distance_2_green[20];

// back camera
int ball_number_3_green;
float ball_X_3_green[20];
float ball_Y_3_green[20];
float ball_distance_3_green[20];


int c_socket, s_socket;
struct sockaddr_in c_addr;
float data[24];

#define RAD2DEG(x) ((x)*180./M_PI)

// new variables
int robot_state = 0;
int global_count = 0;
int global_state = 0;
int global_state2 = 0;

int exist_red[6];
int distance_between_blue_balls[6];


void dataInit()
{
	data[0] = 0; //lx*data[3];ey’
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
void camera1_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  int count_red = position -> size_red;
  int count_blue = position -> size_blue;
  int count_green = position -> size_green;

  ball_number_1_red=position->size_red;
  ball_number_1_blue=position->size_blue;
  ball_number_1_green=position->size_green;
  for(int i = 0; i < count_red; i++)
  {
    ball_X_1_red[i] = position->img_x_red[i];
    ball_Y_1_red[i] = position->img_y_red[i];
    ball_distance_1_red[i] = ball_X_1_red[i]*ball_X_1_red[i]+ball_Y_1_red[i]*ball_Y_1_red[i];
  }
  for(int i = 0; i < count_blue; i++)
  {
    ball_X_1_blue[i] = position->img_x_blue[i];
    ball_Y_1_blue[i] = position->img_y_blue[i];
    ball_distance_1_blue[i] = ball_X_1_blue[i]*ball_X_1_blue[i]+ball_Y_1_blue[i]*ball_Y_1_blue[i];
  }
  for(int i = 0; i < count_green; i++)
  {
    ball_X_1_green[i] = position->img_x_green[i];
    ball_Y_1_green[i] = position->img_y_green[i];
    ball_distance_1_green[i] = ball_X_1_green[i]*ball_X_1_green[i]+ball_Y_1_green[i]*ball_Y_1_green[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl
  }
}
void camera2_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  int count_red = position -> size_red;
  int count_blue = position -> size_blue;
  int count_green = position -> size_green;

  ball_number_2_red=position->size_red;
  ball_number_2_blue=position->size_blue;
  ball_number_2_green=position->size_green;
  for(int i = 0; i < count_red; i++)
  {
    ball_X_2_red[i] = position->img_x_red[i];
    ball_Y_2_red[i] = position->img_y_red[i];
    ball_distance_2_red[i] = ball_X_2_red[i]*ball_X_2_red[i]+ball_Y_2_red[i]*ball_Y_2_red[i];
  }
  for(int i = 0; i < count_blue; i++)
  {
    ball_X_2_blue[i] = position->img_x_blue[i];
    ball_Y_2_blue[i] = position->img_y_blue[i];
    ball_distance_2_blue[i] = ball_X_2_blue[i]*ball_X_2_blue[i]+ball_Y_2_blue[i]*ball_Y_2_blue[i];
  }
  for(int i = 0; i < count_green; i++)
  {
    ball_X_2_green[i] = position->img_x_green[i];
    ball_Y_2_green[i] = position->img_y_green[i];
    ball_distance_2_green[i] = ball_X_2_green[i]*ball_X_2_green[i]+ball_Y_2_green[i]*ball_Y_2_green[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl
  }
}
void camera3_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  int count_green = position -> size_green;
  ball_number_3_green = position->size_green;
  for(int i = 0; i < count_green; i++)
  {
    ball_X_3_green[i] = position->img_x_green[i];
    ball_Y_3_green[i] = position->img_y_green[i];
    ball_distance_3_green[i] = ball_X_3_green[i] * ball_X_3_green[i] + ball_Y_3_green[i] * ball_Y_3_green[i];
  }
}

// function list
void robot_move_forward(float velocity)
{
  data[1] = velocity;
}
void robot_move_back(float velocity)
{
  data[1] = -velocity;
}
void robot_move_left(float velocity)
{
  data[4] = -velocity;
}
void robot_move_right(float velocity)
{
  data[4] = velocity;
}
void robot_rotate_CW(float velocity)
{
  data[0] = velocity;
}
void robot_rotate_CCW(float velocity)
{
  data[0] = -velocity;
}
void robot_pick_ball(float velocity)
{
  data[5] = velocity;
}
void robot_exit_ball()
{
  data[8] = 1;
}
void robot_move_arc_CCW(float x)
{
  float threshold = 0.2;
  if (std::abs(x) < threshold)
  {
    robot_move_right(0.5);
    robot_rotate_CCW(0.2);
    return;
  }
  else if (x < -threshold)
  {
    robot_move_right(0.5);
		robot_rotate_CCW(0.3);
    return;
  }
  else if (x > threshold)
  {
    robot_move_right(0.5);
    return;
  }
}
void robot_move_arc_CW(float x)
{
  float threshold = 0.2;
  if (std::abs(x) < threshold)
  {
    robot_move_right(0.5);
    robot_rotate_CW(0.2);
    return;
  }
  else if (x < -threshold)
  {
    robot_move_left(0.5);
    return;
  }
  else if (x > threshold)
  {
    robot_move_left(0.5);
		robot_rotate_CW(0.3);
    return;
  }
}



void data_print()
{
	std::cout << "data = [" << data[1] << ", " << data[4] << ", " << data[0] << ", " << data[5] <<", " << data[8] << "]"<< std::endl;
}
int robot_wait_second(float wait_time)
{
  int mult = 100;
  if (global_count > wait_time * mult)
  {
    global_count = 0;
    return 1;
  }
  dataInit();
  global_count++;
  return 0;
}



int robot_find_blue_ball_CW()
{
  if ((ball_number_1_blue != 0) || (ball_number_2_blue != 0))
  {
    return 1;
  }
	dataInit();
  robot_rotate_CW(1);
  return 0;
}

int robot_find_blue_ball_CCW()
{
  if ((ball_number_1_blue != 0) || (ball_number_2_blue != 0))
  {
    return 1;
  }
	dataInit();
  robot_rotate_CCW(1);
  return 0;
}




bool is_blue_ball_in_threshold(float threshold)
{
	float m = 10;
	float neck = 0.7;
	if (ball_number_2_blue > 0){
		if (ball_Y_2_blue[0] < neck){
			if (std::abs(ball_X_2_blue[0]) < threshold){return 1;}
			else if (std::abs(ball_X_2_blue[0]) > threshold){return 0;}
			return 0;
		}
		else if (ball_Y_2_blue[0] > neck){
			if (ball_Y_2_blue[0] > -m * (ball_X_2_blue[0] + threshold) + neck && ball_Y_2_blue[0] > m * (ball_X_2_blue[0] - threshold) + neck){return 1;}
			else{return 0;}
		}
		return 0;
	}
	else if (ball_number_1_blue > 0){
		if (ball_Y_1_blue[0] < neck){
			if (std::abs(ball_X_1_blue[0]) < threshold){
				return 1;
			}
			else if(std::abs(ball_X_1_blue[0]) > threshold){
				return 0;
			}
			return 0;
		}
		else if (ball_Y_1_blue[0] > neck){
			if (ball_Y_1_blue[0] > -m*(ball_X_1_blue[0] + threshold) + neck && ball_Y_1_blue[0] > m*(ball_X_1_blue[0] - threshold) + neck){
				return 1;
			}
			else{return 0;}
		}
		return 0;
	}
	return 0;
}
bool is_red_ball_in_threshold(float red_threshold)
{
	// blue ball in camera 2
	if (ball_number_2_blue > 0){
		// red ball in camera 2
		if (ball_number_2_red > 0){
			// blue ball is close
			if (ball_Y_2_red[0] > ball_Y_2_blue[0]){return 0;}
			// red ball is close
			else if (ball_Y_2_red[0] < ball_Y_2_blue[0]){
				if (std::abs(ball_X_2_red[0]) < red_threshold && ball_Y_2_red[0] < 0.6){return 1;}
				return 0;
			}
			return 0;
		}
		// red ball in camera 1
		else if (ball_number_1_red > 0){return 0;}
		return 0;
	}
	// blue ball in camera 1
	else if (ball_number_1_blue > 0){
		// red ball in camera 2
		if (ball_number_2_red > 0){
			if (std::abs(ball_X_2_red[0]) < red_threshold){return 1;}
			return 0;
		}
		// red ball in camera 1
		else if (ball_number_1_red > 0){
			// blue ball is close
			if (ball_Y_1_blue[0] < ball_Y_1_red[0]){
				return 0;
			}
			// red ball is close
			else if (ball_Y_1_blue[0] > ball_Y_1_red[0]){
				if (std::abs(ball_X_1_red[0]) < red_threshold && ball_Y_1_red[0] < 0.6){return 1;}
				return 0;
			}
		}
		return 0;
	}
	return 0;
}
int is_cam1_detect_red_ball_in_threshold(float red_threshold)
{
	int i;

	if (ball_number_1_red == 0){return 0;}
	else{
		for (i = 0; i < ball_number_1_red; i++){
			if (std::abs(ball_X_1_red[i]) < red_threshold && ball_Y_1_red[i] < 0.6){
				return (i+1);
			}
		}
		return 0;
	}
}
int is_cam2_detect_red_ball_in_threshold(float red_threshold)
{
	int i;

	if (ball_number_2_red == 0){return 0;}
	else{
		for (i = 0; i < ball_number_2_red; i++){
			if (std::abs(ball_X_2_red[i]) < red_threshold && ball_Y_2_red[i] < 0.6){
				return (i+1);
			}
		}
		return 0;
	}
}



void robot_avoid_red_ball(float red_threshold)
{
	// cam 2 detects red ball
	if (is_cam2_detect_red_ball_in_threshold(red_threshold) > 0){
		std::cout << "if (is_cam2_detect_red_ball_in_threshold)" << std::endl;
		if (ball_X_2_red[is_cam2_detect_red_ball_in_threshold(red_threshold)-1] > 0){
			std::cout << "if (ball_X_2_red[~~~~~~ -1] > 0)" << std::endl;
			dataInit();
			if (ball_Y_2_red[is_cam2_detect_red_ball_in_threshold(red_threshold)-1] > 0.5){robot_move_forward(0.5);}
			robot_move_left(0.5);
			if (ball_number_2_blue != 0){
				if (ball_X_2_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_2_blue[0] < 0){robot_rotate_CCW(0.111);}
			}
			else if (ball_number_1_blue != 0){
				if (ball_X_1_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_1_blue[0] < 0){robot_rotate_CCW(0.111);}
			}
			return;
		}
		else if (ball_X_2_red[is_cam2_detect_red_ball_in_threshold(red_threshold)-1] < 0){
			std::cout << "else if (ball_X_2_red[~~~~~~ -1] < 0)" << std::endl;
			dataInit();
			if (ball_Y_2_red[is_cam2_detect_red_ball_in_threshold(red_threshold)-1] > 0.5){robot_move_forward(0.5);}
			robot_move_right(0.5);
			if (ball_number_2_blue != 0){
				if (ball_X_2_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_2_blue[0] < 0){robot_rotate_CCW(0.111);}
			}
			else if (ball_number_1_blue != 0){
				if (ball_X_1_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_1_blue[0] < 0){robot_rotate_CCW(0.111);}
			}
			return;
		}
		else{
			std::cout << "Error: enter else 1" << std::endl;
			return;
		}
		return;
	}
	// cam 1 detects red ball
	else if (is_cam1_detect_red_ball_in_threshold(red_threshold) > 0){
		std::cout << "else if (is_cam1_detect_red_ball_in_threshold)" << std::endl;
		if (ball_X_1_red[is_cam1_detect_red_ball_in_threshold(red_threshold)-1] > 0){
			std::cout << "if (ball_X_1_red[~~~~~~ -1] > 0)" << std::endl;
			dataInit();
			if (ball_Y_1_red[is_cam1_detect_red_ball_in_threshold(red_threshold)-1] > 0.5){robot_move_forward(0.5);}
			robot_move_left(0.5);

			if (ball_number_2_blue != 0){
				if (ball_X_2_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_2_blue[0] < 0){robot_rotate_CCW(0.111);}
			}
			else if (ball_number_1_blue != 0){
				if (ball_X_1_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_1_blue[0] < 0){robot_rotate_CCW(0.111);}
			}

			return;
		}
		else if (ball_X_1_red[is_cam1_detect_red_ball_in_threshold(red_threshold)-1] < 0){
			std::cout << "else if (ball_X_1_red[~~~~~~ -1] < 0)" << std::endl;
			dataInit();
			if (ball_Y_1_red[is_cam1_detect_red_ball_in_threshold(red_threshold)-1] > 0.5){robot_move_forward(0.5);}
			robot_move_right(0.5);

			if (ball_number_2_blue != 0){
				if (ball_X_2_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_2_blue[0] < 0){robot_rotate_CCW(0.111);}
			}
			else if (ball_number_1_blue != 0){
				if (ball_X_1_blue[0] > 0){robot_rotate_CW(0.111);}
				else if (ball_X_1_blue[0] < 0){robot_rotate_CCW(0.111);}
			}


			return;
		}
		else{
			std::cout << "enter else 2" << std::endl;
			return;
		}
	}
	else{
		std::cout << "Error : robot_avoid_red_ball()" << std::endl;
		return;
	}
}
void robot_move_new_threshold(float threshold)
{
	// camera 2 detects blue ball
	if (ball_number_2_blue != 0){
		if (ball_number_2_blue == 1 && global_state2 == 1){
			global_state2 = 2;
		}
		if (ball_number_2_blue == 2){
			global_state2 = 1;
		}
		global_state = 1;
		// blue ball is in threshold
		if (is_blue_ball_in_threshold(threshold)){
			if (ball_Y_2_blue[0] > 1){
				dataInit();
				robot_move_forward(1);
				if(ball_X_2_blue[0] > 0){robot_rotate_CW(0.21);}
				else if(ball_X_2_blue[0] < 0){robot_rotate_CCW(0.21);}
				return;
			}
			else if (ball_Y_2_blue[0] > 0.5){
				dataInit();
				robot_move_forward(0.6*(ball_Y_2_blue[0] - 1) + 1);
				if(ball_X_2_blue[0] > 0){robot_rotate_CW(0.21);}
				else if (ball_X_2_blue[0] < 0){robot_rotate_CCW(0.21);}
				return;
			}
			else if (ball_Y_2_blue[0] > 0){
				dataInit();
				robot_move_forward(0.7);
				if(ball_X_2_blue[0] > 0){robot_rotate_CW(0.21);}
				else if (ball_X_2_blue[0] < 0){robot_rotate_CCW(0.21);}
				return;
			}
			return;
		}
		// blue ball is on your right
		else if(ball_X_2_blue[0] > 0){
			if (ball_Y_2_blue[0] > 1){
				dataInit();
				robot_move_forward(1);
				robot_rotate_CW(0.74);
				return;
			}
			else if (ball_Y_2_blue[0] > 0.5){
				dataInit();
				robot_move_forward(0.6*(ball_Y_2_blue[0] - 1) + 1);
				robot_rotate_CW(0.41);
				return;
			}
			else if (ball_Y_2_blue[0] > 0){
				dataInit();
				robot_move_forward(0.7);
				robot_rotate_CW(0.34);
				return;
			}
			return;
		}
		// blue ball is on your left
		else if(ball_X_2_blue[0] < 0){
			if (ball_Y_2_blue[0] > 1){
				dataInit();
				robot_move_forward(1);
				robot_rotate_CCW(0.74);
				return;
			}
			else if (ball_Y_2_blue[0] > 0.5){
				dataInit();
				robot_move_forward(0.6*(ball_Y_2_blue[0] - 1) + 1);
				robot_rotate_CCW(0.41);
				return;
			}
			else if (ball_Y_2_blue[0] > 0){
				dataInit();
				robot_move_forward(0.7);
				robot_rotate_CCW(0.34);
				return;
			}
			return;
		}
		return;
	}
	// camera 1 detects blue ball
	else if (ball_number_1_blue != 0){
		if (is_blue_ball_in_threshold(threshold)){
			if (ball_Y_1_blue[0] > 1){
				dataInit();
				robot_move_forward(1);
				if (ball_X_1_blue[0] > 0.3){robot_rotate_CW(0.3);}
				else if (ball_X_1_blue[0] < -0.3){robot_rotate_CCW(0.3);}
				else if(ball_X_1_blue[0] > 0){robot_rotate_CW(0.1);}
				else if(ball_X_1_blue[0] < 0){robot_rotate_CCW(0.1);}
				return;
			}
			else if (ball_Y_1_blue[0] > 0.5){
				dataInit();
				robot_move_forward(0.6*(ball_Y_1_blue[0] - 1) + 1);
				if(ball_X_1_blue[0] > 0){robot_rotate_CW(0.21);}
				else if(ball_X_1_blue[0] < 0){robot_rotate_CCW(0.21);}
				return;
			}
			else if (ball_Y_1_blue[0] > 0){
				dataInit();
				robot_move_forward(0.7);
				if(ball_X_1_blue[0] > 0){robot_rotate_CW(0.21);}
				else if(ball_X_1_blue[0] < 0){robot_rotate_CCW(0.21);}
				return;
			}
			return;
		}
		else if (ball_X_1_blue[0] > 0){
			if (ball_Y_1_blue[0] > 1){
				dataInit();
				robot_move_forward(1);
				robot_rotate_CW(0.74);
				return;
			}
			else if (ball_Y_1_blue[0] > 0.5){
				dataInit();
				robot_move_forward(0.6*(ball_Y_1_blue[0] - 1) + 1);
				robot_rotate_CW(0.41);
				return;
			}
			else if (ball_Y_1_blue[0] > 0){
				dataInit();
				robot_move_forward(0.7);
				robot_rotate_CW(0.34);
				return;
			}
			return;
		}
		else if (ball_X_1_blue[0] < 0){
			if (ball_Y_1_blue[0] > 1){
				dataInit();
				robot_move_forward(1);
				robot_rotate_CCW(0.74);
				return;
			}
			else if (ball_Y_1_blue[0] > 0.5){
				dataInit();
				robot_move_forward(0.6*(ball_Y_1_blue[0] - 1) + 1);
				robot_rotate_CCW(0.41);
				return;
			}
			else if (ball_Y_1_blue[0] > 0){
				dataInit();
				robot_move_forward(0.7);
				robot_rotate_CCW(0.34);
				return;
			}
			return;
		}
		return;
	}
	return;
}



int robot_new_pick_blue_ball()
{
	float threshold = 0.13/2;
	float red_threshold = 0.16;
	int stop_number = 100;
	// collect ball
	if ((ball_number_2_blue == 0 && global_state == 1) || (global_state2 == 2) && ball_distance_2_blue[0] > 0.3){
		dataInit();
		robot_move_forward(0.7);
		// robot_pick_ball(0.8);
		global_count++;
		// end collecting ball
		if (global_count < 10){
			dataInit();
			robot_move_forward(0.7);
			global_count++;
			return 0;
		}
		else if (global_count < 55){
			dataInit();
			robot_move_forward(0.7);
			robot_pick_ball(0.5);
			global_count++;
			return 0;
		}
		else if (global_count <= stop_number){
			dataInit();
			robot_pick_ball(0.5);
			global_count++;
			return 0;
		}
		else if (global_count > stop_number){
			global_count = 0;
			global_state = 0;
			global_state2 = 0;
			return 1;
		}
		return 0;
	}
	// avoid red ball
	if (is_red_ball_in_threshold(red_threshold)){
		std::cout << "Avoid red ball" << std::endl;
		robot_avoid_red_ball(red_threshold);
		return 0;
	}
	// move to blue ball
	std::cout << "Get blue ball" << std::endl;
	robot_move_new_threshold(threshold);
	return 0;
}



int robot_rotate_green_center()
{
	if (ball_number_2_green == 2){
		if (ball_X_2_green[0] * ball_X_2_green[1] < 0){return 1;}
		else{
			if (ball_X_2_green[0] > 0){
				dataInit();
				robot_rotate_CW(0.1);
				return 0;
			}
			else if (ball_X_2_green[0] < 0){
				dataInit();
				robot_rotate_CCW(0.1);
				return 0;
			}
			return 0;
		}
	}
	else if (ball_number_1_green == 2){
		if (ball_X_1_green[0] * ball_X_1_green[1] < 0){return 1;}
		else{
			if (ball_X_1_green[0] > 0){
				dataInit();
				robot_rotate_CW(0.1);
				return 0;
			}
			else if (ball_X_1_green[0] < 0){
				dataInit();
				robot_rotate_CCW(0.1);
				return 0;
			}
			return 0;
		}
	}
	else{
		dataInit();
		robot_rotate_CW(0.4);
		return 0;
	}
}
int robot_go_to_green_ball()
{
	float min_Y = 0.3;
	if (ball_number_2_green == 2){
		std::cout << "Enter if (ball_number_2_green == 2)" << std::endl;
		std::cout << "ball_distance_2_green[0], min_Y = " << ball_distance_2_green[0] << ", " << min_Y << std::endl;
		if (ball_distance_2_green[0] < min_Y){
			std::cout << "Enter if (ball_distance_2_green[0] < min_Y)" << std::endl;
			std::cout << "green ball distance 2 = " << ball_distance_2_green[0] << std::endl;
			dataInit();
			return 1;
		}
		else if (ball_distance_2_green[0] > min_Y){
			std::cout << "Enter else if (ball_distance_2_green[0] > min_Y)" << std::endl;
			std::cout << "green ball distance 2 = " << ball_distance_2_green[0] << std::endl;
			if (ball_distance_2_green[0] > 2){
				std::cout << "Enter if (ball_distance_2_green[0] > 2)" << std::endl;
				dataInit();
				robot_move_forward(1);
				return 0;
			}
			else if (ball_distance_2_green[0] > 1){
				std::cout << "Enter else if (ball_distance_2_green[0] > 1)" << std::endl;
				dataInit();
				robot_move_forward(0.7 * (ball_distance_2_green[0] - 2) + 1);
				return 0;
			}
			else{
				std::cout << "Enter else 1" << std::endl;
				dataInit();
				robot_move_forward(0.3);
				return 0;
			}
		}
		return 0;
	}
	else if (ball_number_1_green == 2){
		std::cout << "Enter else if (ball_number_1_green == 2)" << std::endl;
		std::cout << "green ball distance 1, min_Y = " << ball_distance_1_green[0] << ", " << min_Y<< std::endl;
		if (ball_distance_1_green[0] < min_Y){
			std::cout << "Enter if (ball_distance_1_green[0] < min_Y)" << std::endl;
			dataInit();
			return 1;
		}
		else if (ball_distance_1_green[0] > min_Y){
			std::cout << "Enter else if (ball_distance_1_green[0] > min_Y)" << std::endl;
			std::cout << "green ball distance 1 = " << ball_distance_1_green[0] << std::endl;
			if (ball_distance_1_green[0] > 2){
				std::cout << "Enter if (ball_distance_1_green[0] > 2)" << std::endl;
				dataInit();
				robot_move_forward(1);
				return 0;
			}
			else if (ball_distance_1_green[0] > 1){
				std::cout << "Enter else if (ball_distance_1_green[0] > 1)" << std::endl;
				dataInit();
				robot_move_forward(0.7 * (ball_distance_2_green[0] - 2) + 1);
				return 0;
			}
			else{
				std::cout << "Enter else 2" << std::endl;
				dataInit();
				robot_move_forward(0.3);
				return 0;
			}
		}
		return 0;
	}
	else if (ball_number_2_green == 1){
		if (ball_X_2_green[0] < 0){
			dataInit();
			robot_move_arc_CCW(ball_X_2_green[0]);
			return 0;
		}
		else if (ball_X_2_green[0] > 0){
			dataInit();
			robot_move_arc_CW(ball_X_2_green[0]);
			return 0;
		}
		return 0;
	}
	else if (ball_number_1_green == 1){
		if (ball_X_1_green[0] < 0){
			dataInit();
			robot_move_arc_CCW(ball_X_1_green[0]);
			return 0;
		}
		else if (ball_X_1_green[0] > 0){
			dataInit();
			robot_move_arc_CW(ball_X_1_green[0]);
			return 0;
		}
		return 0;
	}
	else{
		std::cout << "Error: Camera cannot detect green ball" << std::endl;
		return 0;
	}
}



int robot_find_two_green_ball_CW()
{
	if (ball_number_1_green == 0){
		dataInit();
		robot_rotate_CW(1);
		return 0;
	}
	else if (ball_number_1_green == 1){
		dataInit();
		robot_rotate_CW(0.2);
		return 0;
	}
	else if (ball_number_1_green == 2){return 1;}
	return 0;
}

int robot_find_two_green_ball_CCW()
{
	if (ball_number_1_green == 0){
		dataInit();
		robot_rotate_CCW(1);
		return 0;
	}
	else if (ball_number_1_green == 1){
		dataInit();
		robot_rotate_CCW(0.2);
		return 0;
	}
	else if (ball_number_1_green == 2){return 1;}
	return 0;
}

int robot_go_to_green_for_distance()
{
	float x_threshold = 0.2;
	float x_sum = ball_X_1_green[0] + ball_X_1_green[1];
	float ball_car_distance = 0.75;
	if (ball_Y_1_green[0] > ball_car_distance){
		if(x_sum > x_threshold){
			dataInit();
			robot_rotate_CW(0.04);
			return 0;
		}
		else if (x_sum < -x_threshold){
			dataInit();
			robot_rotate_CCW(0.04);
			return 0;
		}
		else if (std::abs(x_sum) < x_threshold){
			dataInit();
			robot_move_forward(0.8);
			return 0;
		}
		return 0;
	}
	else if (ball_Y_1_green[0] < ball_car_distance){
		dataInit();
		return 1;
	}
	return 0;
}


int robot_head_parallel_to_green()
{
	// Try to go parallel and go center of two green ball
	if (ball_number_1_green > 1){
		float y_diff_threshold = 0.013;
		float x_diff_threshold = 0.1;
		float escape_threshold = 0.2;

		//When the ass is not parallel yet
		if (((std::abs(ball_Y_1_green[1] - ball_Y_1_green[0]) > y_diff_threshold)) && global_state == 0){
			float x_sum = ball_X_1_green[0] + ball_X_1_green[1];
			if (ball_X_1_green[0] > ball_X_1_green[1]){
				if (ball_X_1_green[1] < -escape_threshold){
					dataInit();
					robot_move_left(0.1);
					robot_rotate_CW(0.2);
					return 0;
				}
				else if(0 > ball_X_1_green[1] > -escape_threshold){
					dataInit();
					robot_rotate_CW(0.2);
					return 0;
				}
				return 0;
			}
			else if (ball_X_1_green[1] > ball_X_1_green[0]){
				if (ball_X_1_green[1] > escape_threshold){
					dataInit();
					robot_move_right(0.1);
					robot_rotate_CCW(0.2);
					return 0;
				}
				else if (ball_X_1_green[1] < escape_threshold){
					dataInit();
					robot_rotate_CCW(0.2);
					return 0;
				}
				return 0;
			}
			return 0;
		}
		else if(global_state == 0 && (std::abs(ball_Y_1_green[1] - ball_Y_1_green[0]) < y_diff_threshold)){global_state = 1;}
		//When the ass is parallel: We have to go to midpoint of two green balls
		else if (global_state == 1){
			ROS_INFO("Parallel has been done. Now starting to get to midpoint");
			if (ball_X_1_green[0] + ball_X_1_green[1] > x_diff_threshold){
				dataInit();
				robot_move_right(0.2);
				std::cout << "y difference" << ball_Y_1_green[1] - ball_Y_1_green[0] << std::endl;
				return 0;
			}
			else if(ball_X_1_green[0] + ball_X_1_green[1] < -x_diff_threshold){
				dataInit();
				robot_move_left(0.2);
				std::cout << "y difference" << ball_Y_1_green[1] - ball_Y_1_green[0] << std::endl;
				return 0;
			}
			else if (std::abs(ball_X_1_green[0] + ball_X_1_green[1]) < x_diff_threshold){
				std::cout << "y difference" << ball_Y_1_green[1] - ball_Y_1_green[0] << std::endl;
				dataInit();
				global_state = 0;
				return 1;
			}
			return 0;
		}
		return 0;
	}
	else{
		dataInit();
		ROS_INFO("camera 2 is watching less than two green ball");
		return 0;
	}
}
int robot_parking_and_release()
{
	float diff_x_threshold = 0.3;
	float diff_green_threshold = 0.01;
	float x_sum = ball_X_1_green[0] + ball_X_1_green[1];
	int move_forward_limit = 40;
	if (global_state == 0){
		if (ball_number_2_green != 2){
			if (x_sum > diff_x_threshold){
				dataInit();
				robot_move_right(0.06);
				return 0;
			}
			else if (x_sum < -diff_x_threshold){
				dataInit();
				robot_move_left(0.06);
				return 0;
			}
			else if (std::abs(x_sum) < diff_x_threshold){
				dataInit();
				robot_move_forward(0.25);
				return 0;
			}
			return 0;
		}
		if (global_state == 0 && ball_number_2_green == 2){
			dataInit();
			global_state = 1;
		}
	}
	else if (global_state == 1){
		if (global_count < 120){
			dataInit();
			robot_move_forward(0.3);
			global_count++;
			return 0;
		}
		else if (global_count == 120){
			dataInit();
			global_state = 2;
			global_count = 0;
			return 0;
		}
		return 0;
	}
	else if (global_state == 2){
		std::cout << "ball_number_3_green = "<< ball_number_3_green << std::endl;
		if (ball_number_3_green == 0){
			dataInit();
			robot_rotate_CW(0.4);
			return 0;
		}
		else if (ball_number_3_green == 1){
			dataInit();
			robot_rotate_CW(0.2);
			return 0;
		}
		else if (ball_number_3_green > 1){
			float x_sum_3 = ball_X_3_green[0] + ball_X_3_green[1];
			if (x_sum_3 > diff_green_threshold){
				dataInit();
				robot_rotate_CW(0.05);
				return 0;
			}
			else if (x_sum_3 < -diff_green_threshold){
				dataInit();
				robot_rotate_CCW(0.05);
				return 0;
			}
			else if (std::abs(x_sum_3) < diff_green_threshold){
				dataInit();
				global_state = 3;
				std::cout << " (back camera)docking state: " << global_state << std::endl;
				return 0;
			}
		}
		return 0;
	}
	else if (global_state == 3){
		std::cout << "Y = " << ball_Y_3_green[0] << std::endl;
		if (ball_Y_3_green[0] > 0.42){
			dataInit();
			robot_move_back(0.3);
			return 0;
		}
		else if ((0.303<ball_Y_3_green[0])&&(ball_Y_3_green[0]<=0.45)){
			dataInit();
			robot_move_back(0.12);
			return 0;
		}
		else if (ball_Y_3_green[0] < 0.303){
			dataInit();
			global_state = 4;
			return 0;
		}
		return 0;
	}
	else if (global_state == 4){
		if (global_count < 30){
			dataInit();
			robot_exit_ball();
			global_count++;
			return 0;
		}
		else if (global_count == 30){
			dataInit();
			global_count = 0;
			global_state = 0;
			return 1;
		}
		return 0;
	}
	return 0;
}



int robot_go_to_green_for_distance_faster()
{
	float x_threshold = 0.2;
	float x_sum = ball_X_1_green[0] + ball_X_1_green[1];
	float ball_car_distance = 0.75;
	if (ball_Y_1_green[0] > ball_car_distance){
		if(x_sum > x_threshold){
			dataInit();
			robot_rotate_CW(0.25);
			robot_move_forward(0.6);
			return 0;
		}
		else if (x_sum < -x_threshold){
			dataInit();
			robot_rotate_CCW(0.25);
			robot_move_forward(0.6);
			return 0;
		}
		else if (std::abs(x_sum) < x_threshold){
			dataInit();
			robot_move_forward(1);
			return 0;
		}
		return 0;
	}
	else if (ball_Y_1_green[0] < ball_car_distance){
		dataInit();
		return 1;
	}
	return 0;
}

int robot_head_parallel_to_green_faster()
{
	// Try to go parallel and go center of two green ball
	if (ball_number_1_green > 1){
		float y_diff_threshold = 0.013;
		float x_diff_threshold = 0.08;
		float escape_threshold = 0.2;

		//When the ass is not parallel yet
		if (((std::abs(ball_Y_1_green[1] - ball_Y_1_green[0]) > y_diff_threshold)) && global_state == 0){
			float x_sum = ball_X_1_green[0] + ball_X_1_green[1];
			if (ball_X_1_green[0] > ball_X_1_green[1]){
				if (ball_X_1_green[1] < -escape_threshold){
					dataInit();
					robot_move_left(0.3);
					robot_move_forward(0.2);
					return 0;
				}
				else if(0 > ball_X_1_green[1] && ball_X_1_green[1] > -escape_threshold){
					dataInit();
					robot_rotate_CW(0.3);
					return 0;
				}
				return 0;
			}
			else if (ball_X_1_green[1] > ball_X_1_green[0]){
				if (ball_X_1_green[1] > escape_threshold){
					dataInit();
					robot_move_right(0.3);
					robot_move_forward(0.4);
					return 0;
				}
				else if (ball_X_1_green[1] < escape_threshold){
					dataInit();
					robot_rotate_CCW(0.3);
					return 0;
				}
				return 0;
			}
			return 0;
		}
		else if(global_state == 0 && (std::abs(ball_Y_1_green[1] - ball_Y_1_green[0]) < y_diff_threshold)){global_state = 1;}
		//When the ass is parallel: We have to go to midpoint of two green balls
		else if (global_state == 1){
			ROS_INFO("Parallel has been done. Now starting to get to midpoint");
			if (ball_X_1_green[0] + ball_X_1_green[1] > x_diff_threshold){
				dataInit();
				robot_move_right(0.35);
				std::cout << "y difference" << ball_Y_1_green[1] - ball_Y_1_green[0] << std::endl;
				return 0;
			}
			else if(ball_X_1_green[0] + ball_X_1_green[1] < -x_diff_threshold){
				dataInit();
				robot_move_left(0.35);
				std::cout << "y difference" << ball_Y_1_green[1] - ball_Y_1_green[0] << std::endl;
				return 0;
			}
			else if (std::abs(ball_X_1_green[0] + ball_X_1_green[1]) < x_diff_threshold){
				std::cout << "y difference" << ball_Y_1_green[1] - ball_Y_1_green[0] << std::endl;
				dataInit();
				global_state = 0;
				return 1;
			}
			return 0;
		}
		return 0;
	}
	else{
		dataInit();
		robot_move_forward(0.1);
		if (ball_number_2_green == 2){return 1;}

		ROS_INFO("camera 2 is watching less than two green ball");
		return 0;
	}
}

int robot_parking_and_release_faster()
{
	float diff_x_threshold = 0.3;
	float diff_green_threshold = 0.05;
	float x_sum = ball_X_1_green[0] + ball_X_1_green[1];
	int move_forward_limit = 40;
	if (global_state == 0){
		if (ball_number_2_green != 2){
			if (x_sum > diff_x_threshold){
				dataInit();
				robot_move_right(0.12);
				robot_move_forward(0.3);
				return 0;
			}
			else if (x_sum < -diff_x_threshold){
				dataInit();
				robot_move_left(0.12);
				robot_move_forward(0.3);
				return 0;
			}
			else if (std::abs(x_sum) < diff_x_threshold){
				dataInit();
				robot_move_forward(0.4);
				return 0;
			}
			return 0;
		}
		if (global_state == 0 && ball_number_2_green == 2){
			dataInit();
			global_state = 1;
		}
	}
	else if (global_state == 1){
		if (global_count < 50){
			dataInit();
			robot_move_forward(0.6);
			global_count++;
			return 0;
		}
		else if (global_count == 50){
			dataInit();
			global_state = 2;
			global_count = 0;
			return 0;
		}
		return 0;
	}
	else if (global_state == 2){
		std::cout << "ball_number_3_green = "<< ball_number_3_green << std::endl;
		if (ball_number_3_green == 0){
			dataInit();
			robot_rotate_CW(1);
			return 0;
		}
		else if (ball_number_3_green == 1){
			dataInit();
			robot_rotate_CW(0.7);
			return 0;
		}
		else if (ball_number_3_green > 1){
			float x_sum_3 = ball_X_3_green[0] + ball_X_3_green[1];
			if (x_sum_3 > diff_green_threshold){
				dataInit();
				robot_rotate_CW(0.3);
				return 0;
			}
			else if (x_sum_3 < -diff_green_threshold){
				dataInit();
				robot_rotate_CCW(0.3);
				return 0;
			}
			else if (std::abs(x_sum_3) < diff_green_threshold){
				dataInit();
				global_state = 3;
				std::cout << " (back camera)docking state: " << global_state << std::endl;
				return 0;
			}
		}
		return 0;
	}
	else if (global_state == 3){
		std::cout << "Y = " << ball_Y_3_green[0] << std::endl;

		dataInit();
		robot_move_back(0.6);

		if (ball_number_3_green < 2 || ball_Y_3_green[0] < 0.32){
			dataInit();
			global_state = 4;
			return 0;
		}
		/*
		if (ball_Y_3_green[0] > 0.45){
			dataInit();
			robot_move_back(0.8);
			return 0;
		}
		else if ((0.32<ball_Y_3_green[0])&&(ball_Y_3_green[0]<=0.45)){
			dataInit();
			robot_move_back(0.25);
			return 0;
		}
		else if (ball_Y_3_green[0] < 0.32){
			dataInit();
			global_state = 4;
			return 0;
		}
		*/
		return 0;
	}
	else if (global_state == 4){
		if (global_count < 30){
			dataInit();
			robot_exit_ball();
			global_count++;
			return 0;
		}
		else if (global_count == 30){
			dataInit();
			global_count = 0;
			global_state = 0;
			return 1;
		}
		return 0;
	}
	return 0;
}






int rotate_guide[3] = {0, 0, 0};

void set_rotate_direction()
{
	float dist12;
	float dist13;
	if (ball_number_1_blue == 3){
		dist12 = (ball_X_1_blue[0] - ball_X_1_blue[1]) * (ball_X_1_blue[0] - ball_X_1_blue[1]) + (ball_Y_1_blue[0] - ball_Y_1_blue[1]) * (ball_Y_1_blue[0] - ball_Y_1_blue[1]);
		dist13 = (ball_X_1_blue[0] - ball_X_1_blue[2]) * (ball_X_1_blue[0] - ball_X_1_blue[2]) + (ball_Y_1_blue[0] - ball_Y_1_blue[2]) * (ball_Y_1_blue[0] - ball_Y_1_blue[2]);
		// index 0 is first
		if (ball_X_1_blue[0] < ball_X_1_blue[1] && ball_X_1_blue[0] < ball_X_1_blue[2]){
			rotate_guide[0] = 1;
			if (ball_X_1_blue[1] < ball_X_1_blue[2]){
				if (ball_Y_1_blue[1] > ball_Y_1_blue[2]){
					rotate_guide[1] = 1;
					rotate_guide[2] = 1;
					return;
				}
				else{
					rotate_guide[1] = -1;
					rotate_guide[2] = 1;
					return;
				}
			}
			else{
				if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
					rotate_guide[1] = 1;
					rotate_guide[2] = 1;
					return;
				}
				else{
					rotate_guide[1] = -1;
					rotate_guide[2] = 1;
					return;
				}
			}
		}
		// index 0 is third
		else if (ball_X_1_blue[0] > ball_X_1_blue[1] && ball_X_1_blue[0] > ball_X_1_blue[2]){
			rotate_guide[0] = -1;
			if (ball_X_1_blue[1] < ball_X_1_blue[2]){
				if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
					rotate_guide[1] = -1;
					rotate_guide[2] = -1;
					return;
				}
				else{
					rotate_guide[1] = 1;
					rotate_guide[2] = -1;
					return;
				}
			}
			else{
				if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
					rotate_guide[1] = 1;
					rotate_guide[2] = -1;
					return;
				}
				else{
					rotate_guide[1] = -1;
					rotate_guide[2] = -1;
					return;
				}
			}
		}
		// index 0 is second
		else{
			if (dist12 > dist13){
				if (ball_X_1_blue[1] < ball_X_1_blue[2]){
					rotate_guide[0] = -1;
					if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
						rotate_guide[1] = 1;
						rotate_guide[2] = 1;
						return;
					}
					else{
						rotate_guide[1] = -1;
						rotate_guide[2] = 1;
						return;
					}
				}
				else{
					rotate_guide[0] = 1;
					if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
						rotate_guide[1] = -1;
						rotate_guide[2] = -1;
						return;
					}
					else{
						rotate_guide[1] = 1;
						rotate_guide[2] = -1;
						return;
					}
				}
			}
			else{
				if (ball_X_1_blue[1] < ball_X_1_blue[2]){
					rotate_guide[0] = 1;
					if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
						rotate_guide[1] = -1;
						rotate_guide[2] = -1;
						return;
					}
					else{
						rotate_guide[1] = -1;
						rotate_guide[2] = -1;
						return;
					}
				}
				else{
					rotate_guide[0] = -1;
					if (ball_Y_1_blue[1] < ball_Y_1_blue[2]){
						rotate_guide[1] = 1;
						rotate_guide[2] = 1;
						return;
					}
					else{
						rotate_guide[1] = 1;
						rotate_guide[2] = 1;
						return;
					}
				}
			}
		}
	}
	return;
}




bool check_inside1(float blue_x, float blue_y, float red_x, float red_y)
{
	float y1, y2, y3, y4;
	float m, cos;
	if(blue_x == 0){
		if(red_y < blue_y) return true;
		return false;
	}

	// 57.3 == tan(89), blue_y/blue_x error(?)
	else if(blue_y/blue_x > 57.3 || blue_y/blue_x < -57.3){
		if(red_y < blue_y) return true;
		return false;
	}

	cos = blue_x / sqrt(blue_x * blue_x + blue_y * blue_y);
	if(cos < 0) cos *= -1;
	m = 0.3/cos;

	y1 = (blue_y/blue_x)*red_x + m;
	y2 = (blue_y/blue_x)*red_x - m;
	y3 = -(blue_x/blue_y)*red_x + (blue_x*blue_x + blue_y*blue_y)/blue_y;
	y4 = -(blue_x/blue_y)*red_x;

	if(y1 > red_y && y2 < red_y && y3 > red_y && y4 < red_y) return true;
	return false;
}
bool check_inside2(float blue_x1, float blue_y1, float blue_x2, float blue_y2, float red_x, float red_y)
{

	float y1, y2, y3, y4, cos, m;
	float up, down;
	if(blue_x2 - blue_x1 == 0){
		if(blue_y1 > blue_y2){
			up = blue_y1;
			down = blue_y2;
		}
		else{
			up = blue_y2;
			down = blue_y1;
		}

		if(red_y < up && red_y > down && blue_x1 - red_x < 0.25 && blue_x1 - red_x > -0.25) return true;

		return false;
	}

	if((blue_y2 - blue_y1)/(blue_x2 - blue_x1) > 57.3 && (blue_y2 - blue_y1)/(blue_x2 - blue_x1) < -57.3) {
		if(red_y < up && red_y > down && blue_x1 - red_x < 0.25 && blue_x1 - red_x > -0.25) return true;

		return false;

	}

	else{
		cos = sqrt((blue_x2 - blue_x1)*(blue_x2 - blue_x1) + (blue_y2 - blue_y1)*(blue_y2 - blue_y1))/(blue_x2 - blue_x1);
		if(cos < 0) cos *= -1;
		m = 0.3/cos;

		y1 = ((blue_y2 - blue_y1)/(blue_x2 - blue_x1))*red_x + (blue_x2*blue_y1 - blue_x1*blue_y2)/(blue_x2 - blue_x1) + m;
		y2 = ((blue_y2 - blue_y1)/(blue_x2 - blue_x1))*red_x + (blue_x2*blue_y1 - blue_x1*blue_y2)/(blue_x2 - blue_x1) - m;
		y3 = -((blue_x2 - blue_x1)/(blue_y2 - blue_y1))*red_x + blue_y1 + ((blue_x2 - blue_x1)/(blue_y2 - blue_y1))*blue_x1;
		y4 = -((blue_x2 - blue_x1)/(blue_y2 - blue_y1))*red_x + blue_y2 + ((blue_x2 - blue_x1)/(blue_y2 - blue_y1))*blue_x2;

		if(red_y < y1 && red_y > y2 && red_y > y3 && red_y < y4) return true;
		return false;

	}
}

//function 1 : execute when 3 blue ball detected, output int array[6]
void check_red_ball_between_blue_balls()
{
	//exist_red[0] : check red ball between robot and first blue ball
	//exist_red[1] : check red ball between robot and second blue ball
	//exist_red[2] : check red ball between robot and third blue ball
	//exist_red[3] : check red ball between first blue ball and second blue ball
	//exist_red[4] : check red ball between second blue ball and third blue ball
	//exist_red[5] : check red ball between first blue ball and third blue ball

	for(int i = 0;i < 6;i++) exist_red[i] = 0;

	if (ball_number_1_red == 0) return;

	for(int i=0;i<3;i++){
		for(int j=0;j<ball_number_1_red;j++){
			if(check_inside1(ball_X_1_blue[i], ball_Y_1_blue[i], ball_X_1_red[j], ball_Y_1_red[j])) {
				exist_red[i] = 1;
				break;
			}
		}
	}

	for(int j=0;j<ball_number_1_red;j++){
		if(check_inside2(ball_X_1_blue[0], ball_Y_1_blue[0], ball_X_1_blue[1], ball_Y_1_blue[1], ball_X_1_red[j], ball_Y_1_red[j])) {
			exist_red[3] = 1;
			break;
		}
	}

	for(int j=0;j<ball_number_1_red;j++){
		if(check_inside2(ball_X_1_blue[1], ball_Y_1_blue[1], ball_X_1_blue[2], ball_Y_1_blue[2], ball_X_1_red[j], ball_Y_1_red[j])) {
			exist_red[4] = 1;
			break;
		}
	}


	for(int j=0;j<ball_number_1_red;j++){
		if(check_inside2(ball_X_1_blue[0], ball_Y_1_blue[0], ball_X_1_blue[2], ball_Y_1_blue[2], ball_X_1_red[j], ball_Y_1_red[j])) {
			exist_red[5] = 1;
			break;
		}
	}
	return ;
}
//function 2 : input 6 float, output
void find_distance_between_blue_balls()
{
	float ball_X_3_blue[20];
	float ball_Y_3_blue[20];

	for(int i=0;i<6;i++){
		distance_between_blue_balls[i] = sqrt(ball_X_1_blue[i/2]*ball_X_1_blue[i/2] + ball_Y_1_blue[i/2]*ball_Y_1_blue[i/2]);

	}
	float dist12 = sqrt((ball_X_1_blue[1] - ball_X_1_blue[0])*(ball_X_1_blue[1] - ball_X_1_blue[0]) + (ball_Y_1_blue[1] - ball_Y_1_blue[0])*(ball_Y_1_blue[1] - ball_Y_1_blue[0]));
	float dist23 = sqrt((ball_X_1_blue[1] - ball_X_1_blue[2])*(ball_X_1_blue[1] - ball_X_1_blue[2]) + (ball_Y_1_blue[1] - ball_Y_1_blue[2])*(ball_Y_1_blue[1] - ball_Y_1_blue[2]));
	float dist13 = sqrt((ball_X_1_blue[2] - ball_X_1_blue[0])*(ball_X_1_blue[2] - ball_X_1_blue[0]) + (ball_Y_1_blue[2] - ball_Y_1_blue[0])*(ball_Y_1_blue[2] - ball_Y_1_blue[0]));

	//123
	distance_between_blue_balls[0] += dist12 + dist23;
	//132
	distance_between_blue_balls[1] += dist13 + dist23;
	//213
	distance_between_blue_balls[2] += dist12 + dist13;
	//231
	distance_between_blue_balls[3] += dist23 + dist13;
	//312
	distance_between_blue_balls[4] += dist13 + dist12;
	//321
	distance_between_blue_balls[5] += dist23 + dist12;

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_integation");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position_1", 1000, camera1_Callback);
	ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position>("/position_2", 1000, camera2_Callback);
	ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/position_3", 1000, camera3_Callback);
	dataInit();

  c_socket = socket(PF_INET, SOCK_STREAM, 0);
	c_addr.sin_addr.s_addr = inet_addr(IPADDR);
  c_addr.sin_family = AF_INET;
  c_addr.sin_port = htons(PORT);

		///////////////////////////////////////////////////////////////////////
		//	렙뷰와 통신이 되었는지 확인하는 코드 아래 코드를 활성화 후 노드를 실행 시켰을때///
		//	노드가 작동 -> 통신이 연결됨, Failed to connect 이라고 뜸 -> 통신이 안됨///
		////////////////////////////////////////////////////////////////////////
  if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
    printf("Failed to connect\n");
    close(c_socket);
    return -1;
	}
	namedWindow("press q to quit");
	while(ros::ok)
	{
		if ((char)waitKey(1) == 'q'){
			dataInit();
			return 0;
		}




/*
		// Simple Demo
    if (robot_state == 0)
    {
    	std::cout << "robot_state " << robot_state << std::endl;
      if (robot_rotate_green_center()){robot_state = 1;}
      data_print();
    }
		else if (robot_state == 1)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			if (robot_go_to_green_ball()){robot_state = 2;}
			data_print();
		}
    else
    {
      dataInit();
      data_print();
      std::cout << "simple demo clear" << std::endl;
			return 0;
    }
*/



		// final main function
		set_rotate_direction();

		if (robot_state == 0)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (robot_find_blue_ball_CW()){robot_state = 1;}
			data_print();
		}
		else if (robot_state == 1)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (robot_new_pick_blue_ball()){robot_state = 2;}
			data_print();
		}
		else if (robot_state == 2)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (rotate_guide[0] == 1){
				if (robot_find_blue_ball_CW()){robot_state = 3;}
				data_print();
			}
			else{
				if (robot_find_blue_ball_CCW()){robot_state = 3;}
				data_print();
			}
		}
		else if (robot_state == 3)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (robot_new_pick_blue_ball()){robot_state = 4;}
			data_print();
		}
		else if (robot_state == 4)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (rotate_guide[1] == 1){
				if (robot_find_blue_ball_CW()){robot_state = 5;}
				data_print();
			}
			else{
				if (robot_find_blue_ball_CCW()){robot_state = 5;}
				data_print();
			}
		}
		else if (robot_state == 5)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (robot_new_pick_blue_ball()){robot_state = 6;}
			data_print();
		}
		else if (robot_state == 6)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			std::cout << "rotate_guide = [" << rotate_guide[0] << rotate_guide[1] << rotate_guide[2] << "]" << std::endl;
			if (rotate_guide[2] == 1){
				if (robot_find_two_green_ball_CW()){robot_state = 7;}
				data_print();
			}
			else{
				if (robot_find_two_green_ball_CCW()){robot_state = 7;}
				data_print();
			}
		}
		else if (robot_state == 7)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			if (robot_go_to_green_for_distance_faster()){robot_state = 8;}
			data_print();
		}
		else if (robot_state == 8)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			if (robot_head_parallel_to_green_faster()){robot_state = 9;}
			data_print();
		}
		else if (robot_state == 9)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			if (robot_parking_and_release_faster()){robot_state = 10;}
			data_print();
		}
		else if (robot_state == 10)
		{
			std::cout << "robot_state " << robot_state << std::endl;
			if (robot_wait_second(1)){robot_state = 11;}
			data_print();
		}
		else
		{
			dataInit();
			data_print();
			std::cout << "final demo clear" << std::endl;
			return 0;
		}



		printf("\n");

    write(c_socket, data, sizeof(data));
    ros::Duration(0.025).sleep();
    ros::spinOnce();
	}
	return 0;
}
