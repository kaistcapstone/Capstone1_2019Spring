// Header file
//*******************************************************************/

#include <iostream>     // c++ header for Input/Output stream
#include <stdio.h>      // header for Input/Output library
#include <algorithm>    // header for a collection of fuctions especially designed to be used on ranges of elements
#include <fstream>      // header for Input/Output stream to operate on files
#include <chrono>       // header for dealing with time 
#include <string>       // header for using std::string
#include <signal.h>     // header for handling signal
#include <math.h>       // header for using functions to compute common mathematical operations and transformations
#include <cmath>        // c++ header like math.h
#include <stdlib.h>     // header for several general purpose functions
#include <string.h>     // header for several functions to manipulate C strings and arrays
#include <unistd.h>     // header for providing access to the POSIX operating system API
#include <arpa/inet.h>  // header for internet operations
#include <sys/types.h>  // C POSIX library header for using various data types
#include <sys/socket.h> // header for Internet protocol family
#include <boost/thread.hpp> // header for using boost library
#include <ros/ros.h>        // header for using ros functions
#include <ros/package.h>    // header for using ros functions
#include "core_msgs/ball_position.h" // header for using in camera callback function
#include "ros/ros.h"        // header for using ros functions
#include "sensor_msgs/LaserScan.h"   // header for using in lidar callback function
#include "std_msgs/Int8.h"           // header for using ROS message types of int8 data
#include "std_msgs/String.h"         // header for using ROS messgae types of string data
#include "opencv2/opencv.hpp"        // header for using opencv functions
using namespace std;

//******************************************************************/

// Global variables
//******************************************************************/

#define RAD2DEG(x) ((x)*180./M_PI)  // define function that angle changes radian to degree
#define PORT 3000    // varaible for Network byte that used in callback function
#define IPADDR "172.16.0.1" // myRIO ipaddress
boost::mutex map_mutex;  // variable for using in callback function

int lidar_size;            // size of lidar data array
float lidar_degree[400];   // angle datas of lidar (unit:degree)
float lidar_distance[400]; // distance datas from lidar to walls (unit:mm)
float lidar_zero_degree;   // distnace data that robot angle is zero (unit:mm)
int zero_degree;           // index of lidar_degree that is zero in absolute coordinate system
int least_distance;        // index of lidar_distance that

int blue_number;           // number of blue ball
int red_number;            // number of red ball
int green_number;          // number of green ball
int blue_X[20];            // x coordinate of blue ball (unit:mm)
int blue_Y[20];            // y coordinate of blue ball (unit:mm)
int blue_distance[20];     // distance of blue ball from robot (unit:mm)
int red_X[20];             // x coordinate of red ball (unit:mm)
int red_Y[20];             // y coordinate of red ball (unit:mm)
int red_distance[20];      // distance of red ball from robot (unit:mm)
int green_X[20];           // x coordinate of green ball (unit:mm)
int green_Y[20];           // y coordinate of green ball (unit:mm)
int green_distance[20];    // distance of green ball from robot (unit:mm)

int near_red;              // index of red_Y that is the nearest red ball
int near_blue;             // index of blue_Y that is the nearest blue ball 
int near_green;            // index of green_Y that is the nearest green ball
int near_center_blue;      // index of blue_X that is blue ball located at center of balls 
int rightmost_blue;        // index of blue_X that is rightmost blue ball among blue balls
int rightmost_green;       // index of green_X that is rightmost green ball among green balls

int c_socket, s_socket;    // socket for TCP/IP
struct sockaddr_in c_addr; // structure to connect internet
float data[24];            // signal from xbox

int check = 0;             // variable for checker function

int corners[4];            // index of lidar_degree that is angle of corner of walls 

//********************************************************************/

// 1st function
//*******************************************************************/

void dataInit(){
  data[0] = 0; //lx*data[3];
  data[1] = 0; //ly*data[3];
  data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
  data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
  data[4] = 0; //rx*data[7];
  data[5] = 0; //ry*data[7];
  data[6] = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
  data[7] = 0; //GamepadStickLength(_dev, STICK_RIGHT);
  data[8] = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT); lb
  data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT); rb
  data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP); //horizontal align(pickup)
  data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
  data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
  data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
  data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
  data[15] = 0; //GamepadButtonDown(_dev, BUTTON_B); //우회전
  data[16] = 0; //GamepadButtonDown(_dev, BUTTON_X); //좌회전
  data[17] = 0; //GamepadButtonDown(_dev, BUTTON_Y); //직진
  data[18] = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
  data[19] = 0; //GamepadButtonDown(_dev, BUTTON_START);  //느린 회전 도움
  data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER); //pick up
  data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
  data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
  data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
}

void turn_right(){   
  dataInit();         
  data[15]=1; //오른쪽으로 이동
  write(c_socket, data, sizeof(data));
  ros::Duration(0.025).sleep();
  ros::spinOnce();
}

void turn_right_slow(){
  dataInit();
  data[15] = 1; //오른쪽으로 이동
  data[19] = 1; //느린 이동
  write(c_socket,data,sizeof(data));
  ros::Duration(0.025).sleep();
  ros::spinOnce();
}

void turn_left(){
  dataInit();
  data[16]=1; //왼쪽으로 이동
  write(c_socket, data, sizeof(data));
  ros::Duration(0.025).sleep();
  ros::spinOnce();
}

void turn_left_slow(){
  dataInit();
  data[16] = 1; //왼쪽으로 이동
  data[19] = 1; //느린 이동
  write(c_socket,data,sizeof(data));
  ros::Duration(0.025).sleep();
  ros::spinOnce();
}

void go_straight(){
  dataInit();
  data[17]=1; //직진
  write(c_socket, data, sizeof(data));
  ros::Duration(0.025).sleep();
  ros::spinOnce();
}

void move_backward(){ //Used for no blue ball found
  dataInit();
  data[14]=1; //후진
  write(c_socket, data, sizeof(data));
  ros::Duration(0.025).sleep();
  ros::spinOnce();
}

void move_backward(float distance){
  for(int i = 0;i<12;i++){  //12/40초 동안
    dataInit();
    data[14] = 1;
    write(c_socket,data,sizeof(data));
    ros::Duration(0.025).sleep();
  }
  dataInit();
  write(c_socket,data,sizeof(data));
  ros::Duration(0.3).sleep();
}

//We do not use
void turn_lidar(float angle, int direction){//direction = 1 좌회전, direction = 0 우회전 1
  ros::spinOnce();
  if(angle>90){
    turn_lidar(angle/2,direction);
    turn_lidar(angle/2,direction);
  }
  float initial_angle = lidar_degree[least_distance];
  float moved_angle = lidar_degree[least_distance] - initial_angle;
  while(abs(moved_angle)<angle){
    ros::spinOnce();
    dataInit();
    data[15+direction] = 1;
    write(c_socket,data,sizeof(data));

    moved_angle = lidar_degree[least_distance] - initial_angle;
    if(abs(moved_angle)>180){
      if(moved_angle<0){
        moved_angle = moved_angle + 180;
      }
      if(moved_angle>0){
        moved_angle = moved_angle - 180;
      }
    }
    ROS_INFO("Turned : %f",abs(moved_angle));
    ros::Duration(0.025).sleep();
  }
}

void go_straight_lidar(float distance){
  ros::spinOnce();
  cout<<"lidar_moving : "<<distance<<endl;
  float initial_distance = lidar_zero_degree;
  float moved_distance = initial_distance-lidar_zero_degree;
  int t = 0;
  while(moved_distance < distance){
    t++;
    moved_distance = initial_distance-lidar_zero_degree;
    cout<<"lidar moved : "<<moved_distance<<endl;
    dataInit();
    data[17] = 1;
    write(c_socket, data, sizeof(data));
    ros::Duration(0.025).sleep();
    ros::spinOnce();
  }
}

//We do not use
void go_pickup_lidar(float distance){
  ros::spinOnce();
  cout<<"lidar_moving : "<<distance<<endl;
  float initial_distance = lidar_zero_degree;
  float moved_distance = initial_distance-lidar_zero_degree;
  int t = 0;
  while(moved_distance < distance){
    t++;
    ros::spinOnce();
    moved_distance = initial_distance-lidar_zero_degree;
    cout<<"lidar moved : "<<moved_distance<<endl;
    dataInit();
    data[17] = 1;
    data[10] = 1;
    write(c_socket, data, sizeof(data));
    ros::Duration(0.025).sleep();
  }
}

/*****************************************************2차적인 함수들 (Detour Red)*******************************************************/

void detour_red() {
	ros::Duration(0.1).sleep();
	ros::spinOnce();
	int near_red_X = red_X[near_red]; // The nearest red ball's x-coordinate wrt. the robot
	int near_blue_X = blue_X[near_blue]; // The nearest blue ball's x-coordinate wrt. the robot
	int t = 0; // Measure how much the robot have turned to detour a ball


	if(abs(near_red_X)>185) { // Check again whether the robot have to detour or not
	  cout<<"You don't need to detour"<<endl;
	  return;
	}


	if (near_red_X > 0) { // If the nearest red ball is on the right side of the robot...
		int red_X_prev = near_red_X; // Initialize red_X_prev (the detoured ball's x-pos) as near_red_X
		while (red_number != 0 && red_X_prev < 185) { // While any red ball is visible and red_X_prev is in the range able to collide...
			int now = -1; // The variable should indicate the 'detoured ball'. i.e. red_X[now] denotes the x-pos of the ball which is detoured
			int smallest_delta = 100;
			for (int j = 0; j < red_number; j++) { // For all elements in red_X[] ...
				int delta_X = (red_X[j] - red_X_prev); // Calculate the distance wrt. red_X_prev (the detoured ball's x-pos in the previous frame)
				if (abs(delta_X) < smallest_delta) { // Choose the most similar value wrt. red_X_prev
					now = j; // And save its index so that red_X[now] means the x-pos of the ball which is detoured
					smallest_delta = abs(delta_X);
				}
			}
			if (now == -1) { // If the tracked ball (= detoured ball) is no more visible or etc ...
				ros::spinOnce();
				ros::Duration(0.1).sleep();
				cout << "now is -1" << endl;
				break; // Break the while loop
			}

			red_X_prev = red_X[now]; // Update the the tracked ball (= detoured ball)'s x-pos
			cout << now << endl;
			cout << "red_X : " << red_X_prev << endl;
			turn_left(); // Turn left
			cout << "After turning left, red number is " << red_number << endl;
			t++; // check the step how much the robot have turned so far
		}
	}
	else if (near_red_X <= 0) { // If the nearest red ball is on the left side of the robot...
		int red_X_prev = near_red_X; // Initialize red_X_prev (the detoured ball's x-pos) as near_red_X
		while (red_number != 0 && red_X_prev > -185) { // While any red ball is visible and red_X_prev is in the range able to collide...
			int now = -1; // The variable should indicate the 'detoured ball'. i.e. red_X[now] denotes the x-pos of the ball which is detoured
			int smallest_delta = 100;
			for (int j = 0; j < red_number; j++) { // For all elements in red_X[] ...
				int delta_X = (red_X_prev - red_X[j]); // Calculate the distance wrt. red_X_prev (the detoured ball's x-pos in the previous frame)
				if (abs(delta_X) < smallest_delta) { // Choose the most similar value wrt. red_X_prev
					now = j; // And save its index so that red_X[now] means the x-pos of the ball which is detoured
					smallest_delta = abs(delta_X);
				}
			}
			if (now == -1) { // Update the the tracked ball (= detoured ball)'s x-pos
				ros::spinOnce();
				ros::Duration(0.1).sleep();
				cout << "now is -1" << endl;
				break;
			}
			red_X_prev = red_X[now];
			cout << now << endl;
			cout << "red_X : " << red_X_prev << endl;
			turn_right(); // Turn right
			cout << "After turning right, red number is " << red_number << endl;
			t++; // check the step how much the robot have turned so far
		}
	}

	//Go straight a bit(time_based control)
	for (int i = 0; i < 38; i++) { //For 38/40s
		go_straight(); // Go straight
	}
	cout << "Avoidance finished!" << endl;

	// Turn the opposite direction for 1.6t (time_based control)
	if (near_red_X < 0) // If the nearest red ball was on the left side of the robot, the robot must have detoured right, so...
		for (int i = 0; i < 1.6 * t; i++)
			turn_left(); // Turn left so that the robot should aim where the robot pointed originally

	else if (near_red_X > 0) // Else if the nearest red ball was on the right, the robot must have detoured left, so...
		for (int i = 0; i < 1.6 * t; i++)
			turn_right(); // Turn right so that the robot should aim where the robot pointed originally

	cout << "re-align stage 1 finished" << endl;

	int wanted = 0;
	ros::spinOnce();
	if (check == 1) // If the checker method is on...
		wanted = near_blue; // Target the nearest blue ball
	else // Else... (when the checker method is not activated)
		wanted = rightmost_blue; // Target the rightmost blue ball

	 // Re-align with the blue ball which the robot is aiming now
	ros::spinOnce();
	if (blue_number == 0) { // If no blue ball have been found when trying to re-align...
		cout << "No blue ball found!" << endl;
		while (blue_number == 0) {
			move_backward(); // Move backward until any blue ball is visible
		}
	}
	// Precisely align with the targeted blue ball (within 30mm from the robot's y-axis)
	if (blue_X[wanted] > 30) { // If the aimed blue ball is on the right side of the robot... (farther than 30mm in x-pos)
		while (blue_X[wanted] > 30) {
			wanted = 0;
			if (check == 1) {
				wanted = near_blue;
			}
			else {
				wanted = rightmost_blue;
			}
			cout << "align2 - blue_X : " << blue_X[wanted] << endl;
			turn_right_slow(); // Turn right slowly so that the aimed blue ball should be kept tracked
		}
	}
	else if (blue_X[wanted] < -30) { // If the aimed blue ball is on the left side of the robot... (farther than 30mm in x-pos)
		while (blue_X[wanted] < -30) {
			wanted = 0;
			if (check == 1) {
				wanted = near_blue;
			}
			else {
				wanted = rightmost_blue;
			}
			cout << "align2 - blue_X : " << blue_X[wanted] << endl;
			turn_left_slow(); // Turn left slowly so that the aimed blue ball should be kept tracked
		}
	}
	cout << "re-align stage 2 finished" << endl;
}

/*****************************************************2차적인 함수들 (Pick up + Drop)*******************************************************/


void pick_up() {
	ROS_INFO("Start picking up \n");
	ros::Duration(0.1).sleep();
	ros::spinOnce();
	cout << "Ball x coordinate(Before align) : " << blue_X[near_blue] << "mm" << endl;
	if (blue_X[near_blue] < -18) { // If the nearest blue ball (the ball should be picked up) is on the left from the pickupable range...
		while (blue_X[near_blue] < -18) {
			turn_left_slow(); // Turn left slowly so that the robot and the ball should be aligned
		}
	}
	else if (blue_X[near_blue] > 20) { // If the nearest blue ball (the ball should be picked up) is on the right from the pickupable range...
		while (blue_X[near_blue] > 20) {
			turn_right_slow(); // Turn right slowly so that the robot and the ball should be aligned
		}
	}
	else {
		cout << "No problem." << endl;
	}

	cout << "aligning in pick-up stage is finished." << endl;
	cout << "Ball x coordinate(After align) : " << blue_X[near_blue] << "mm" << endl;

	// Put upward the wing for 32/40s
	for (int i = 0; i < 32; i++) {
		dataInit();
		data[10] = 1; // Command to raise up the wing
		write(c_socket, data, sizeof(data)); // Send the command
		ros::Duration(0.025).sleep();
	}

	//Go straight for 30/40s
	for (int i = 0; i < 30; i++) {
		dataInit();
		data[17] = 1; // Command to move forward
		data[10] = 1; // Command to raise up the wing
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
	cout << "Going straight by time_based control is finished. Now pick up!" << endl;
	dataInit();
	data[20] = 1; // Command to pick up
	write(c_socket, data, sizeof(data));
	ros::Duration(1).sleep(); // Wait for wing to go back to its original position.
}


void drop_balls() {
	ros::spinOnce();
	for (int i = 0; i < 26; i++)
		go_straight(); // Go straight for 26 steps
}
for (int i = 0; i < t * 40; i++) { // Raise up the storage for 40/40s.
	dataInit();
	data[21] = 1; // Command to drop balls
	write(c_socket, data, sizeof(data));
	ros::Duration(0.025).sleep();
}
}

/*****************************************************3차적인 함수들 (Toward blue)*******************************************************/


int toward_blueballs() {
	ros::spinOnce();
	int now_distance = 0;//initialize "now_distance" variable
	cout << "rightmost blue ball's distance : " << blue_distance[rightmost_blue] << endl;
	cout << "rightmost blue ball's x,y coordinate : [" << blue_X[rightmost_blue] << " , " << blue_Y[rightmost_blue] << "]" << endl;
	cout << "we have " << blue_number << " blue balls in sight." << endl;
	int wanted = 0;//initialize "wanted" variable.

	if (check == 1) //update "wanted" variable
		wanted = near_blue; // if an exception happens,change the priority to near ball. exception happens when another near blue ball is in route
	else
		wanted = rightmost_blue; // if no exception, target the rightmost ball

	while (blue_Y[wanted] > 500) {//if target blue ball is farther than 50cm
		dataInit();

		if (blue_number == 0) { //When no blue ball is found. (at the moment)
			cout << "No blue ball found!" << endl;
			while (blue_number != 0) {
				move_backward();
			}
			ros::Duration(0.2).sleep(); // move backward for 0.2 second until blue ball is detected
			ros::spinOnce();
			return 0;
		}

		if (check == 1) //update "wanted" variable
			wanted = near_blue;// if an exception happens,change the priority to near ball. exception happens when another near blue ball is in route
		else
			wanted = rightmost_blue;// if no exception, target the rightmost ball

		if (red_number != 0 && red_distance[near_red] <= blue_distance[wanted] &&
			red_Y[near_red] < 520 && abs(red_X[near_red]) < 185 && red_Y[near_red] > 0) { // if red ball is detected and blocks the path (threshold is 18.5cm in x-axis and 52cm in y-axis)
			cout << "Avoid red ball!" << endl;
			cout << "red ball's distance is" << red_distance[near_red] << endl;
			cout << "red ball X , Y coordinate is :" << red_X[near_red] << "and " << red_Y[near_red] << endl;
			detour_red();// during going to the target blue ball, detour detected red ball
		}
		else {
			go_straight();// no red ball blocks the path, go straight
		}
		if (now_distance != blue_distance[wanted]) {
			now_distance = blue_distance[wanted]; // as vehicle moves, continuosly define "now_distacne" variable to disctance of target blue ball
			cout << "Targeting blue ball Y coordinate is : " << blue_Y[wanted] << "mm" << endl;
		}
		ros::spinOnce();
	}

	if (blue_Y[wanted] < 0) {//if blue ball distance is minus, which is strange case
		cout << "Y coordinate is very strange" << endl;
		while (blue_Y[wanted] < 0) {
			move_backward();
		}
		ros::Duration(0.2).sleep(); // go backward to get new information
		ros::spinOnce();
		return 0;//Y coordinate strange! Retry required
	}
	ros::Duration(0.2).sleep();
	ros::spinOnce();
	cout << "Targeting blue ball's X, Y coordinate is : [" << blue_X[wanted] << " , " << blue_Y[wanted] << "]" << endl;
	cout << "Targeting blue ball's distance is :" << blue_distance[wanted] << endl;

	int pickup_distance = static_cast<double>(blue_Y[near_blue] - 30) / 1000; // if target blue ball is near to pick up
	pick_up(pickup_distance); // try to pick up
	return 1;//successfully got a blue ball.
}


int checker() {//See if near ball is close enough and return 1 if it's close enough
	ros::spinOnce();
	int check1 = 0;
	cout << "rightmost = " << rightmost_blue << ", near blue = " << near_blue << ", near blue_x = " << blue_X[near_blue] << endl;
	if (near_blue != rightmost_blue && blue_X[near_blue] + 500 > 0) {//if near blue ball is in 500mm distance is x coordinate, check triggered!
		check1 = 1;
		cout << "change priority" << endl;
	}
	else {//Or checker is not triggered and keep going forward.
		cout << "Keep forward" << endl;
	}
	return check1;//return 0 or 1
}

/*****************************************************3차적인 함수들 (Return to Base)*******************************************************/

//This function find four corners and store degree and distance value from -180 degree to 180 degree
void find_corners() {
	ros::spinOnce();
	int corner_num = 0;

	for (int i = 0; i < lidar_size; i++) {
		if (isinf(lidar_distance[i])) continue;//if corresponding distance value is null value, lidar returns infinite value, so we need to neglect such value.
		float differenceSum = 0;//To check the corresponding angle is corner angle, we need to sum up differences
		for (int j = i - 25; j < i + 25; j++) {//check for +-25 degree to seek corners
			if (isinf(lidar_distance[j])) {//if corresponding distance value is null value, lidar returns infinite value, so we need to neglect such value.
				continue;
			}
			int k = j;
			if (j < 0) k = lidar_size + j;//if the value is smaller than 0, its real value is 180 - value.
			if (j > lidar_size) k = j - lidar_size;//vise versa
			differenceSum = differenceSum + 1 - lidar_distance[k] / lidar_distance[i];//Sum up all differences
		}
		if (differenceSum > 1.0) {//if the difference sum is larger than certain value, it means there's corner-like region.
			corners[corner_num] = i;
			//cout<<"corner"<<corner_num+1<<" angle is "<<lidar_degree[corners[corner_num]]<<endl;
			//cout<<"corner"<<corner_num+1<<" distance is "<<lidar_distance[corners[corner_num]]<<endl;
			corner_num++;
			i = i + 20;//if corner is detected, it means there's no corner in +-20 degree.
		}
		if (corner_num == 4) {//there's only four corners, so we need to stop
		  //cout<<"Now we have four corners"<<endl;
			break;
		}
	}
}

float find_goal() {//return goal's angle in degree.
	find_corners();//before finding goal, update corner's position
	int farthest = 0;

	//initialize values.
	int partner1;//rightside partner
	float angle_partner1;
	float partner1_distance;
	int partner2;//leftside partner
	float angle_partner2;
	float partner2_distance;

	for (int i = 1; i < 4; i++) {//first machine need to find farthest corner to use the algorithm.
		if (lidar_distance[corners[i]] > lidar_distance[corners[farthest]]) farthest = i;
	}
	float farthest_distance = lidar_distance[corners[farthest]];

	//generally partner corners have adjecant values.
	partner1 = farthest + 1;
	partner2 = farthest - 1;

	//exceptional cases. if farthest corner is located at first corner of last corner.
	if (farthest == corners[3]) partner1 = 0;
	else if (farthest == corners[0]) partner2 = 3;

	//Need to get angle between farthest and partner and partners' distance.
	angle_partner1 = (lidar_degree[corners[partner1]] - lidar_degree[corners[farthest]]) * M_PI / 180;
	partner1_distance = lidar_distance[corners[partner1]];
	angle_partner2 = (lidar_degree[corners[farthest]] - lidar_degree[corners[partner2]]) * M_PI / 180;
	partner2_distance = lidar_distance[corners[partner2]];

	//partner angle exceptional cases. if partner and farthest is located at boundaries.
	if (abs(lidar_degree[corners[partner1]]) + abs(lidar_degree[corners[farthest]]) > 180 && lidar_degree[corners[partner1]] * lidar_degree[corners[farthest]] < 0) {
		angle_partner1 = 360 - (abs(lidar_degree[corners[partner1]]) + abs(lidar_degree[corners[farthest]])) * M_PI / 180;
	}
	if (abs(lidar_degree[corners[partner2]]) + abs(lidar_degree[corners[farthest]]) > 180 && lidar_degree[corners[partner2]] * lidar_degree[corners[farthest]] < 0) {
		angle_partner2 = 360 - (abs(lidar_degree[corners[partner2]]) + abs(lidar_degree[corners[farthest]])) * M_PI / 180;
	}

	//calculate square of sides and substract each other to compare which one is smaller.
	float compare = pow(partner1_distance, 2.0) - pow(partner2_distance, 2.0) - 2 * farthest_distance * (partner1_distance * cos(angle_partner1) - partner2_distance * cos(angle_partner2));
	int real_partner;
	if (compare < 0) {//if compare value is less than zero, the leftside partner is real partner
		real_partner = partner1;
	}
	else {
		real_partner = partner2;
	}
	float ans = (lidar_degree[corners[farthest]] + lidar_degree[corners[real_partner]]) / 2;//value to return.

	//if answer value is extreme cases, we need to calibrate.
	if (abs(lidar_degree[corners[farthest]]) + abs(lidar_degree[corners[real_partner]]) > 180 && lidar_degree[corners[farthest]] * lidar_degree[corners[real_partner]] < 0) {
		if (ans < 0) return 180 + ans;
		else return -180 + ans;
	}
	else {
		return ans;
	}
}

//This function let robot turn toward the goal post and, search for green balls.
void to_find_green() {
	while (find_goal() > 10 || find_goal() < -10) {//Until the angle between front and goal is less than 10 degree, turns.
		if (find_goal() > 0) {
			turn_left();
		}
		else {
			turn_right();
		}
	}
	ros::spinOnce();

	int t = 0;
	//Go straight to see two green ball.
	while (green_number != 2) {
		go_straight();
		t++;
		if (t > 100) {//if camera can't find green balls for 2.5 sec, it means the robot miscalculated the goal position, so start it again.
			to_find_green();
			return;
		}
	}
	for (int i = 0; i < 40; i++) {//go straight for 1 sec (To assure seeing 2 green balls)
		go_straight();
	}

	dataInit();
	write(c_socket, data, sizeof(data));
	ros::Duration(0.5).sleep();//for drop out dragged red balls by pickup module.
	cout << "Webcam sees " << green_number << " green balls now" << endl;//To ensure in testing scenario.
}

//Function for final drop out.
void toward_goal() {
	//align to rightmost green ball since our drop module is on leftside. It is ok to align toward right ball.
	//At first, we don't need to align too precise.
	if (green_X[rightmost_green] > 70) {
		ros::spinOnce();
		while (green_X[rightmost_green] > 70) {
			turn_right_slow();
		}
	}
	else if (green_X[rightmost_green] < -100) {
		ros::spinOnce();
		while (green_X[rightmost_green] < -100) {
			turn_left_slow();
		}
	}

	//Go toward rightmost green until distance is 540mm
	ros::spinOnce();
	while (green_Y[rightmost_green] > 540) {
		dataInit();
		ros::spinOnce();
		if (green_number == 0) { //When no green ball is found. (at the moment)
			ros::Duration(0.025).sleep();
			ros::spinOnce();
		}
		go_straight();
	}

	//align strictly and go straight robot need to see slightly left of green ball.
	if (green_X[rightmost_green] < 30) {
		while (green_X[rightmost_green] < 30) {
			turn_left_slow();
		}
	}
	else if (green_X[rightmost_green] > 60) {
		while (green_X[rightmost_green] > 60) {
			turn_right_slow();
		}
	}
	else {
		cout << "No problem." << endl;
	}
	ros::spinOnce();

	//Go straight for 10/40 sec and drop off for 3 sec to ensure all the blue balls dropped out.
	drop_balls(3);
}


void return_to_base() {
	to_find_green();
	toward_goal();
}

/*****************************************************4차적인 함수들 (start path finding)*******************************************************/



void start_path_finding() { //Final function, integrate everything and select order of blue balls.
	go_straight_lidar(1);//Need to go 1m before starting. To ensure every blue balls are detected.
	int i = 0;
	while (i < 3) {//Collect three balls.
		ros::spinOnce();

		//When we can't find blue balls. turn CCW to find blue ball.
		while (blue_number == 0) {
			turn_left();
		}

		//align to rightmost blue ball within 10cm boundaries.
		if (blue_X[rightmost_blue] > 100) {
			ros::spinOnce();
			cout << blue_X[rightmost_blue] << endl;
			while (blue_X[rightmost_blue] > 100)
				turn_right();
		}
		else if (blue_X[rightmost_blue] < -100) {
			ros::spinOnce();
			cout << blue_X[rightmost_blue] << endl;
			while (blue_X[rightmost_blue] < -100) {
				turn_left();
			}
		}

		//Consider checker-trigger situation && toward_blueball starts!.
		check = checker();


		if (check == 1) {//if checker triggered
			int t = 0;
			while (blue_X[near_blue] < -60) {//align toward the nearest ball. which means left ball.
				turn_left_slow();
				t++;
			}
			if (i > 0) move_backward(0); //Move backward for a short time for no red ball detection. for satety!
			if (toward_blueballs() == 1) { //For extreme case such as blue ball no found!, toward_blueballs start!
				i++;
			}
			else {
				cout << "Toward blue failed with unknown error! Retry" << endl;//Retry i-th trial, not happening many times
				continue;
			}
			ros::spinOnce();
			for (int i = 0; i < t; i++) {//After picking up nearest ball, try to re align. before that, camera need to detect blue ball.
				turn_right_slow();
			}
			while (blue_X[rightmost_blue] > 60) {//Align to the rightmost ball again.
				turn_right_slow();
			}
			check = 0;
		}
		else {//If checker is not triggered, just start toward_blueballs.
			cout << "Toward blue ball starts!" << endl;
			if (i > 0) move_backward(0); //Move backward for a short time for no red ball detection.
			if (toward_blueballs() == 1) { //For extreme case such as blue ball no found!
				i++;
			}
			else {
				cout << "Toward blue failed with unknown error! Retry" << endl;//Retry i-th trial, not happening many times
				continue;
			}
		}

	}
	dataInit();
	write(c_socket, data, sizeof(data)); //Data Initialize for stop moving
	move_backward(0); //Move backward for a short time. In case of too close to the middle position.
	return_to_base();
}




/*****************************************************Callback 함수들 ***********************************************************/

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  blue_number = position->blue_size;   // put number of blue balls
  near_blue = 0;                       // initialize index
  near_center_blue = 0;
  rightmost_blue = 0;
  for(int i = 0; i < blue_number; i++)
    {
        blue_X[i] = (position->blue_x[i])+110;  // revise the value of x coordinate of blue ball correctly
        blue_Y[i] = position->blue_y[i];        // put y coordinate of blue ball
        blue_distance[i] = position->blue_distance[i];    // put distance of blue ball
        if(blue_distance[i]<blue_distance[near_blue]){    // put index of the nearest blue ball 
          near_blue = i;
        }
        if(abs(blue_X[i])<abs(blue_X[near_center_blue])){ // put index of blue ball at center 
          near_center_blue = i;
        }
        if(blue_X[rightmost_blue]<blue_X[i]){             // put index of rightmost blue ball
          rightmost_blue = i;
        }
    }
  red_number = position->red_size;     // put number of red balls
  near_red = 0;                        // initialize index
  for(int i = 0; i < red_number; i++)  
    {
        red_X[i] = (position->red_x[i])+110;         // revise the value of x coordinate of red ball correctly
        red_Y[i] = position->red_y[i];               // put y coordinate of red ball
        red_distance[i] = position->red_distance[i]; // put distance of red ball
        if(red_Y[i]<red_Y[near_red]){                // put index of the nearest red ball
          near_red = i;
        }
    }
  green_number = position->green_size;               // put number of green balls
  rightmost_green = 0;                               // initailize index   
  for(int i = 0; i < green_number; i++)              
    {
      green_X[i] = (position->green_x[i])+110;         // revise the value of x coordinate of gree ball correctly
      green_Y[i] = position->green_y[i];               // put y coordinate of green ball
      green_distance[i] = position->green_distance[i]; // put distance of green ball
      if(green_X[rightmost_green]<green_X[i]){         // put index of the nearest green ball
          rightmost_green = i;
        }
    }
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  map_mutex.lock();  // protect collision of two callback functions
  int count = scan->scan_time / scan->time_increment; // compute lidar size by scan message
  lidar_size=count;  // put size of lidar angle data
  zero_degree = 0;   // initialize zero_degree
  least_distance = 0;  // initialize least_distance 
  for(int i = 0; i < count; i++)
    {
      lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);  // put real angle of robot that unit is degree
      if(static_cast<int>(lidar_degree[i]+0.5) == 0){zero_degree = i;}  // check that the angle is zero
      lidar_distance[i]=scan->ranges[i];  // put distances of robot at each angles
      if(lidar_distance[i]<lidar_distance[least_distance]){least_distance = i;}  // put the least distance of robot
    }
  if(isinf(scan->ranges[zero_degree])){   // check range is infinite
    lidar_zero_degree = lidar_zero_degree; 
  }
  else{
    lidar_zero_degree = scan->ranges[zero_degree];  // put the distance of robot when is zero
  }
  map_mutex.unlock();
}

int main(int argc, char **argv){

  ros::init(argc, argv, "data_integrate_node");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe<core_msgs::ball_position>("/ball_position", 100, camera_Callback); // subscribe ball position data by camera
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);  // subscribe robot position data by lidar

  c_socket = socket(PF_INET, SOCK_STREAM, 0);  // TCP/IP
  c_addr.sin_addr.s_addr = inet_addr(IPADDR);  // change IP address type from string to long int
  c_addr.sin_family = AF_INET;                 // put IPv4 internet protocol to address family 
  c_addr.sin_port = htons(PORT);               // change short memory value from Host byte of PORT to Network byte
  connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr));  // request to connect server through c_socket 

  start_path_finding();   // do start_path_finding
}
