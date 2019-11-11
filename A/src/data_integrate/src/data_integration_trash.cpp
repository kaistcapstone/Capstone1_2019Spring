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
using namespace std;


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"


#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int red_ball_number;
int blue_ball_number;
float r_ball_X[20];
float r_ball_Y[20];
float b_ball_X[20];
float b_ball_Y[20];
float ball_distance[20];
int near_ball;

int action;

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];

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

// void turn_right(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }

void turn_left(){
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = -1;
  data[5] = 0;
}

void move_forward(){
  data[0] = 0;
  data[1] = 1;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
}

// void move_diagonal_l(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
// void move_diagonal_r(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
// void move_back(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
// void move_left(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
// void move_right(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
void stop(){
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
}
//
// void roller_on(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
// void roller_back(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }
//
// void roller_stop(){
//   data[0] = ;
//   data[1] = ;
//   data[2] = ;
//   data[3] = ;
//   data[4] = ;
//   data[5] = ;
// }

// void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
// 		map_mutex.lock();
//
//     int count = scan->scan_time / scan->time_increment;
//     lidar_size=count;
//     for(int i = 0; i < count; i++)
//     {
//         lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
//         lidar_distance[i]=scan->ranges[i];
//     }
// 		map_mutex.unlock();
//
// }
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int r_count = position->r_size;
		red_ball_number = r_count;
		int b_count = position->b_size;
		blue_ball_number = b_count;

    for(int r = 0; r < r_count; r++)
    {
        r_ball_X[r] = position->r_img_x[r];
        r_ball_Y[r] = position->r_img_y[r];
    }

    for(int b = 0; b < b_count; b++)
    {
        b_ball_X[b] = position->b_img_x[b];
        b_ball_Y[b] = position->b_img_y[b];
    }

}
void find_ball()
{
	data[20]=1;
}


int main(int argc, char **argv)
{
	  std::cout << "main start"<< endl;
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;
		ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);


		//
    // ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);


    // c_socket = socket(PF_INET, SOCK_STREAM, 0);
    // c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    // c_addr.sin_family = AF_INET;
    // c_addr.sin_port = htons(PORT);

		///////////////////////////////////////////////////////////////////////
		//	렙뷰와 통신이 되었는지 확인하는 코드 아래 코드를 활성화 후 노드를 실행 시켰을때///
		//	노드가 작동 -> 통신이 연결됨, Failed to connect 이라고 뜸 -> 통신이 안됨///
		////////////////////////////////////////////////////////////////////////
    // if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
    //     printf("Failed to connect\n");
    //     close(c_socket);
    //     return -1;
    // }

    std::cout << "while start"<< endl;
    while(ros::ok){

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// // 각노드에서 받아오는 센서 테이터가 잘 받아 왔는지 확인하는 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		/////////////////////////////////////////////////////////////////////////////////////////////////

	 //  for(int i = 0; i < lidar_size; i++
   // {
	 //    std::cout << "degree : "<< lidar_degree[i];
	 //    std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
	 //  }

	 std::cout << "cor show"<< endl;

		for(int r = 0; r < red_ball_number; r++)
		{
			std::cout << "red_ball_X : "<< r_ball_X[r];
			std::cout <<"\t";
			std::cout << "red_ball_Y : "<< r_ball_Y[r]<<std::endl;

		}
		for(int b = 0; b < blue_ball_number; b++)
		{
			std::cout << "blue_ball_X : "<< b_ball_X[b];
			std::cout <<"\t";
			std::cout << "blue_ball_Y : "<< b_ball_Y[b]<<std::endl;
		}



		////////////////////////////////////////////////////////////////
		// // 자율 주행을 예제 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		////////////////////////////////////////////////////////////////
		// dataInit();
		// for(int i = 0; i < lidar_size-1; i++)
		// 	    {
		// 		if(lidar_distance[i]<lidar_distance[i+1]){lidar_obs=i;}
		// 		else if(lidar_distance[i]==lidar_distance[i+1]){lidar_obs=i;}
		// 		else {lidar_obs=i+1;}
		// 	    }
		// if(ball_number==0 || lidar_obs<0.3)
		// {
		// 		find_ball();
		// }
		// else
		// {
		// 	for(int i = 0; i < ball_number-1; i++)
		// 	    {
		// 		if(ball_distance[i]<ball_distance[i+1]){near_ball=i;}
		// 		else if(ball_distance[i]==ball_distance[i+1]){near_ball=i;}
		// 		else {near_ball=i+1;}
		// 	    }
		// 	if(ball_distance[near_ball]<0.1){data[4]=0; data[5]=0; data[21]=0;}
		// 	else
		// 	{
		// 		data[20]=1;
		// 		if(ball_X[near_ball]>0){data[4]=1;}  else{data[4]=-1;}
		// 		if(ball_Y[near_ball]>0){data[5]=1;}  else{data[5]=-1;}
		// 	}
		// }

		//자율 주행 알고리즘에 입력된 제어데이터(xbox 컨트롤러 데이터)를 myRIO에 송신(tcp/ip 통신)
	      // for (int i = 0; i < 24; i++){
	      // printf("%f ",data[i]);
				//
	      // }
	      // printf("\n");
			// printf("%d\n",action);


			//
			// cout<<"move forward"<<endl;
			// for(int j=0; j<33; j++){		//0.6m/s -> 1.66s(33frame) required for 1m move forward.
			// 	move_forward();
			// 	write(c_socket, data, sizeof(data));
			// 	cout<<"move forward"<<endl;
			// 	ros::Duration(0.05).sleep();
			// }
			//
			// cout<<"turn left"<<endl;
			// for(int j=0; j<25; j++){
			// 	turn_left();
			// 	cout<<"turn left"<<endl;
			// 	write(c_socket, data, sizeof(data));
			// 	ros::Duration(0.05).sleep();
			// }
			//
			// cout<<"turn and move forward"<<endl;
			// for(int j=0; j<33; j++){
			// 	move_forward();
			// 	cout<<"turn and move forward"<<endl;
			// 	write(c_socket, data, sizeof(data));
			// 	ros::Duration(0.05).sleep();
			// }
			//
			// cout<<"stop"<<endl;
			// stop();
			// write(c_socket, data, sizeof(data));
			// ros::Duration(2).sleep();
			//
			// break
			dataInit();
			ros::spinOnce();
    }
    return 0;
}
