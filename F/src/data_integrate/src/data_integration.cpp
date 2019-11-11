#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"


#include "opencv2/opencv.hpp"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
//#define IPADDR "127.0.0.1" // myRIO ipadress
#define IPADDR "172.16.0.1"
boost::mutex map_mutex;




//ball_detect node에서 받은 msg를 저장하기 위한 여러 변수들을 선언하는 코드이다.
int r_ball_number; //red ball의 갯수를 저장한다.
float r_ball_X[20]; //red ball의 x위치를 저장한다.
float r_ball_Y[20]; //red ball의 y위치를 저장한다.
float r_ball_Z[20]; //red ball의 z위치를 저장한다.
float r_ball_angle[20]; //red ball의 x축 각도를 저장한다.
float r_ball_distance[20]; //red ball의 거리를 저장한다.
vector<vector<float>> r_posi; //모든 red ball의 위치정보를 갖고있는 벡터 선언
vector<float> nearest_r_posi; //가장 가까운 red ball의 위치 정보를 갖고있는 벡터 선언

//아래는 모두 같은 형식이다.
int b_ball_number;
float b_ball_X[20];
float b_ball_Y[20];
float b_ball_Z[20];
float b_ball_angle[20];
float b_ball_distance[20];
vector<vector<float>> b_posi;
vector<float> nearest_b_posi;
vector<float> rightest_b_posi;


int b_ball_number2;
float b_ball_X2[90];
float b_ball_Y2[90];
float b_ball_Z2[90];
float b_ball_angle2[90];
float b_ball_distance2[90];
vector<vector<float>> b_posi2;
vector<float> nearest_b_posi2;

int g_ball_number;
float g_ball_X[20];
float g_ball_Y[20];
float g_ball_Z[20];
float g_ball_angle[20];
float g_ball_distance[20];
vector<vector<float>> g_posi;
vector<float> rightest_g_posi;
vector<float> g_mid;

int g_ball_number2;
float g_ball_X2[20];
float g_ball_Y2[20];
float g_ball_Z2[20];
float g_ball_angle2[20];
float g_ball_distance2[20];
vector<vector<float>> g_posi2;
vector<float> rightest_g_posi2;


int pick_ball_num = 0;//주운 공의 개수
int m = 1;//회피기동시 왼, 오 구분
int phase = 1;//align, moving, pickup, release
float theta_c = 6;//align before moving 허용 오차 각
float theta_c_i = 8;//align before moving initial 허용 오차 각
float theta_c_ii = 4;//픽업파트 내에서 두번째 웹캡으로 얼라인시 허용 오차 각
float theta_c_iii = 4;//픽업파트 내에서 두번째 웹캠으로 얼라인시 허용 오차 각
float theta_car1; // 웹캠이 차 중앙이 있지 않으니까 두개로 나눔 line 806 참고, 자동차 끝의 z축 평행선과 공의 x축 평생선이 만나는 지점이 z축과 이루는 각도
float theta_car2;//웹캠 2 기준
float car_half_width1 = 0.3;//웹캠 1에서의 차의 너비
float car_half_width2 = 0.2;//웹캠 2에서의 차의 너비
float theta_avoid = 3;//회피기동시 회피할 각도
float r_c = 0.70;//pickup시작
float l_c = 0.9;//빨간볼 회피기동 시작거리
float p_c = 0.52;//pickup시작거리
float g_c = 0.7;//final align 시작거리
float p_c_g = 0.58;//release 거리
float x_f = 0.08;//xaxis align 허용 오차
float z_diff=0.3;//정면수직 정렬 허용 오차
float b_a = 1.13;//회피기동후 align시 blue ball 과의 거리//1.1
float b_aa = 1.25; // 회피기동후 일단 앞으로 갈 거리 //1.2
float b_a_g = 1.3;
int p = 0; //release
float g_x = 0.15;//마지막 release전 초록공 정렬
float g_dis = 2;//정렬 이동하기전 직진 구간
int u = 0; // 가까운 블루공이 오른쪽 공보다 가까이 있을 떄
float b_avoid = 2;//블루공 회피 각도

float v=0;//motor speed
float v0=0;//motor speed origin
int f_No=14;//이동함수 구별 넘버
int kf=9;//경로함수 구간 길이
int k=1;//경로함수 현재 구간
int p_f_No=0;//이전 이동함수 넘버
int kf_s=4;//nothing



int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];

#define RAD2DEG(x) ((x)*180./M_PI)

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


//거리가 0으로 뜨는 에러를 지우기 위한 코드이다. 거리가 0.01보다 작을 경우 거리를 10m으로 만들어 사용할 수 없게 만든다.
vector<float> remove_error_nearest(vector<float> est){
	if(!est.empty()){
		if(est[4]<0.01){
			est[0] = 0;
			est[1] = 0;
			est[2] = 0;
			est[3] = 0;
			est[4] = 10;
		}
	}
	return est;
}


//가장 가까운 공의 위치 정보를 저장해서 nearest_posi로 반환하는 함수이다.
vector<float> find_nearest_ball_position(vector<vector<float>> posi){
	vector<float> nearest_posi;
	if(posi.empty()){
	}
	else{//i번쨰와 j번째를 비교하는 것을 반복해 가장 가까운 공의 위치를 반환한다.
		for(int i=0; i<posi.size(); i++){
			//거리가 0으로 뜨는 에러를 지우기 위해 거리가 0.01보다 클 때 실행되도록한다.
			if(posi[i][4]>0.01){
				nearest_posi = posi[0];

				for(int j=0; j<posi.size(); j++){
					if(nearest_posi[4]>0.01){ //한번 더 거리가 0으로 뜨는 에러를 지우기 위한 부분이다.
						if(nearest_posi[4]<posi[j][4]){
						}
						else{
							nearest_posi = posi[j];
						}
					}
					remove_error_nearest(nearest_posi);
				}
			}
			else{

			}
		}
	}
		return nearest_posi;
}

//웹캠2에서는 x좌표가 0.25보다 작은 경우에 대해서만 구하기 위해 비슷한 함수를 정의한 것이다.
//웹캠2의 경우 카메라가 뒤집혀있기 떄문에 부호가 달라 조금 수정한 코드이다.
vector<float> find_nearest_ball_position2(vector<vector<float>> posi){
	vector<float> nearest_posi;
	if(posi.empty()){
	}
	else{
		for(int i=0; i<posi.size(); i++){
			if(posi[i][0]<0.25 && posi[i][4]>0.01){
				nearest_posi = posi[0];

				for(int j=0; j<posi.size(); j++){
					if(nearest_posi[4]>0.01){
						if(nearest_posi[4]<posi[j][4]){
						}
						else{
							nearest_posi = posi[j];
						}
					}
					remove_error_nearest(nearest_posi);
				}
			}
		else{

			}
		}
	}
		return nearest_posi;
}

//가장 오른쪽에 있는 공의 위치 정보를 반환해주는 함수이다. nearest 공의 위치를 구하는 함수와 같은 알고리즘이다.
vector<float> find_rightest_ball_position(vector<vector<float>> posi){
	vector<float> nearest_posi;
	if(posi.empty()){
	}
	else{
		nearest_posi = posi[0];

		for(int i=0; i<posi.size(); i++){
			if(posi[i][4]>0.01){
				if(nearest_posi[3]>posi[i][3]){
				}
				else{
					nearest_posi = posi[i];
				}
			}
		}
	}
		return nearest_posi;
}

//웹캠2의 경우 카메라가 뒤집혀있기 떄문에 부호가 달라 조금 수정한 코드이다.
vector<float> find_rightest_ball_position2(vector<vector<float>> posi){
	vector<float> nearest_posi;
	if(posi.empty()){
	}
	else{
		nearest_posi = posi[0];

		for(int i=0; i<posi.size(); i++){
			if(posi[i][4]>0.01){
				if(nearest_posi[3]<posi[i][3]){
				}
				else{
					nearest_posi = posi[i];
				}
			}
		}
	}
		return nearest_posi;
}



//msg를 받아 callback함수를 통해 위치 정보를 저장하는 코드이다.
//posi라는 벡터안에 x,y,z,theta,distance순서로 이루어진 벡터를 넣는 것이다.
//가장 가까운 위치의 공의 정보가 필요한 경우 nearest를 찾는 함수를 사용하고 가장 오른쪽 위치의 공의 정보가 필요한 경우 rightest를 찾는 함수를 사용한다.
//각각의 웹캠과 색깔의 공에 따라 callback함수를 각각 정의한 것이다.
void red_ball_callback(const core_msgs::ball_position::ConstPtr& position){
	r_posi.clear();
	nearest_r_posi.clear();
	//ball number 갯수만큼 for문을 돌려서 r_p에 저장한 후 이 r_p로 이루어진 벡터인 r_posi에 저장한다.
	vector<float> r_p;
    r_ball_number = position->size;
    for(int i = 0; i < r_ball_number; i++)
    {
				r_p.clear();
        r_ball_X[i] = position->img_x[i];
        r_ball_Y[i] = position->img_y[i];
        r_ball_Z[i] = position->img_z[i];
				r_ball_angle[i] = RAD2DEG(atan(r_ball_X[i]/r_ball_Z[i]));
				r_ball_distance[i] = sqrt(r_ball_X[i]*r_ball_X[i]+r_ball_Y[i]*r_ball_Y[i]+r_ball_Z[i]*r_ball_Z[i]);
				r_p.push_back(r_ball_X[i]);
				r_p.push_back(r_ball_Y[i]);
				r_p.push_back(r_ball_Z[i]);
				r_p.push_back(r_ball_angle[i]);
				r_p.push_back(r_ball_distance[i]);
				r_posi.push_back(r_p);
    }
		nearest_r_posi = find_nearest_ball_position(r_posi);
}

void blue_ball_callback(const core_msgs::ball_position::ConstPtr& position){
	b_posi.clear();
	nearest_b_posi.clear();
	rightest_b_posi.clear();
	vector<float> b_p;
  b_ball_number = position->size;
  for(int i = 0; i < b_ball_number; i++)
  {
			b_p.clear();
      b_ball_X[i] = position->img_x[i];
      b_ball_Y[i] = position->img_y[i];
      b_ball_Z[i] = position->img_z[i];
			b_ball_angle[i] = RAD2DEG(atan(b_ball_X[i]/b_ball_Z[i]));
			b_ball_distance[i] = sqrt(b_ball_X[i]*b_ball_X[i]+b_ball_Y[i]*b_ball_Y[i]+b_ball_Z[i]*b_ball_Z[i]);
			b_p.push_back(b_ball_X[i]);
			b_p.push_back(b_ball_Y[i]);
			b_p.push_back(b_ball_Z[i]);
			b_p.push_back(b_ball_angle[i]);
			b_p.push_back(b_ball_distance[i]);

			b_posi.push_back(b_p);
    }
		nearest_b_posi = find_nearest_ball_position(b_posi);
		rightest_b_posi = find_rightest_ball_position(b_posi);
}

void blue_ball_callback2(const core_msgs::ball_position::ConstPtr& position){
	b_posi2.clear();
	nearest_b_posi2.clear();
	vector<float> b_p;
  b_ball_number2 = position->size;
  for(int i = 0; i < b_ball_number2; i++)
  {
			b_p.clear();
      b_ball_X2[i] = position->img_x[i];
      b_ball_Y2[i] = position->img_y[i];
      b_ball_Z2[i] = position->img_z[i];
			b_ball_angle2[i] = RAD2DEG(atan(b_ball_X2[i]/b_ball_Z2[i]));
			b_ball_distance2[i] = sqrt(b_ball_X2[i]*b_ball_X2[i]+b_ball_Y2[i]*b_ball_Y2[i]+b_ball_Z2[i]*b_ball_Z2[i]);
			b_p.push_back(b_ball_X2[i]);
			b_p.push_back(b_ball_Y2[i]);
			b_p.push_back(b_ball_Z2[i]);
			b_p.push_back(b_ball_angle2[i]);
			b_p.push_back(b_ball_distance2[i]);

			b_posi2.push_back(b_p);
    }
		nearest_b_posi2 = find_nearest_ball_position2(b_posi2);

}


void green_ball_callback(const core_msgs::ball_position::ConstPtr& position){
	g_posi.clear();
	vector<float> g_p;
  g_ball_number = position->size;
  for(int i = 0; i < g_ball_number; i++)
  {
			g_p.clear();
      g_ball_X[i] = position->img_x[i];
      g_ball_Y[i] = position->img_y[i];
      g_ball_Z[i] = position->img_z[i];
			g_ball_angle[i] = RAD2DEG(atan(g_ball_X[i]/g_ball_Z[i]));
			g_ball_distance[i] = sqrt(g_ball_X[i]*g_ball_X[i]+g_ball_Y[i]*g_ball_Y[i]+g_ball_Z[i]*g_ball_Z[i]);
			g_p.push_back(g_ball_X[i]);
			g_p.push_back(g_ball_Y[i]);
			g_p.push_back(g_ball_Z[i]);
			g_p.push_back(g_ball_angle[i]);
			g_p.push_back(g_ball_distance[i]);

			g_posi.push_back(g_p);
    }
    if(g_ball_number>=2){
        g_mid.clear();
        g_mid.push_back((g_posi[0][3]+g_posi[1][3])/2);//angle
        g_mid.push_back((g_posi[0][4]+g_posi[1][4])/2);//distance
        if(g_posi[0][0]>g_posi[1][0]){//right ball - left ball
          g_mid.push_back(g_posi[0][2]-g_posi[1][2]);//z difference
        }
        else{
          g_mid.push_back(g_posi[1][2]-g_posi[0][2]);//z difference
        }
        g_mid.push_back((g_posi[0][0]+g_posi[1][0])/2);//x
    }
		rightest_g_posi = find_rightest_ball_position(g_posi);
}

void green_ball_callback2(const core_msgs::ball_position::ConstPtr& position){
	g_posi2.clear();
	rightest_g_posi2.clear();
	vector<float> g_p;
  g_ball_number2 = position->size;
  for(int i = 0; i < g_ball_number2; i++)
  {
			g_p.clear();
      g_ball_X2[i] = position->img_x[i];
      g_ball_Y2[i] = position->img_y[i];
      g_ball_Z2[i] = position->img_z[i];
			g_ball_angle2[i] = RAD2DEG(atan(g_ball_X2[i]/g_ball_Z2[i]));
			g_ball_distance2[i] = sqrt(g_ball_X2[i]*g_ball_X2[i]+g_ball_Y2[i]*g_ball_Y2[i]+g_ball_Z2[i]*g_ball_Z2[i]);
			g_p.push_back(g_ball_X2[i]);
			g_p.push_back(g_ball_Y2[i]);
			g_p.push_back(g_ball_Z2[i]);
			g_p.push_back(g_ball_angle2[i]);
			g_p.push_back(g_ball_distance2[i]);
			g_posi2.push_back(g_p);
    }
	rightest_g_posi2 = find_rightest_ball_position2(g_posi2);
}


float path_3(float v0,float vf,int kf,int k){//3th order polynomial path, current velocity v0, final velocity vf, 구간의 길이 kf, 현재구간 k
	if(k > kf) k = kf;      //restrict speed
	float a0=v0;            //coefficient setting
	float a2=3*(vf-v0)/pow(kf,2);
	float a3=-2*(vf-v0)/pow(kf,3);
	float y=a0+a2*pow(k,2)+a3*pow(k,3);
	return y;
}

void go_straight(){//No.1 speed : 55
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = 0;
	data[6] = 55;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void go_straight_slowly(){//No.2 speed : 30
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = 0;
	data[6] = 30;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void go_left(){//No.3 speed : 35
	data[0] = -1;//left & right
	data[1] = 0;
	data[4] = 0;
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void go_right(){//No.4 speed : 35
	data[0] = 1;//left & right
	data[1] = 0;
	data[4] = 0;
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_ccw(){//No.5 speed : 20
	data[0] = 0;
	data[1] = 0;
	data[4] = -1;//turn ccw & cw
	data[6] = 20;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_ccw_slowly(){//No.6 speed : 10
	data[0] = 0;
	data[1] = 0;
	data[4] = -1;//turn ccw & cw
	data[6] = 10;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_ccw_so_slowly(){//No.7 speed : 10
	data[0] = 0;
	data[1] = 0;
	data[4] = -1;//turn ccw & cw
	data[6] = 10;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 1;
}

void turn_cw(){//No.8 speed : 20
	data[0] = 0;
	data[1] = 0;
	data[4] = 1;//turn ccw & cw
	data[6] = 20;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_cw_slowly(){//No.9 speed : 10
	data[0] = 0;
	data[1] = 0;
	data[4] = 1;//turn ccw & cw
	data[6] = 10;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_cw_so_slowly(){//No.10 speed : 10
	data[0] = 0;
	data[1] = 0;
	data[4] = 1;//turn ccw & cw
	data[6] = 10;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 1;
}

void pick_up(){//No.11
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = 0;
	if(f_No == 11){//pickup이 이어지도록
		v = path_3(v0, 0, kf, k);
		data[6] = v;
	}
	else if(f_No == 1 || f_No == 2){//직진 중 pickup시 이어지도록
		k=1;
		v0=v;
		v = path_3(v, 0, kf, k);
		data[6] = v;
	}
	else{//직진 이외의 경우 바로 멈춰서 pickup으로
		v=0;
		v0=0;
		k=1;
	}
	k++;
	data[6] = v;
	f_No = 11;
	data[8] = 0;
	data[9] = 1;//pick up motion
	data[15] = 0;
	data[16] = 0;
}

void release1(){//No.12
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = 0;
	data[6] = 7;//speed
	data[8] = 1;//release motion
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void release2(){//No.12
	data[0] = 0;
	data[1] = -1;//back & forth
	data[4] = 0;
	data[6] = 7;// speed
	data[8] = 1;//release motion
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void go_left_slowly(){//No.13 speed : 20
	data[0] = -1;//left & right
	data[1] = 0;
	data[4] = 0;
	data[6] = 20;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void go_right_slowly(){//No.14 speed : 20
	data[0] = 1;//left & right
	data[1] = 0;
	data[4] = 0;
	data[6] = 20;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_ccw_fast(){//speed : 55
	data[0] = 0;
	data[1] = 0;
	data[4] = -1;//turn ccw & cw
	data[6] = 55;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void go_back(){//speed : 20
	data[0] = 0;
	data[1] = -1;//back & forth
	data[4] = 0;
	data[6] = 20;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}
void turn_ccw_go(){//speed : 35
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = -1;//turn ccw & cw
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}
void turn_cw_go(){//speed : 35
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = 1;//turn ccw & cw
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_ccw_go_r(){//No.5 speed : 35
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = -1;//turn ccw & cw
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}
void turn_cw_go_r(){//speed : 35
	data[0] = 0;
	data[1] = 1;//back & forth
	data[4] = 1;//turn ccw & cw
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_ccw_little_fast(){//speed : 35
	data[0] = 0;
	data[1] = 0;
	data[4] = -1;//turn ccw & cw
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}

void turn_cw_little_fast(){//speed : 35
	data[0] = 0;
	data[1] = 0;
	data[4] = 1;//turn ccw & cw
	data[6] = 35;//speed
	data[8] = 0;
	data[9] = 0;
	data[15] = 0;
	data[16] = 0;
}


void dup_go_straight(){//파란공이 일직선 상에 있어 겹쳐보일때 회피후 조금 앞으로 가게 하는 함수
	if(b_ball_number == 2 && b_ball_number2 == 1 && abs(b_posi[0][0]-b_posi[1][0]) < 0.2){
		cout<<"dup haha"<<endl;
		int k =1;
		while(k<50){
			go_straight_slowly();
			write(c_socket, data, sizeof(data));//open loop이므로 매 루프마다 data 송신
			k++;
			ros::Duration(0.02).sleep();//0.02초 term 두기
		}
	}
}



int main(int argc, char **argv) // 메인함수 작성
{
    ros::init(argc, argv, "data_integation"); // 노드명 초기화
    ros::NodeHandle n; // 노드 핸들 선언

    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/redball_info", 1, red_ball_callback); // 서브스크라이버 선언
    ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position>("/blueball_info", 1, blue_ball_callback);
    ros::Subscriber sub22 = n.subscribe<core_msgs::ball_position>("/blueball_info2", 1, blue_ball_callback2);
    ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/greenball_info", 1, green_ball_callback);
    ros::Subscriber sub33 = n.subscribe<core_msgs::ball_position>("/greenball_info2", 1, green_ball_callback2);
		dataInit();

    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){ // 랩뷰와의 통신이 에러가 떴을 때 확인하기 위한 코드
        printf("Failed to connect\n");
        close(c_socket);
        return -1;
    }

		int j =1; // j 선언 및 밑의 open loop을 돌리기 위한 j값 설정
		while(j<8){ // 가장 오른쪽 공을 detect 하기 위해 시작하자마자 살짝 오른쪽으로 회전하게끔 했다
			turn_cw(); // 시계방향으로 회전
			write(c_socket, data, sizeof(data)); // 랩뷰로 통신할 데이터를 write한다
			j++; // j=j+1
			ros::Duration(0.02).sleep(); //duration을 0.02로 설정하여 일정한 시간동안 while문이 돌아가게 만든다
		}



		ros::Rate loop_rate(20);

    while(ros::ok){

    ros::spinOnce();



    if(pick_ball_num == 0){ // 픽업한 공의 개수가 0개 일 때
      switch(phase){ // phase의 초기값은 글로벌 변수 선언에서 1로 설정되어 있음

        case 1: // align 단계. 즉, 차체가 가장 오른쪽에 있는 파란공을 파라보게끔 만드는 단계
        if(b_ball_number == 0){ // 카메라1에 보이는 파란공의 개수가 0일 때. 카메라1은 위쪽에 있어서 멀리 있는 공을 탐지할 때 쓰인다.
          if(m == 0){ // case2에서 빨간공을 회피할 때 차체가 오른쪽 또는 왼쪽으로 움직이게 되고 그에 따라 m값을 0 또는 1로 설정해준다. 만약 공을 잃어버려서 case1으로 다시 오게 된다면 원래 case1에서 align한 공을 다시 찾게끔 만들어준다
						turn_ccw(); // 시계반대방향으로 회전
          }
          else{ // m이 1일 때
						turn_cw(); // 시계방향으로 회전
          }
        }
        else{ // 파란공의 개수가 0이 아닐 때
          if(abs(rightest_b_posi[3]) > theta_c_i){ // 가장 오른쪽에 있는 파란공과 차체가 바라보고 있는 방향의 각도가 theta_c_i 이상일 경우. 이 때는 차체를 회전시켜서 align한다
            if(rightest_b_posi[3] > 0){ // 파란공이 차체가 바라보고 있는 방향 기준으로 오른쪽에 있을 경우
							turn_cw_slowly(); // 시계방향으로 천천히 회전
            }
            else{ // 파란공이 차체가 바라보고 있는 방향 기준으로 왼쪽에 있을 경우
							turn_ccw_slowly(); // 시계반대방향으로 천천히 회전
            }
          }
          else{ // 위의 두 경우가 아닌 경우, 즉, align이 끝난 경우
            phase = 2; // phase를 2로 설정하여 case2가 실행되도록 한다
          }
        }
        break; // break하여 swith문을 빠져나온다

        case 2: //move around 단계. 즉, case1에서 align한 파란공까지 장애물이 있다면 피하면서 다가가는 단계이다.
				if(b_ball_number == 0){ // 카메라1에 보이는 파란공의 개수가 0일 때
					phase = 1; // phase를 1로 설정하여 다시 case1이 실행되도록 한다
				}
				else if (abs(RAD2DEG(atan(car_half_width1/nearest_b_posi[2])) + theta_avoid) > abs(nearest_b_posi[3]) && nearest_b_posi[4] < rightest_b_posi[4] && nearest_b_posi[4] != rightest_b_posi[4]){ // 가장 오른쪽에 있는 파란공에 align된 상태에서 거기까지 가는 직선 경로 안에 파란공이 하나 더 있을 경우. 이 경우 가까운 파란공이 목표가 된다
						if(!nearest_r_posi.empty()){ // 빨간공이 카메라에 보일 경우
	            if(nearest_r_posi[4]<nearest_b_posi[4]){ // 가장 가까운 빨간공이 가장 가까운 파란공보다 가까이 있을 경우
								if(nearest_r_posi[4]>l_c){ // 가장 가까운 빨간공과의 거리가 l_c보다 클 경우
									if(rightest_b_posi[0]<0 && abs(rightest_b_posi[3])>theta_c){ // 가장 오른쪽에 있는 공의 위치가 차체 방향기준 왼쪽에 있고, 각도가 theta_c보다 클 때.
										turn_ccw_go(); // 앞으로 전진하면서 시계반대방향으로 회전
									}
									else if(rightest_b_posi[0]>0 && abs(rightest_b_posi[3])>theta_c){ // 가장 오른쪽에 있는 공의 위치가 차체 방향기준 오른쪽에 있고, 각도가 theta_c보다 클 때
										turn_cw_go(); // 앞으로 전진하면서 시계방향으로 회전
									}
									else{ // 위의 두 경우가 아닌 경우. 즉, 가장 오른쪽 파란공에 align 된 상태일 때
										go_straight(); // 앞으로 전진
									}
								}
								else{ // 가장 가까운 빨간공과의 거리가 l_c보다 작을 경우
		              theta_car1 = RAD2DEG(atan(car_half_width1/nearest_r_posi[2])); // (가장 가까운 빨간공의 z좌표)와 (웹캠중심과 차 왼쪽 끝의 거리)를 이용하여 각도를 구한다. 즉 이 각도는 빨간공의 z좌표를 가지고 자 왼쪽 끝의 x좌표를 가지는 좌표의 각도이다. 웹캠이 차의 중심에 달려있지 않아 두 개의 각도를 구한다
									theta_car2 = RAD2DEG(atan(car_half_width2/nearest_r_posi[2])); // (가장 가까운 빨간공의 z좌표)와 (웹캠중심과 차 오른쪽 끝의 거리)를 이용하여 각도를 구한다. 즉 이 각도는 빨간공의 z좌표를 가지고 자 오른쪽 끝의 x좌표를 가지는 좌표의 각도이다

		              if(abs(theta_car2 + theta_avoid)>abs(nearest_r_posi[3]) && nearest_r_posi[3]>0){ // 빨간공이 차체가 파란공까지 가는 직선경로 안에 들어와있고 빨간공이 차체 중심보다 오른쪽에 있을 때 . theta_avoid 값을 넣어줘서 오차를 예상하여 더 옆으로 피하게끔 만들었다
										go_left(); // 왼쪽으로 차를 움직인다
		                m = 1; // m을 1로 설정하여 만약 공을 잃어버렸을 경우 case1로 갔을 때 시계방향으로 회전하여 공을 찾게 한다
		              }
		              else if(abs(theta_car1 + theta_avoid)>abs(nearest_r_posi[3]) && nearest_r_posi[3]<0){ // 빨간공이 차체가 파란공까지 가는 직선경로 안에 들어와있고 빨간공이 차체 중심보다 왼쪽에 있을 때
										go_right(); // 오른쪽으로 차를 움직인다
		                m = 0; // m을 0로 설정하여 만약 공을 잃어버렸을 경우 case1로 갔을 때 반시계방향으로 회전하여 공을 찾게 한다
		              }
									else{ // 가장 가까운 빨간공이 가장 가까운 파란공보다 가까이 있지만 파란공까지 가는 직선경로 안에 빨간공이 없는 경우 및 빨간공을 옆으로 피한 경우
										if(nearest_b_posi[4] > b_aa){ // 가장 가까운 파란공과의 거리가 b_aa보다 클 경우
											go_straight(); // 앞으로 전진한다
										}
										else if(nearest_b_posi[4] > b_a){ // 가장 가까운 파란공과의 거리가 b_aa보다 작고 b_a보다 큰 경우. 즉, 빨간 공 피했는데 바로 얼라인 하면 안되니까 일단 어느정도 거리까지 앞으로 가게함
											if(nearest_b_posi[0] < 0 && abs(nearest_b_posi[3])>theta_c_i){ // 가장 가까운 파란공이 왼쪽에 있으면서 각도가 theta_c보다 클 때
												turn_ccw_go_r(); // 앞으로 전진하면서 반시계방향으로 조금 회전한다. 다만 turn_ccw_go 함수보다 회전하는 속도가 느려서 주로 앞으로 전진하게 된다. 빨간공을 옆으로 피하고서 앞으로 갈 때 원래 얼라인했던 파란공을 잃어버릴 수 있기에 조금씩 얼라인을 병향하면서 앞으로 전진시키도록 했다
											}
											else if (nearest_b_posi[0] > 0 && abs(nearest_b_posi[3])>theta_c_i) { // 가장 가까운 파란공이 오른쪽에 있으면서 각도가 theta_c보다 클 때
												turn_cw_go_r(); // 앞으로 전진하면서 시계방향으로 조금 회전한다
											}
											else{ // 위의 두 경우가 아닐 때, 즉, 파란공이 얼라인 되어 있을 때
												go_straight(); // 앞으로 전진한다
											}
										}
										else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]<0 && abs(nearest_b_posi[3])>theta_c){ // 가장 가까운 파란공과의 거리가 b_a보다 작고, 왼쪽에 있으면서 각도가 theta_c보다 클 때
											turn_ccw_go(); // 잎으로 전진하면서 반시계방향으로 회전한다. 회전하는 각도가 turn_ccw_go_r보다 더 크다
										}
										else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]>0 && abs(nearest_b_posi[3])>theta_c){ // 가장 가까운 파란공과의 거리가 b_a보다 작고, 오른쪽에 있으면서 각도가 theta_c보다 클 때
											turn_cw_go(); // 앞으로 전진하면서 시계방향으로 회전한다
										}
										else if(nearest_b_posi[4] > r_c){ // 위의 경우가 모두 아니면서, 가장 가까운 파란공과의 거리가 r_c보다 클 때
											go_straight(); // 앞으로 전진한다
										}
										else{ // 위의 경우가 모두 아닐 떄, 즉, 빨간공 다 피하고 얼라인 되어있는 상태로 파란공 앞 거리 r_c까지 왔을 때
											dup_go_straight(); // case3로 가게 되면 카메라2를 쓰게 되고 카메라2가 아래쪽에 달려 있어서 파란 공이 2개가 붙어있으면 1개의 큰 공으로 판단할 수 있기에 그것을 막기 위해 함수를 실행한다
											phase = 3; // phase를 3으로 설정하여 case3로 넘어가게 한다
										}
									}
		            }
							}
	            else{ // 카메라1에서 빨간공이 보이지만 가장가까운 파란공이 가장 가까운 빨간공보다 더 가까이 있을 경우. 즉, 빨간 공이 멀리있어서 목표가 되는 파란공까지 가는 직선거리에 빨간 공이 없을 때
							  // 이 else 안의 코드는 위의 else 안의 코드와 같다. 즉, (빨간공을 피했거나) (빨간공이 파란공까지 가는 직선경로 안에 없을) 경우 같은 알고리즘을 쓴다
								if(nearest_b_posi[4] > b_aa){
									go_straight();
								}
								else if(nearest_b_posi[4] > b_a){
									if(nearest_b_posi[0] < 0 && abs(nearest_b_posi[3])>theta_c_i){
										turn_ccw_go_r();
									}
									else if (nearest_b_posi[0] > 0 && abs(nearest_b_posi[3])>theta_c_i) {
										turn_cw_go_r();
									}
									else{
										go_straight();
									}
								}
								else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]<0 && abs(nearest_b_posi[3])>theta_c){
									turn_ccw_go();
								}
								else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]>0 && abs(nearest_b_posi[3])>theta_c){
									turn_cw_go();
								}
								else if(nearest_b_posi[4] > r_c){
									go_straight();
								}
								else{
									dup_go_straight();
									phase = 3;
								}
								// 즉, 여기까지 코드가 같다
	            }
	          }

	          else{ // 카메라1에 빨간공이 없는 경우. 즉, 빨간공이 카메라1에 안보이니까 파란공까지 가는 직선거리에 빨간공이 없는 경우
							// 이 else 안의 코드 또한 위의 else 안의 코드와 같다. 즉, (빨간공을 피했거나) (빨간공이 파란공까지 가는 직선경로 안에 없을) 경우 같은 알고리즘을 쓴다
							if(nearest_b_posi[4] > b_aa){
								go_straight();
							}
							else if(nearest_b_posi[4] > b_a){
								if(nearest_b_posi[0] < 0 && abs(nearest_b_posi[3])>theta_c_i){
									turn_ccw_go_r();
								}
								else if (nearest_b_posi[0] > 0 && abs(nearest_b_posi[3])>theta_c_i) {
									turn_cw_go_r();
								}
								else{
									go_straight();
								}
							}
							else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]<0 && abs(nearest_b_posi[3])>theta_c){
								turn_ccw_go();
							}
							else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]>0 && abs(nearest_b_posi[3])>theta_c){
								turn_cw_go();
							}
							else if(nearest_b_posi[4] > r_c){
								go_straight();
							}
							else{
								dup_go_straight();
								phase = 3;
							}
							// 즉, 여기까지 코드가 같다
	        }
				}
				else{ // 가장 오른쪽 공까지 가는 경로에 파란 공이 없을 경우. 즉, 파란공의 개수가 0이 아닐 때 가장 오른쪽 공까지 가는 경로에 다른 파란공이 존재하면 이 전의 else if에서 목표 공이 그 경로 안에 들어있는 다른 공이 되고, 경로에 다른 공이 존재하지 않으면 여기 else에서 목표 공이 그대로 가장 오른쪽 파란공이 된다
					// 이 else 안의 코드는 바로 전의 else if 코드와 같다. 다만 목표 파란공이 여기서는 가장 오른쪽 파란공이기 때문에, 모든 nearest_b_posi를 rightest_b_posi로 바꿔준다
					if(!nearest_r_posi.empty()){
  						if(nearest_r_posi[4]<rightest_b_posi[4]){
								if(nearest_r_posi[4]>l_c){
									if(rightest_b_posi[0]<0 && abs(rightest_b_posi[3])>theta_c){
										turn_ccw_go();
									}
									else if(rightest_b_posi[0]>0 && abs(rightest_b_posi[3])>theta_c){
										turn_cw_go();
									}
									else{
										go_straight();
									}
								}
								else{
  							theta_car1 = RAD2DEG(atan(car_half_width1/nearest_r_posi[2]));
								theta_car2 = RAD2DEG(atan(car_half_width2/nearest_r_posi[2]));

	  							if(abs(theta_car2 + theta_avoid)>abs(nearest_r_posi[3]) && nearest_r_posi[3]>0){
										go_left();
	  								m = 1;
	  							}
	  							else if(abs(theta_car1 + theta_avoid)>abs(nearest_r_posi[3]) && nearest_r_posi[3]<0){
										go_right();
	  								m = 0;
	  							}
									else{
										if(rightest_b_posi[4] > b_aa){
											go_straight();
										}
										else if(rightest_b_posi[4] > b_a){
											if(rightest_b_posi[0] < 0 && abs(rightest_b_posi[3])>theta_c_i){
												turn_ccw_go_r();
											}
											else if (rightest_b_posi[0] > 0 && abs(rightest_b_posi[3])>theta_c_i){
												turn_cw_go_r();
											}
											else{
												go_straight();
											}
										}
										else if(rightest_b_posi[4] < b_a && rightest_b_posi[0]<0 && abs(rightest_b_posi[3])>theta_c){
											turn_ccw_go();
										}
										else if(rightest_b_posi[4] < b_a && rightest_b_posi[0]>0 && abs(rightest_b_posi[3])>theta_c){
											turn_cw_go();
										}
										else if(rightest_b_posi[4] > r_c){
											go_straight();
										}
										else{
											u = 1;
											dup_go_straight();
											phase = 3;
										}
									}
								}
							}

  						else{
								if(rightest_b_posi[4] > b_aa){
									go_straight();
								}
								else if(rightest_b_posi[4] > b_a){
									if(rightest_b_posi[0] < 0 && abs(rightest_b_posi[3])>theta_c_i){
										turn_ccw_go_r();
									}
									else if (rightest_b_posi[0] > 0 && abs(rightest_b_posi[3])>theta_c_i){
										turn_cw_go_r();
									}
									else{
										go_straight();
									}
								}
								else if(rightest_b_posi[4] < b_a && rightest_b_posi[0]<0 && abs(rightest_b_posi[3])>theta_c){
									turn_ccw_go();
								}
								else if(rightest_b_posi[4] < b_a && rightest_b_posi[0]>0 && abs(rightest_b_posi[3])>theta_c){
									turn_cw_go();
								}
								else if(rightest_b_posi[4] > r_c){
									go_straight();
								}
								else{
									u = 1;
									dup_go_straight();
									phase = 3;
								}
  						}
          }

					else{
						if(rightest_b_posi[4] > b_aa){
							go_straight();
						}
						else if(rightest_b_posi[4] > b_a){
							if(rightest_b_posi[0] < 0 && abs(rightest_b_posi[3])>theta_c_i){
								turn_ccw_go_r();
							}
							else if (rightest_b_posi[0] > 0 && abs(rightest_b_posi[3])>theta_c_i){
								turn_cw_go_r();
							}
							else{
								go_straight();
							}
						}
						else if(rightest_b_posi[4] < b_a && rightest_b_posi[0]<0 && abs(rightest_b_posi[3])>theta_c){
							turn_ccw_go();
						}
						else if(rightest_b_posi[4] < b_a && rightest_b_posi[0]>0 && abs(rightest_b_posi[3])>theta_c){
							turn_cw_go();
						}
						else if(rightest_b_posi[4] > r_c){
							go_straight();
						}
						else{
							u = 1;
							dup_go_straight();
							phase = 3;
						}
  				}
					// else 코드의 마지막으로 여기까지 코드가 전의 else if 코드와 똑같다.  다만 목표 파란공이 여기서는 가장 오른쪽 파란공이기 때문에, 모든 nearest_b_posi를 rightest_b_posi로 바꿔줬다
				}
					break; // break하여 swith문을 빠져나온다



          case 3: // picking ball 단계이다. 즉, case 1,2를 통해 파란공 앞까지 왔고 이 단계에서는 밑에 달린 카메라2를 통해 공을 탐지한다
					if(b_ball_number2 == 0){ // 카메라2에서 파란공이 보이지 않을 때
						j = 1; // 밑의 open loop을 돌리기 위해 j값 초기화
						while(j<15){ // 일정시간 동안 차를 뒤로 가게 만든다. 즉, case1로 보내도 카메라1에서 파란공이 안보일 수가 있기에 그 전에 차를 뒤로 빼서 파란공이 보이게 만든 뒤 case1로 보내게 한다. 여기서 j<15는 실험으로 정한 값이다
							go_back(); // 차를 뒤로 가게 하는 함수 실행
							write(c_socket, data, sizeof(data)); // 랩뷰로 통신할 데이터를 write한다
							j++; // j=j+1
							ros::Duration(0.02).sleep(); //duration을 0.02로 설정하여 일정한 시간동안 while문이 돌아가게 만든다
						}
						phase = 1; // phase를 1로 설정하여 case1으로 보낸다
					}
					else if(abs(nearest_b_posi2[3]) > theta_c_ii && nearest_b_posi2[0] > 0){ // 카메라2에서 파란공과의 각도가 theta_c_ii보다 크고 파란공의 위치가 왼쪽에 있을 때. 카메라2가 뒤집어져 있기에 여기서 x좌표가 +면 왼쪽에 있는 것이다
						turn_ccw_so_slowly(); // 천천히 반시계방향으로 회전
					}
					else if(abs(nearest_b_posi2[3]) > theta_c_iii && nearest_b_posi2[0] < 0){ // 카메라2에서 파란공과의 각도가 theta_c_ii보다 크고 파란공의 위치가 오른쪽에 있을 때. 카메라2가 뒤집어져 있기에 여기서 x좌표가 -면 오른쪽에 있는 것이다
						turn_cw_so_slowly(); // 천천히 시계방향으로 회전
					}
					else if(nearest_b_posi2[4]>p_c){ // 위의 세 경우가 아닌 경우, 즉, 얼라인 되고 파란공과의 거리가 p_c보다 클 때
						go_straight_slowly(); // 앞으로 천천히 전진
          }
					else if(nearest_b_posi2[4]<0.4){ // 갑자기 노이즈로 인해 파란공과의 거리가 0이 뜨는 경우가 있어서 그 경우 픽업이 실행되지 않게하기 위해 거리가 40cm 미만이면 계속 앞으로 전진하게 한다
						go_straight_slowly(); // 앞으로 천천히 전진
					}
          else{ // 얼라인 된 상태에서 파란공과의 거리가 p_c가 되었을 때
						int i =1; // 밑의 open loop을 실행하기 위해 i값을 1로 선언한다
            while(i<45){ // 일정시간 동안 blade를 돌려서 공을 픽업하게 한다. 여기서 i<45는 실험으로 정한 값이다
							pick_up(); // blade를 돌리는 픽업함수 실행
							write(c_socket, data, sizeof(data)); // 랩뷰로 통신할 데이터를 write한다
							i++; // i=i+1
							ros::Duration(0.02).sleep(); // duration을 0.02로 설정하여 일정한 시간동안 while문이 돌아가게 만든다
            }
            phase = 1; // phase를 1로 초기화 한다
						m = 0; // m을 0으로 초기화 한다
            pick_ball_num++; // 픽업한 공의 개수를 하나 추가한다.
						dataInit(); // data 값을 초기화하여 랩뷰로 통신하는 data를 초기값으로 돌려놓는다
          }
            break; // break하여 swith문을 빠져나온다
			}
    }

    else if(pick_ball_num<3){ // 픽업한 공의 개수가 1~2개 일 때 실행. 픽업한 공의 개수가 0일 떄랑 경우를 나눈 이유는, 픽업한 공의 개수가 0개일 떄는 기본적으로 목표 파란공이 가장 오른쪽에 있는 공이기 떄문이다
			// 이 else if 안의 코드는 if (pick_ball_num == 0) 안의 코드와 거의 일치하다. 즉, case 2안의 else if(경로 안에 파란공 있을 때) 안의 코드는 삭제되고 모든 rightest_b_posi가  nearest_b_posi로 바뀌어서 작성되었다. 많은 코드가 반복되니 주석은 또 달지 않겠다
			switch(phase){
        case 1:
          if(b_ball_number == 0){
            if(m == 0){
							turn_ccw_little_fast();
            }
            else{
							turn_ccw_little_fast();
            }
          }

          else{
            if(abs(nearest_b_posi[3]) > theta_c){
              if(nearest_b_posi[3] > 0){
								turn_cw();
              }
              else{
								turn_ccw();
              }
            }
            else{
              phase = 2;
            }
          }
          break;

        case 2:
				if(b_ball_number == 0){
					phase = 1;
				}
				else{
					if(!nearest_r_posi.empty()){
            if(nearest_r_posi[4]<nearest_b_posi[4]){
              theta_car1 = RAD2DEG(atan(car_half_width1/nearest_r_posi[2]));
							theta_car2 = RAD2DEG(atan(car_half_width2/nearest_r_posi[2]));

              if(abs(theta_car2 + theta_avoid)>abs(nearest_r_posi[3]) && nearest_r_posi[3]>0){
								go_left();
                m = 1;
              }
              else if(abs(theta_car1 + theta_avoid)>abs(nearest_r_posi[3]) && nearest_r_posi[3]<0){
								go_right();
                m = 0;
              }
							else{
								if(rightest_b_posi[4] > b_aa){
									go_straight();
								}
								else if(nearest_b_posi[4] > b_a){
									if(rightest_b_posi[0] < 0 && abs(rightest_b_posi[3])>theta_c_i){
										turn_ccw_go_r();
									}
									else if (rightest_b_posi[0] > 0 && abs(rightest_b_posi[3])>theta_c_i){
										turn_cw_go_r();
									}
									else{
										go_straight();
									}
								}
								else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]<0 && abs(nearest_b_posi[3])>theta_c){
									turn_ccw_go();
								}
								else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]>0 && abs(nearest_b_posi[3])>theta_c){
									turn_cw_go();
								}
								else if(nearest_b_posi[4] > r_c){
									go_straight();
								}
								else{
									dup_go_straight();
									phase = 3;
								}
							}
            }
            else{
							if(rightest_b_posi[4] > b_aa){
								go_straight();
							}
							else if(nearest_b_posi[4] > b_a){
								if(rightest_b_posi[0] < 0 && abs(rightest_b_posi[3])>theta_c_i){
									turn_ccw_go_r();
								}
								else if (rightest_b_posi[0] > 0 && abs(rightest_b_posi[3])>theta_c_i){
									turn_cw_go_r();
								}
								else{
									go_straight();
								}
							}
							else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]<0 && abs(nearest_b_posi[3])>theta_c){
								turn_ccw_go();
							}
							else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]>0 && abs(nearest_b_posi[3])>theta_c){
								turn_cw_go();
							}
							else if(nearest_b_posi[4] > r_c){
								go_straight();
							}
							else{
								dup_go_straight();
								phase = 3;
							}
            }
          }

          else{
						if(rightest_b_posi[4] > b_aa){
							go_straight();
						}
						else if(nearest_b_posi[4] > b_a){
							if(rightest_b_posi[0] < 0 && abs(rightest_b_posi[3])>theta_c_i){
								turn_ccw_go_r();
							}
							else if (rightest_b_posi[0] > 0 && abs(rightest_b_posi[3])>theta_c_i){
								turn_cw_go_r();
							}
							else{
								go_straight();
							}
						}
						else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]<0 && abs(nearest_b_posi[3])>theta_c){
							turn_ccw_go();
						}
						else if(nearest_b_posi[4] < b_a && nearest_b_posi[0]>0 && abs(nearest_b_posi[3])>theta_c){
							turn_cw_go();
						}
						else if(nearest_b_posi[4] > r_c){
							go_straight();
						}
						else{
							dup_go_straight();
							phase = 3;
						}
        }
			}
          break;



        case 3:
				if(b_ball_number2 == 0){
					j = 1;
					while(j<15){
						go_back();
						write(c_socket, data, sizeof(data));
						j++;
						ros::Duration(0.02).sleep();
					}
					phase = 1;
				}
				else if(abs(nearest_b_posi2[3]) > theta_c_ii && nearest_b_posi2[0] > 0){
					turn_ccw_so_slowly();
				}
				else if(abs(nearest_b_posi2[3]) > theta_c_iii && nearest_b_posi2[0] < 0){
					turn_cw_so_slowly();
				}
				else if(nearest_b_posi2[4]>p_c || nearest_b_posi2[4] < 0.4){
					go_straight_slowly();
        }
        else{
					int i =1;
					while(i<45){
						pick_up();
						write(c_socket, data, sizeof(data));
						i++;
						ros::Duration(0.02).sleep();
          }
          pick_ball_num++;
          phase = 1;
					m = 0;
					dataInit();
        }
          break;
      }
    }

		else{ // 픽업한 공의 개수가 3개 일 때 실행. 즉, 공을 3개 다 주웠을 경우 relase 모드 실행
		  switch(phase){ // phase를 전 단계에서 1로 초기화 해줘서 case1이 실행됨
		    case 1: // case1 살행. 이 단계에서는 초록공 2개를 탐지하고 g_dis 거리까지 간 뒤에 z와 x틀 얼라인 하여 릴리즈 할 준비를 한다
		      if(g_ball_number < 2){ // 카메라1에 보이는 초록공의 개수가 2개 미만일 경우
						if (p == 0){ // p의 값이 0일 때 실행. p의 초기값은 0으로 설정되어 있다. 즉, 처음으로 case1에 들어왔을 경우 실행된다
							turn_ccw_fast(); // 빠르게 반시계방향으로 회전
						}
						else if (p == 1){ // p의 값이 1일 때 실행. 즉, 밑의 else에서 얼라인하다가 카메라에 공이 2개가 안보이면 다시 여기가 실행되는데 시계방향으로 돌아야 할 경우 p가 1로 설정되어 여기가 실행된다
							turn_cw_go(); // 앞으로 전진하면서 시계방향으로 회전
						}
						else{ // p의 값이 2일 때 실행. 즉, 밑의 else에서 얼라인하다가 카메라에 공이 2개가 안보이면 다시 여기가 실행되는데 반시계방향으로 돌아야 할 경우 p가 2로 설정되어 여기가 실행된다
							turn_ccw_go(); // 앞으로 전진하면서 반시계방향으로 회전
						}

		      }

		      else{ // 카메라1에 보이는 초록공의 개수가 2개일 경우
						if(g_mid[1] > g_dis){ // 초록공 2개 사이의 중심과의 거리가 g_dis 이상일 경우
							go_straight(); // 앞으로 전진
						}
						else if(g_mid[2] > z_diff){ // 오른쪽 초록공 z좌표에서 왼쪽 초록공 z좌표를 뺸 값이 양수일 경우, 즉, 차를 반시계방향으로 회전시켜야 수평이 맞아지는 경우
							turn_ccw_go(); // 앞으로 전진하면서 반시계방향으로 회전
							p = 1; // p를 1로 설정. 따라서 반시계방향으로 회전하다가 공을 하나 일어버리면 전 if문으로 가서 시계방향으로 회전하여 공을 찾게끔 만들어준다.
						}
						else if(g_mid[2] < -z_diff){ // 오른쪽 초록공 z좌표에서 왼쪽 초록공 z좌표를 뺸 값이 음수일 경우, 즉, 차를 시계방향으로 회전시켜야 수평이 맞아지는 경우
							turn_cw_go(); // 앞으로 전진하면서 시계방향으로 회전
							p = 2; // p를 2로 설정. 따라서 시계방향으로 회전하다가 공을 하나 일어버리면 전 if문으로 가서 반시계방향으로 회전하여 공을 찾게끔 만들어준다.
						}
						else if(rightest_g_posi[0]> g_x){ // 가장오른쪽에 있는 초록공의 x좌표가 g_x보다 큰 경우. 즉 이 경우는 가장 오른쪽에 있는 초록공이 카메라1의 중심보다 오른쪽에 있기에 시계방향으로 회전시켜야 한다. 가장 오른쪽에 있는 공으로 얼라인 하는 이유는 릴리즈하는 부분이 차체의 왼쪽에 있기에 카메라 중심을 오른쪽 공으로 얼라인해야 바구니 가운데 쪽에 릴리즈 하기 떄문이다
							turn_cw_go(); // 앞으로 전진하면서 시계방향으로 회전
							p = 2; // p를 2로 설정. 따라서 시계방향으로 회전하다가 공을 하나 일어버리면 전 if문으로 가서 반시계방향으로 회전하여 공을 찾게끔 만들어준다.
						}
						else if(rightest_g_posi[0]< -g_x){ // 가장오른쪽에 있는 초록공의 x좌표가 -g_x보다 작은 경우. 즉 이 경우는 가장 오른쪽에 있는 초록공이 카메라1의 중심보다 왼쪽에 있기에 반시계방향으로 회전시켜야 한다.
							turn_ccw_go(); // 앞으로 전진하면서 반시계방향으로 회전
							p = 1; // p를 1로 설정. 따라서 반시계방향으로 회전하다가 공을 하나 일어버리면 전 if문으로 가서 시계방향으로 회전하여 공을 찾게끔 만들어준다.
						}
						else{ // 초록공과의 거리가 g_dis 이내에서 x,z가 얼라인 되었을 경우
							phase = 2; // phase를 2로 설정하여 case2를 실행하게 한다
							p = 0; // p의 값을 0으로 초기화한다
						}
					}
					break; // break하여 switch문을 빠져나온다

				case 2: //case2 실행. 이 단계에서는 초록공과의 거리가 g_c가 될 때까지 얼라인을 유지하면서 앞으로 전진한다
					if(g_ball_number == 0){ // 카메라1에 보이는 초록공의 개수가 0일 경우
						phase = 1; // phase를 1로 설정하여 다시 case1이 실행되게끔 한다
					}
					else{ // 카메라1에 보이는 초록공의 개수가 0이 아닐 경우. 즉, case1에서 오른쪽 초록공으로 얼라인을 했기에 앞으로 가다가 카메라1에서 왼쪽 초록공이 안보이더라도 이제는 신경쓰지 않고 그대로 전진한다
							if(rightest_g_posi[0]<0 && abs(rightest_g_posi[3])>theta_c){ // 가장 오른쪽에 있는 초록공이 차체 중심보다 왼쪽에 있고 각도가 theta_c보다 클 때
								turn_ccw_go(); // 앞으로 전진하면서 반시계방향으로 회전
							}
							else if(rightest_g_posi[0]>0 && abs(rightest_g_posi[3])>theta_c+1){ // 가장 오른쪽에 있는 초록공이 차체 중심보다 오른쪽에 있고 각도가 theta_c보다 클 때
								turn_cw_go(); // 앞으로 전진하면서 시계방향으로 회전
							}
							else if(rightest_g_posi[4] > g_c){ // 위의 두 경우가 아니면서, 즉, 얼라인 된 상태에서 가장 오른쪽 초록공과의 거리가 g_c보다 클 때
								go_straight(); // 앞으로 전진
							}
							else{ // 얼라인 된 상태에서 가장 오른쪽 초록공과의 거리가 g_c보다 작을 때
								phase = 3; // phase를 3으로 설정하여 case3를 실행시킨다
							}
					}

					break; // break하여 switch문을 빠져나온다



		    case 3: // 이 단계에서는 카메라2를 사용하여 초록공을 탐지한다. 공을 얼라인하면서 전진하고 일정거리가 되면 공을 끝까지보고 릴리즈를 한다
		        if(g_ball_number2 == 0){ // 카메라2에서 보이는 초록공의 개수가 0일 때 실행
							if (g_ball_number == 0){ // 카메라1에서 보이는 초록공의 개수가 0일 때 실행. 즉, 카메라1,2 모두에서 초록공이 안보이면 차체를 뒤로 가게 하여 카메라1에서 탐지되게끔 한다
									go_back(); // 차를 뒤로 가게 하는 함수 실행
								}
							else{ // 카메라1에서 초록공의 개수가 0이 아닐 때 실행
								phase = 1; // phase를 1로 설정하여 다시 case1으로 돌아가게 만든다. 카메라1에서 공이 탐지될 때 까지 차를 뒤로 움직였기 떄문에 case1의 카메라1에서 초록공이 안보일 위험이 없다
							}
		        }
		        else if(rightest_g_posi2[0] < -0.1){ // 초록공의 x좌표가 -10cm보다 작을 때 실행. 카메라2는 뒤집혀 있기 떄문에 x좌표가 음수이면 카메라2보다 오른쪽에 공이 있는 것이다
		          turn_cw_so_slowly(); // 시계방향으로 매우 천천히 회전
		        }
		        else if(rightest_g_posi2[0] > 0.1){ // 초록공의 x좌표가 10cm보다 클 때 실행. 카메라2는 뒤집혀 있기 떄문에 x좌표가 양수이면 카메라2보다 왼쪽에 공이 있는 것이다
		          turn_ccw_so_slowly(); // 반시계방향으로 매우 천천히 회전
		        }
		        else if(rightest_g_posi2[4]>p_c_g){ // 얼라인 된 상태로 초록공과의 거리가 p_c_g 이상일 경우 실행
		          go_straight_slowly(); // 천천히 앞으로 전진
		        }

		        else{ // 얼라인 된 상태로 초록공과의 거리가 p_c_g 이하일 경우 실행. 여기서는 blade를 들어올려 공을 릴리즈 한다

		            int i = 1; // 밑의 open loop을 실행하기 위해 i값을 1로 선언한다
		            while(i<60){ // 일정시간 동안 blade를 들어올려서 공이 릴리즈 되게 한다. 여기서 i<60은 실험으로 정한 값이다. 이때 동시에 바퀴는 앞으로 천천히 전진한다.
		              release1(); // blade 들어올리는 릴리즈1 함수 실행. 릴리즈1 함수에서는 공을 안정적으로 떨어뜨리기 위해 바퀴를 천천히 앞으로 전진한다
		              write(c_socket, data, sizeof(data)); // 랩뷰로 통신할 데이터를 write한다
		              i++; // i=i+1
		              ros::Duration(0.02).sleep(); // duration을 0.02로 설정하여 일정한 시간동안 while문이 돌아가게 만든다
		            }
								i = 1; // 밑의 open loop을 실행하기 위해 i값을 1로 초기화한다
		            while(i<100){ // 일정시간 동안 blade를 들어올려서 공이 릴리즈 되게 한다. 여기서 i<100은 실험으로 정한 값이다. 이때 동시에 바퀴는 뒤로 천천히 후진한다
		              release2(); // blade 들어올리는 릴리즈2 함수 실행. 릴리즈2 함수에서는 공이 바구니 안에 낄 수도 있기 때문에 바퀴를 천천히 뒤로 후진한다
		              write(c_socket, data, sizeof(data)); // 랩뷰로 통신할 데이터를 write한다
		              i++; // i=i+1
		              ros::Duration(0.02).sleep(); // duration을 0.02로 설정하여 일정한 시간동안 while문이 돌아가게 만든다
		            }
		            dataInit(); // data 값을 초기화하여 랩뷰로 통신하는 data를 초기값으로 돌려놓는다
		            data[14] = 1; // data[14]값을 1로 설정하여 랩뷰를 종료하는 데이터를 설정한다
		            write(c_socket, data, sizeof(data)); // 랩뷰를 종료하는 데이터를 write한다
		            ros::shutdown(); // shutdown하여 ros를 shutdouwn한다
		        }
		      break; // break하여 swith문을 빠져나온다
		  }
		}

		write(c_socket, data, sizeof(data)); // 랩뷰로 통신할 데이터를 write한다
		loop_rate.sleep(); // 위에서 정한 loop rate를 타임슬립에 집어넣는다

}


    return 0; // return 0을 하여 메인 함수를 종료한다
}
