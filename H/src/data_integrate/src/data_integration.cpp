#include <iostream>	//헤더파일 선언.
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
#include <vector>
#include <cmath> // std::abs
#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress

using namespace std;

int ball_number=0;					// 웹캠에 보이는 공 개수 변수.

//Ball detect node에서 가져온 공 정보들을 새로 담을 변수 선언. 최대 각 3개씩이니 사이즈는 넉넉하게 20으로 먼저 잡았다.
vector<float>ball_rX(20);								//vector 형태로 선언하고, 빨간공들의 X좌표 정보의 벡터 데이터 선언.
vector<float>ball_rY(20);								//빨간공들의 Y좌표 정보의 벡터 데이터 선언.
vector<float>ball_bX(20);								//파란공들의 X좌표 정보의 벡터 데이터 선언.
vector<float>ball_bY(20);								//파란공들의 Y좌표 정보의 벡터 데이터 선언.
vector<float>ball_gX(20);								//초록공들의 X좌표 정보의 벡터 데이터 선언.
vector<float>ball_gY(20);								//초록공들의 Y좌표 정보의 벡터 데이터 선언.
vector<float>basket_X(20);								//바구니의 X좌표 정보의 벡터 데이터 선언.
vector<float>basket_Y(20);								//바구니의 Y좌표 정보의 벡터 데이터 선언.
vector<float>red_ball_distance(20);				//빨간공들까지의 거리 정보의 벡터 데이터 선언.
vector<float> blue_ball_distance(20);			//파란공들까지의 거리 정보의 벡터 데이터 선언.
vector<float> green_ball_distance(20);		//초록공들까지의 거리 정보의 벡터 데이터 선언.
vector<float> basket_distance(20);				//바구니까지의 거리 정보의 벡터 데이터 선언.

//모터에 입력할 각속도 값을 전달하는 write 함수를 구성하는 원소 선언.
int c_socket, s_socket;
struct sockaddr_in c_addr;

//53줄 ~ 83줄 : 뒤에 사용하는 변수들을 선언한다.
float data[24];				//myRIO를 통해 Labview로 전달할 변수들 data를 선언.
int state=0;					//state를 구분하는 변수 선언.
int pickup_state=0;		//pickup state를 구분하는 변수 선언.
int anti_error=0;			//뒤에 나올 avoid redball시 발생 가능한 coredumped error를 막기 위해 사용하는 변수.
float theta_release=0;	//Release로 넘어가는 trigger로, 파란공을 찾을때 까지 회전한 각도값이다. 360도 가량 회전해도 못찾을 경우 release로 넘어간다.
int num_pickup=0;				//pickup한 횟수를 저장하는 변수.

//차량 스펙과 관련된 변수 선언.
float margin=17;				//함수 avoid redball의 trigger로서, 빨간공을 피할때 차와 빨간공 사이의 거리 margin값, cm단위.
float margin_b = 30;		//파란공 타겟 선정 방법에 있어서 타겟이 아닌 파란공을 부딪히지 않기 위한 margin값, setting a target 함수에서 사용된다.
float r0=20; 						//car diagonal spec으로 circular move 함수를 사용할 때 사용된다.

//Openloop시 사용하는 시간변수 선언.
int parking_time=0;			//마지막 parking과 releasing시 openloop 구간에서 사용되는 시간변수.
int time_last=0;				//parking시 안전한 제어를 위해 feedforward로 잠깐동안 멈춰 관성을 제거할 때 쓰는 시간변수.

//pickup_time설정(position control을 하지만 한 루프만에 제어가 되지 않기 때문에 넣어주는 margin 시간)
int t_p1=20;				//blade 위상별 margin 시간. 구간 (close to open)
int t_d1=10;				//blade 위상별 margin 시간. 구간 (open to close)
int t_d2=35;				//blade 위상별 margin 시간. 구간 (push back ball to storage)
int t_d3=40;				//blade 위상별 margin 시간. 구간 (ready to next pickup)
int state2_time=0;	//pickup 할때의 state2에서 사용하는 시간변수 선언.
int pickup_time=0;	//pickup 할때의 pickup state1에서 사용하는 시간변수 선언.

//PD control에 사용할 gain값 변수 선언.
float k_p=7;			//P gain값(가까울 때)
float k_p2=7;			//P gain값(멀 때)
float k_d=3.5;			//D gain값
float kc=5;				//원운동 할때의 회전보정 P gain

double theta_pd0=0; // PDcontrol시 D error를 위해 저장하는 이전 loop 정보 변수.

//86줄 ~ 294줄 코드에서 사용하는 함수들을 선언.
void dataInit()			//myRIO에 보내주는 data 변수들 초기화 함수 선언.

{
	data[0] = 0;
	data[1] = 0; //wheel 1 각속도
	data[2] = 0; // wheel 2 각속도
	data[3] = 0; // wheel 3 각속도
	data[4] = 0; //wheel 4 각속도
	data[5] = 0; //pick-up motor 각속도
	data[6] = 310; //pick-up motor 위치제어 각도
	data[7] = 1;   //pick-up motor 속도제어, 각도제어 trigger (1:position 0:velocity)
	data[8] = 30;  //모터 maximum acceleration값
	data[9] = 0;
	data[10] = 0;
	data[11] = 0;
	data[12] = 0;
	data[13] = 0;
	data[14] = 0;
	data[15] = 0;
	data[16] = 0;
	data[17] = 0;
	data[18] = 0;
	data[19] = 0;
	data[20] = 0;
	data[21] = 0;
	data[22] = 0;
	data[23] = 0;
}
//115줄 ~ 218줄 : 차량 움직임과 관련된 함수. 자주 사용하는 움직임으로 함수화 하였다.
//pickup motor 회전하는 함수. 속도제어
int pick_up(float vel_p) {
	data[5] = vel_p;
	return 0;
}

//직진운동 함수.
int movef(float ang_vel){
	data[1] = ang_vel;
	data[2] = ang_vel;
	data[3] = ang_vel;
	data[4] = ang_vel;
	cout<<"moving"<<endl;
	return 0;
}

//회전운동 함수.
int align(float ang_vel){
	data[1] = ang_vel;
	data[2] = -ang_vel;
	data[3] = ang_vel;
	data[4] = -ang_vel;
	cout<<"aligning"<<endl;
	return 0;
}

//좌회전 함수.
int move_left(float ang_vel){
	data[1] = ang_vel;
	data[2] = ang_vel;
	data[3] = -ang_vel;
	data[4] = -ang_vel;
	cout<<"move left"<<endl;
	return 0;
}

//우회전 함수.
int move_right(float ang_vel){
	data[1] = -ang_vel;
	data[2] = -ang_vel;
	data[3] = ang_vel;
	data[4] = ang_vel;
	cout<<"move right"<<endl;
	return 0;
}

//원운동 함수. 원의 중심을 항상 바라보는 공전운동이다.
//r은 원의 중심까지의 거리(주로 파란공이나 바구니까지 거리가 된다)
//v는 원운동시의 선속도이다.
//t는 원운동이 실제로 구현하는데에 생기는 오차를 줄이기 위해 만든 에러 변수로, 차의 중심에서부터 원의 중심이 벗어난 각도 오차이다.
void circular_move(float r,float v, float t){
	data[1]= -v+v*r0/r+kc*t;
	data[2]= v-v*r0/r-kc*t;
	data[3]= v+v*r0/r+kc*t;
	data[4]= -v-v*r0/r-kc*t;
}

//PD control하는 함수 선언이다.
//x, y값은 타겟 공과의 x거리 오차, y 거리 오차로 공의 x좌표와 y 좌표가 된다.
//m, n은 주행의 효율을 위해 둔 PD control의 작동 오차범위 threshold로 오차가 이보다 작을경우 빠르게 직진한다.
int PD_control(float x, float y, float m, float n){
	double theta_pd=atan(x/y);		//거리 좌표를 이용해 angle 변수 생성
	if(sqrt(x*x+y*y)>65){					//거리에 따라서 제어의 threshold를 다르게 하였다.(시간 효율성 때문)
		if(x>m){
			data[1]=50+theta_pd*k_p-(theta_pd0-theta_pd)*k_d;
			data[2]=50-theta_pd*k_p+(theta_pd0-theta_pd)*k_d;
			data[3]=50+theta_pd*k_p-(theta_pd0-theta_pd)*k_d;
			data[4]=50-theta_pd*k_p+(theta_pd0-theta_pd)*k_d;
		}
		else if(x<-1*m){
			data[1]=50+theta_pd*k_p-(theta_pd0-theta_pd)*k_d;
			data[2]=50-theta_pd*k_p+(theta_pd0-theta_pd)*k_d;
			data[3]=50+theta_pd*k_p-(theta_pd0-theta_pd)*k_d;
			data[4]=50-theta_pd*k_p+(theta_pd0-theta_pd)*k_d;
		}
		else{
			data[1]=67;
			data[2]=67;
			data[3]=67;
			data[4]=67;
		}
	}else{
		if(x>n){
			data[1]=20+theta_pd*k_p2-(theta_pd0-theta_pd)*k_d;
			data[2]=20-theta_pd*k_p2+(theta_pd0-theta_pd)*k_d;
			data[3]=20+theta_pd*k_p2-(theta_pd0-theta_pd)*k_d;
			data[4]=20-theta_pd*k_p2+(theta_pd0-theta_pd)*k_d;
		}
		else if(x<-1*n){
			data[1]=20+theta_pd*k_p2-(theta_pd0-theta_pd)*k_d;
			data[2]=20-theta_pd*k_p2+(theta_pd0-theta_pd)*k_d;
			data[3]=20+theta_pd*k_p2-(theta_pd0-theta_pd)*k_d;
			data[4]=20-theta_pd*k_p2+(theta_pd0-theta_pd)*k_d;
		}
		else{
			data[1]=67;
			data[2]=67;
			data[3]=67;
			data[4]=67;
		}
	}
	double theta_pd0=theta_pd;	//D control를 위한 이전 루프 오차변수 저장.
	cout<<theta_pd<<endl;
}

//220줄 ~ 294줄 : 타겟 설정 함수들로, pickup할 파란공 타겟을 선정하거나, 피할 빨간공 타겟을 선정하거나, release할 초록공 타겟 지점을 구하는 함수.
//픽업할 파란공을 타겟으로 선정하는 함수.
//a는 시야에 보이는 파란공들의 x좌표들, b는 y좌표들이다.
//일반적으로는 가장 왼쪽에 보이는 파란공을 타겟으로 선정하는데, 더 가까이 있는 파란공과 부딪힐 것 같을때만 그 공을 타겟으로 선정하는 예외를 두었다.
int setting_a_target(vector<float> a,vector<float> b){
	float temp = a[0];
	int t=0;
	for(int i=0; i<a.size(); i++){
		if( temp > a[i]) {
			temp = a[i];								//가장 왼쪽에 있는 공을 타겟으로 선정하는 과정. t는 타겟의 공 번호로 최소 0에서 최대 2까지 있다.
			t=i;
			}
		}if(t==1){										//가장 왼쪽에 있는 파란공 앞에 파란공이 하나 존재할 경우
			float d = std::abs(a[0]*b[1] - b[0]*a[1]) / sqrtf(a[0]*a[0]+b[1]*b[1]);
			if(d<margin_b){							//타겟이 아닌 파란공과 부딪힐 것 같은 경우
				t=0;											//타겟을 그 공으로 변경한다.
				cout<<"blue ball 1,2 are too close!"<<endl;
			}else{
				t=1;
			}
		}if(t==2){										//가장 왼쪽에 있는 파란공 앞에 파란공이 두개 존재할 경우 마찬가지로 두 공들에 대해 위와같은 처리를 해준다.
			float d02=std::abs(a[0]*b[2] - b[0]*a[2]) / sqrtf(a[0]*a[0]+b[2]*b[2]);
			float d12=std::abs(a[1]*b[2] - b[1]*a[2]) / sqrtf(a[1]*a[1]+b[2]*b[2]);
			if(d02<margin_b){
				t=0;
				cout<<"blue ball 1,3 are too close!"<<endl;

			}else if(d12<margin_b){
				t=1;
				cout<<"blue ball 2,3 are too close!"<<endl;

			}else{
				t=2;
			}
		}return t;
	}

//회피할 빨간공을 타겟으로 선정하는 함수로, 타겟 파란공과의 수직거리가 가장 가까운 빨간공을 선정하는 함수이다.
float setting_a_redball_target(float bx, float by, vector<float> rx, vector<float>ry){//rx, ry : red ball's position datas; rnumber : red ball's number; bx, by : blue ball's position
	int d = 10000; // to avoid returning trash value
	int l = 0;
	for(int i = 0; i < rx.size(); i++){ //웹캠 시야의 모든 빨간공에 대해
		l = abs(bx*ry[i] - by*rx[i]) / sqrtf(bx*bx+by*by); //타겟 파란공과 빨간공의 수직거리를 계산.
		if(d > l){
			d = l; // find smallest distance
			if(sqrt(rx[i]*rx[i]+ry[i]*ry[i])>50){		//빨간공과 파란공의 수직거리가 50보다 크면, 충돌 위험 없다고 판단하고 무시.
				d=10000;
			}
		}
	}
	cout<<"d:	"<<d<<endl;
	return d;
}

//초록공들의 중심 x, y좌표를 얻어내는 함수로 후에 release를 하기위해 다가갈때 사용한다.
//초록공이 하나만 보이더라도 작동할 수 있도록 코드를 만들었다.
//a는 초록공들의 x좌표들, b는 y좌표들이다.
float setting_green_x(vector<float> a,vector<float> b){
	float t=0;
	float sum=0;
	for(int i=0; i<a.size(); i++){
		sum = sum + a[i];
	}
	t=sum/a.size();
	return t;
}
float setting_green_y(vector<float> a,vector<float> b){
	float t=0;
	float sum=0;
	for(int i=0; i<b.size(); i++){
		sum = sum + b[i];
	}
	t=sum/b.size();
	return t;
}

//Callback function으로, 웹캠 정보가 들어올때 작동한다. 본격적인 모든 platform의 알고리즘이 담겨있다.
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)//27Hz, v*t=3375=1m
{
		int count=position->img_rx.size()+ position->img_bx.size(); //웹캠에 보이는 빨간공과 파란공의 개수
		ball_number=count;
		ball_rX.resize(position->img_rx.size());										//메세지에서 받아오는 빨간공들의 x좌표를 저장하는 변수 사이즈 동기화.
		ball_rY.resize(position->img_ry.size());										//메세지에서 받아오는 빨간공들의 y좌표를 저장하는 변수 사이즈 동기화.
		ball_bX.resize(position->img_bx.size());										//메세지에서 받아오는 파란공들의 x좌표를 저장하는 변수 사이즈 동기화.
		ball_bY.resize(position->img_by.size());										//메세지에서 받아오는 파란공들의 y좌표를 저장하는 변수 사이즈 동기화.
		ball_gX.resize(position->img_gx.size());										//메세지에서 받아오는 초록공들의 x좌표를 저장하는 변수 사이즈 동기화.
		ball_gY.resize(position->img_gy.size());										//메세지에서 받아오는 초록공들의 y좌표를 저장하는 변수 사이즈 동기화.
		basket_X.resize(position->img_basket_x.size());							//메세지에서 받아오는 바구니의 x좌표를 저장하는 변수 사이즈 동기화.
		basket_Y.resize(position->img_basket_y.size());							//메세지에서 받아오는 바구니의 y좌표를 저장하는 변수 사이즈 동기화.

		red_ball_distance.resize(position->img_rx.size());					//공, 바구니까지의 거리를 저장할 변수 사이즈 동기화.
		blue_ball_distance.resize(position->img_bx.size());
		green_ball_distance.resize(position->img_gx.size());
		basket_distance.resize(position->img_basket_x.size());

		//316줄~340줄 : 메세지에서 받아오는 공, 바구니들의 좌표 정보를 새로운 array 변수에 입력하는 과정. 이 과정을 하는 이유는 후에 target을 설정하는데에 있어서
		//공들을 labelling하여 더 쉽게 코딩하기 위함이다.
		for(size_t i = 0; i < position->img_rx.size(); i++)
    {
				ball_rX[i]=position->img_rx[i] ;
				ball_rY[i]=position->img_ry[i] ;
				red_ball_distance[i] = sqrt(position->img_rx[i]*position->img_rx[i]+position->img_ry[i]*position->img_ry[i]);
    }
		for(size_t i = 0; i < position->img_bx.size(); i++)
    {
				ball_bX[i]=position->img_bx[i] ;
				ball_bY[i]=position->img_by[i] ;
				blue_ball_distance[i] = sqrt(position->img_bx[i]*position->img_bx[i]+position->img_by[i]*position->img_by[i]);
    }
		for(size_t i = 0; i < position->img_gx.size(); i++)
    {
				ball_gX[i]=position->img_gx[i] ;
				ball_gY[i]=position->img_gy[i] ;
				green_ball_distance[i] = sqrt(position->img_gx[i]*position->img_gx[i]+position->img_gy[i]*position->img_gy[i]);
    }
		for(size_t i = 0; i < position->img_basket_x.size(); i++)
    {
				basket_X[i]=position->img_basket_x[i] ;
				basket_Y[i]=position->img_basket_y[i] ;
				basket_distance[i] = sqrt(position->img_basket_x[i]*position->img_basket_x[i]+position->img_basket_y[i] *position->img_basket_y[i] );
    }

////////////////////////////

	//pickup state는 pickup시 chain과 blade 즉 pickup module을 제어하는 구간이다.
	//pickup state는 0과 1일때로 이루어져있으며, 1일때 blade를 움직이게 해 pickup을 한다.
	//이 과정은 state decision 파트인 state가 0일때 pickup을 해야겠다고 판단될경우 작동한다.
	//position control을 기반으로 해 정확한 제어가 가능하다.
	if (pickup_state==1){
		cout<<"picking up"<<endl;
		data[7]=1;
		if (pickup_time<t_p1){
			data[6]=350;
			cout<<"blade-up"<<endl;
		}
		else if (pickup_time<t_p1+t_d1){
			data[6]=0;
		}else if (pickup_time<t_p1+t_d1+1){
			data[6]=0;
			state=0;				//중요한 trigger인데, 블레이드가 닫히는 과정에서 공이 안빠져나갈 수 있는 시점에서
											//wheel mode와 decision mode를 담당하는 state를 다시 state=0으로 변경하여 블레이드는 공을 마저 storage에 넣고 바퀴는 다음공을 찾으러 간다.
			num_pickup=num_pickup+1;	//이 시점에서 총 pickup한 횟수를 누적해준다.
		}else if (pickup_time<t_p1+t_d1+t_d2){
			data[6]=0;
		}
		else if (pickup_time<t_p1+t_d1+t_d2+t_d3){
			data[6]=310;
		}
		else{
			pickup_time=0;
			pickup_state=0;
		}
		pickup_time++;

	}

///////////////////////

	//state2는 pickup시 wheel을 제어하는 구간이다.
	//이 과정은 state decision 파트인 state가 0일때 pickup을 해야겠다고 판단될경우 작동한다.
	//이 짧은 pickup하는 구간동안, 타겟 공이 너무 가까워 웹캠으로 볼 수 없어 바퀴는 멈추지 않고 feedforward로 직진한다.
	//하지만 본 platform의 pickup module은 꽤 robust해, 이정도의 openloop는 항상 잘 작동하였다.
 	if(state==2){

			cout<<"state2_time :"<<state2_time<<endl;
			cout<<"pickup_time :"<<pickup_time<<endl;

			cout<<"state = 2(pick up wheel mode)"<<endl;
			state2_time++;

			if(pickup_time - state2_time >7){		//파란공을 연달아 바로 pickup해야할 때 생길 수 있는 특이 케이스 문제를 해결하기 위한 코드이다.
																					//블레이드가 다음 파란공을 pickup을 할 준비를 끝마칠 때 까지는 느리게 뒤로 가거나 align을 맞추는 등 바퀴도 준비를 한다.
				if(pickup_time<104){
					movef(-20);
					if(ball_bX[0]>9){
						align(20);
					}else if(ball_bX[0]<-9){
						align(-20);
					}

					cout<<"ready for next pick-up(too close blue balls)"<<endl;
				}else{
					state2_time=0;
					pickup_time=0;
					state=0;
				}
			}else{
				movef(60);												//특이케이스가 아닌 경우 pickup시 바퀴는 pickup state에서 state를 0으로 바꾸기 전까지 직진운동 한다.

			}
			if(state2_time>30){
				state=0;
				state2_time=0;

			}
			cout<<"pickup_time :"<<pickup_time<<endl;
			cout<<"state2_time :"<<state2_time<<endl;

	}

////////////////////////

//state0은 state decision과 PD control을 하는 구간이다.
//기본적으로는 다음 타겟 파란공을 찾고, 그 공에 PD control을 하며 다가간다.
//이 과정에서 빨간공을 피해야 한다고 판단되면 state=1로 변경해 avoid redball을 하는 state로 변경하고,
//타겟 파란공을 못찾거나 픽업을 몇번 이상 했다고 판단되면 release state로 넘어간다.
	if (state==0){
		data[8]=30;
		cout<<"state = 0(state decision & moving around)"<<endl;
		cout<<"num_pickup : "<<num_pickup<<endl;
		cout<<"pickup_time :"<<pickup_time<<endl;

		if(num_pickup>2){	//현재는 pickup을 세번 이상 했다고 판단될 때 바로 release하는 state로 보내주는 역할을 한다.
			state=3;
		}
		if(position->img_bx.size()==0){   /////blueballs on the cam
			align(40);
		}
		else{
			theta_release=0;
			int t = setting_a_target(position->img_bx, position->img_by);	//픽업할 target 파란공을 정하는 함수.
			cout<<"target ball x position : "<<ball_bX[t]<<endl;
			cout<<"target ball distance : "<<blue_ball_distance[t]<<endl;
			cout<<t+1<<"  is target blue ball"<<endl;

			if(blue_ball_distance[t]>40){	//타겟 파란공과의 거리가 40cm보다 멀때,
				cout<<"here!"<<endl;
				int d=setting_a_redball_target(ball_bX[t], ball_bY[t], position->img_rx, position->img_ry); //위험한 빨간공이 있는지 확인한다.
				if(d>margin){															//타겟 파란공과 가장 가까운 빨간공과의 x축 거리가 margin보다 크면 부딪힐 일이 없으므로
					PD_control(ball_bX[t],ball_bY[t],7,5);	//PD control로 타겟 파란공에 다가간다.
					cout<<"PD controlling"<<endl;
				}
				else{				//타겟 파란공과 가장 가까운 빨간공의 x축 거리가 margin보다 작아 부딪힐 위험이 있으므로
					state=1;	//avoid red ball 동작을 하는 state 1로 변경해준다.
				}
			}
			else{					//타겟 파란공과의 거리가 40cm보다 가까울 때,
				pickup_state=1;		//블레이드를 pickup state=1 로 하여 pickup mode로 진입하게 하고
				state=2;					//이때의 wheel mode 또한 state=2로 하여 pickup을 시작한다.
			}
		}

	}

//////////////////////////

	//state1은 target blue ball에 다가가는 과정에서 빨간공을 피해야겠다고 판단될 경우 작동한다.
	//state1은 먼저 타겟 파란공을 차 정중앙에 align을 맞추고, 이후 state6으로 넘어가 정중앙에 놓여진 타겟 파란공을 중심으로 빨간공과 멀어지는 방향으로 원운동을 한다.
	//이 과정에서 빨간공을 피하지 않아도 된다고 판단되면 다시 state0으로 돌아가 타겟 파란공을 향해 PD control을 하거나 pickup을 실행한다.
	if (state==1){
		cout<<"state = 1(Avoid red ball)"<<endl;
		int t = setting_a_target(position->img_bx, position->img_by);				//타겟 파란공 선정하고 그 공에 t라는 라벨을 붙힌다.
		int d_1=setting_a_redball_target(ball_bX[t], ball_bY[t], position->img_rx, position->img_ry);  //타겟 파란공에 제일 가까운 빨간공과 타겟 파란공의 x축 거리차이를 구한다.
		cout<<"blue ball x distance :"<<position->img_bx[t]<<endl;
		if(d_1>margin){		//그 x축 거리차이가 margin보다 커서 충돌이 일어나지 않는다고 판단되면 state0으로 돌아간다.
			state=0;
		}
		else{	//충돌이 여전히 일어날 것 같다고 생각되면, 타겟 파란공에 대한 align을 마저 하고, 이후 state=6으로 넘어간다.
			if(position->img_bx[t]>7){
				align(30);
				anti_error=1;										//이 과정에서 anti_error라는 변수를 사용했는데, 이는 state6에 들어오기 전에 차가 어떠한 방향으로 회전해서
			}																	//aling을 맞추게 되었는지 기억하게 한다. 이 작업은 state1에서 state6으로 넘어갈 때, 간혹 align을 맞추는 회전 관성이 커서 오히려 반대쪽으로
																				//넘어가 파란공을 시야에서 놓치는 일이 발생하는데 이때 coredumped error가 발생하는것을 방지하기 위해 만들었다. state6에서 이러한 전후과정을 더 잘 알 수 있을것이다.
			else if(position->img_bx[t]<-7){
				align(-30);
				anti_error=2;
			}
			else{
				state=6;
			}
		}
	}

	//////////////////////////

	//state6은 빨간공을 피해야겠다고 판단한 후, 타겟 파란공에 align을 맞춘뒤 작동한다.
	//본 state는 align을 맞춘 파란공을 중심으로 빨간공의 반대방향으로 원운동을 하면서 파란공과 빨간공의 x축 거리 차이가 platform의 입구 크기의 절반만큼 증가하면 원운동이 종료된다.
	//즉 안전거리 margin만큼 확보가 될 때 까지 원운동을 진행하다가 충돌하지 않을것이라 판단이 되면 다시 state0으로 돌아가 타겟 파란공을 향해 PD control을 하며 다가간다.
	if(state==6){
		data[8]=10;
		cout<<"anti_error :"<<anti_error<<endl;
		cout<<"state = 6(circular moving)"<<endl;

		if(ball_bX.size()==0){			//state6에 도달했는데 화면에 파란공이 없는 상태가 발생하였다는것은 관성을 이겨내지 못하고 제어가 잘 안됨을 나타낸다. 이러한 특이 케이스를 방지하기 위해 타겟을 놓쳤을 때를 대비한 코드이다.
																//본 코드에서 이러한 문제가 나타날 경우, 파란공이 웹캠에 없어졌을때를 지금과 같이 구분하지 않을때 밑에 나오는 circular move 함수에서 core dumped error가 발생하기 때문에 이에 대한 failure plan이다.
			if(anti_error==1){				//anti error에 저장된 값이 1이면 오른쪽으로 과하게 회전하여 웹캠 화면상으로 왼쪽 바깥으로 파란공을 놓친것이기 때문에 공이 중앙에 놓일때 까지 다시 반대로 회전한다.
				align(-20);
			}else if(anti_error==2){	//마찬가지로 저장된 값이 2이면 반대 방향으로 회전한다.
				align(20);
			}else{
				state=0;
			}
		}else{
				int t = setting_a_target(position->img_bx, position->img_by);				//원운동의 중심이 될 파란공의 타겟 라벨 정보를 가져와 원의 반지름 정보를 얻어낸다.
				int d_1=setting_a_redball_target(ball_bX[t], ball_bY[t], position->img_rx, position->img_ry);			//원운동의 종료 시점을 결정할 타겟 파란공과 빨간공 사이의 x축 거리를 계산한다.
				if(d_1>margin){									//platform이 빨간공과 충동하지 않는 거리가 되면
					state=0;											//state 0으로 돌아간다.
				}else{													//아니라면, 원운동을 시작한다.
					if(position->img_rx[0]>0){		//빨간공이 차를 기준으로 오른쪽에 존재하면 왼쪽으로 원운동을 한다.
						circular_move(blue_ball_distance[t],45,atan(ball_bY[t]/ball_bX[t]));		//위에서 선언한 원운동함수를 사용한다.
						cout<<"avoid left"<<endl;
					}else{												//반대로 빨간공이 차를 기준으로 왼쪽에 존재하면 오른쪽으로 원운동을 한다.
						circular_move(blue_ball_distance[t],-45,atan(ball_bY[t]/ball_bX[t]));
						cout<<"avoid right"<<endl;
					}
				}
			}
		}

	////////////////

		//state3은 state 0에서 pickup이 3번 종료되거나, 다음 타겟 파란공을 360도 동안 못찾을 경우 작동한다.
		//먼저 release를 하기 위해 목적지인 초록공 혹은 바구니를 찾을 때 까지 회전한다. 이때, 거리가 멀 경우에는 바구니를 초록공보다 보통 잘 찾아내어 바구니를 먼저 찾는다.
		//차와 바구니의 거리가 2m가 될 때까지 바구니의 중심점까지 PD control로 다가가고, 이후에는 보통 초록공을 잘 찾아내어 더 정밀한 release를 하기 위해 초록공의 중심점을 향해 PD control을 한다.
		//이 도중에 빨간공을 피해야겠다고 판단이 되면, 위에서 나타난 state1이나 6처럼 타겟을 바구니 혹은 초록공들의 중심좌표로 두고 원운동을 하여 피해간다.
		if(state==3){
			cout<<"state3"<<endl;
			if(position->img_gx.size()==0 && position->img_basket_x.size()==0){	//state 3에 도달했는데 바구니가 보이지 않으면 바구니가 보일 때 까지 회전한다.
				align(45);
			}else{
				float grx=setting_green_x(position->img_gx, position->img_gy);
				float gry=setting_green_y(position->img_gx, position->img_gy);
				if(basket_distance[0]>200){																				//만약 바구니와의 거리가 2m보다 클 경우,
					int dm=setting_a_redball_target(basket_X[0], basket_Y[0], position->img_rx, position->img_ry);
					if(dm>margin){																									//도중에 피해야 할 빨간공이 없다면
						PD_control(basket_X[0],basket_Y[0], 8,6);											//바구니를 향해 PD control을 한다.
					}else{																													//피해야 할 빨간공이 나타난다면, state9로 이동해 바구니를 중심으로 빨간공 피하는 과정들을 진행한다.
						state=9;
					}
				}else if(sqrt(grx*grx+gry*gry)>65){																//바구니와의 거리가 2m보다 가까워 초록공들을 잘 발견할 수 있으면, 65cm보다 가까워지기 전 까지 초록공들의 중심좌표로 제어를 하기 시작한다.
					int d=setting_a_redball_target(grx, gry, position->img_rx, position->img_ry);
					if(d>margin){
						PD_control(grx,gry, 8,6);																			//초록공들의 중심좌표로 PD control을 하며 다가가는데, 피해야할 빨간공이 나타난다면
					}else{
						state=7;																											//state7로 이동해 초록공들의 중심좌표를 target으로 빨간공 피하는 과정들을 진행한다.
					}
				}else{													//목표지점까지 65cm보다 가까워졌을 경우,
					if(ball_gX.size()==1){				//이러한 과정 속에서 웹캠에서 초록공을 두개가 아닌 하나만 발견할 경우, 회전을 해 두개를 다 찾을 수 있도록 한다.
						if(ball_gX[0]>0){
							align(20);
						}else{
							align(-20);
						}
					}else if(abs(ball_gY[0]-ball_gY[1])<1.5){		//일반적인 경우로 초록공이 두개 다 보인다면, 현재 초록공들의 중심좌표만 align이 맞춰진 상태이기 때문에, 바구니를 정면으로 보도록 제어를 한다.
						if(grx<-1.5){															//563~567줄 과정은 바구니가 정면으로 맞을 경우, 디테일한 정면 맞추는 과정을 한 후 최종 release state인 state4, 5에 돌입힌다.
							move_left(15);
						}else if(grx>0.5){
							move_right(15);
						}else{
							state=4;
						}
					}else if(ball_gX[0]>ball_gX[1]){						//563줄에서 중심좌표만 align이 맞아 바구니를 정면으로 보도록 제어를 하는 과정이 필요할 때, 이를 중심으로 다시한번 원운동을 한다.
						circular_move(sqrt(grx*grx+gry*gry),15,atan(gry/grx));
						cout<<"circular move to left"<<endl;
					}else if(ball_gX[0]<ball_gX[1]){
						circular_move(sqrt(grx*grx+gry*gry),-15,atan(gry/grx));
						cout<<"circular move to right"<<endl;
					}
			}
		}
	}

	//state7은 state 3에서 초록공들의 중심좌표를 향해 PD control을 하는 도중, 빨간공을 피해야 한다고 판단되었을 때 실행된다.
	//이는 state1과 같이 원의 중심을 향해 align을 먼저 맞추는 과정과 동일하다.
	//원의 중심과 align이 맞으면, state1에서 이어지는 state6처럼 state7에서 align을 맞춘 후 state8로 이어져 빨간공을 피할 수 있을 때 까지 원운동을 진행한다.
	if (state==7){
		cout<<"state = 7(Avoid red ball for greenball)"<<endl;
		float grx=setting_green_x(position->img_gx, position->img_gy);
		float gry=setting_green_y(position->img_gx, position->img_gy);
		cout<<"x pos : "<<grx<< "	y pos : "<<gry<<endl;
		cout<<"distance : "<<sqrt(grx*grx+gry*gry)<<endl;
		int d_1=setting_a_redball_target(grx, gry, position->img_rx, position->img_ry);
		if(d_1>margin){
			state=3;
		}
		else{
			if(grx>7){
				align(30);
			}
			else if(grx<-7){
				align(-30);
			}
			else{
				state=8;
			}
		}
	}

	//////////////////////////
	//state8은 state6과 거의 동일하게 작동한다.
	//state7은 빨간공을 피해야겠다고 판단한 후, 타겟인 초록공들의 중심좌표에 align을 맞춘뒤 작동한다.
	//본 state는 align을 맞춘 초록공들의 중심좌표를 중심으로 빨간공의 반대방향으로 원운동을 하면서 파란공과 빨간공의 x축 거리 차이가 platform의 입구 크기의 절반만큼 증가하면 원운동이 종료된다.
	//즉 안전거리 margin만큼 확보가 될 때 까지 원운동을 진행하다가 충돌하지 않을것이라 판단이 되면 다시 state3으로 돌아가 타겟 초록공들의 중심좌표을 향해 PD control을 하며 다가간다.
	if(state==8){
		data[8]=5;
		cout<<"state = 8(circular moving for greenball)"<<endl;
		float grx=setting_green_x(position->img_gx, position->img_gy);
		float gry=setting_green_y(position->img_gx, position->img_gy);
		cout<<"x pos : "<<grx<< "	y pos : "<<gry<<endl;
		cout<<"distance : "<<sqrt(grx*grx+gry*gry)<<endl;
		int d_1=setting_a_redball_target(grx, gry, position->img_rx, position->img_ry);
		if(d_1>margin){
			state=3;
		}else{
			if(position->img_rx[0]>0){
				circular_move(sqrt(grx*grx+gry*gry),52,atan(gry/grx));
				cout<<"avoid left"<<endl;

			}else{
				circular_move(sqrt(grx*grx+gry*gry),-52,atan(gry/grx));
				cout<<"avoid right"<<endl;
			}
		}
	}

	//////////////////////////
	//state9는 state 7과 거의 비슷한데 유일한 차이점은 target을 초록공의 중심좌표가 아닌 바구니로 지정할 때의 빨간공을 피하는 과정이다.
	//이는 state1과 같이 원의 중심을 향해 align을 먼저 맞추는 과정과 동일하다.
	//원의 중심과 align이 맞으면 원운동을 시작하는 state10으로 넘어간다.
	if (state==9){
		cout<<"state = 9(Avoid red ball for basket)"<<endl;
		float bx=basket_X[0];
		float by=basket_Y[0];
		cout<<"x pos : "<<bx<< "	y pos : "<<by<<endl;
		cout<<"distance : "<<sqrt(bx*bx+by*by)<<endl;
		int d_b=setting_a_redball_target(bx, by, position->img_rx, position->img_ry);
		if(d_b>margin){
			state=3;
		}
		else{
			if(bx>7){
				align(30);
			}
			else if(bx<-7){
				align(-30);
			}
			else{
				state=10;
			}
		}
	}

	//////////////////////////
	//state10은 state6, state8과 거의 동일하게 작동한다.
	//state8과 유일한 차이점은 원운동의 중심이 초록공들의 중심좌표가 아닌 바구니의 좌표라는 것 이다.
	//즉 안전거리 margin만큼 확보가 될 때 까지 원운동을 진행하다가 충돌하지 않을것이라 판단이 되면 다시 state3으로 돌아가 타겟 바구니를 향해 PD control을 하며 다가간다.
	if(state==10){
		data[8]=10;
		cout<<"state = 10(circular moving for greenball)"<<endl;
		float bx=basket_X[0];
		float by=basket_Y[0];
		cout<<"x pos : "<<bx<< "	y pos : "<<by<<endl;
		cout<<"distance : "<<sqrt(bx*bx+by*by)<<endl;
		int d_b=setting_a_redball_target(bx, by, position->img_rx, position->img_ry);
		if(d_b>margin){
			state=3;
		}else{
			if(position->img_rx[0]>0){
				circular_move(sqrt(bx*bx+by*by),52,atan(by/bx));
				cout<<"avoid left"<<endl;
			}else{
				circular_move(sqrt(bx*bx+by*by),-52,atan(by/bx));
				cout<<"avoid right"<<endl;
			}
		}
	}


////////////////////////////////////
	//state4는 바구니를 향해 디테일한 정면을 맞춘 후, 다시 한번 더 정밀하게 바구니와 align을 맞추는 작업을 한다.
	//이전 움직임의 관성을 배제하기 위해 아주 잠깐동안 속도를 0으로 하고, 매우 낮은 속도로 정면을 맞춘다.
	//이러한 align이 맞춰지면 피드백으로 바구니에 다가가 웹캠에 보이지 않을 때 state 5로 넘어가 open loop로 파란공을 release한다.
	if(state==4){
		if(time_last<10){	//10번 루프가 회전할 때 까지 멈추게 한다. 관성을 제거하기 위해.
			movef(0);
		}else{
			cout<<"state 5"<<endl;
			float grx=setting_green_x(position->img_gx, position->img_gy);
			float gry=setting_green_y(position->img_gx, position->img_gy);
			cout<<"distance"<<sqrt(grx*grx+gry*gry)<<endl;
			if(ball_gX[1]>ball_gX[0]){						//디테일한 align 맞추기 과정이다.
				if(ball_gY[1]-ball_gY[0]>1){
					align(-5);
				}

			}else{
				if(ball_gY[1]-ball_gY[0]>1){
					align(5);
				}
			}
			if(sqrt(grx*grx+gry*gry)>40){					//align이 잘 맞으면, 정면으로 웹캠의 최단 가시거리에 가까운 40cm까지 다가가고, 최종 openloop release 단계인 state5로 넘어간다.
				movef(30);
			}else{
				state=5;
			}
		}
		time_last++;
	}

////////////////////////
//state5는 정밀한 align이 완성된 후, 웹캠이 보이지 않는 거리 내에 바구니 앞에 차를 주차시키고 공을 release하는 과정이다.
//본 platform의 rlease 과정의 robust함을 이용해 웹캠을 하나만 사용함에 대한 불가피한 요소인 짧은 구간의 openloop이 들어간다.
//특정 시간동안 직진을 하고, 이 직진과정이 끝나 충분히 바구니와 밀착을 한 후 pickup motor를 반대로 회전시켜 공들을 release한다.
	if(state==5){
		data[7]=0;							//release시에는 pickup motor를 position control이 아닌 velocity control로 하였는데 이는 release에 필요한 모터 회전 바퀴수가 커서 absolute encoder로는
		cout<<"state 4"<<endl;  //position control이 불가능한 각도까지 회전시켜야 하기 때문이다.
		if(parking_time<40){
			movef(30);
			cout<<"parking time : "<<parking_time<<endl;
		}else if(parking_time<170){
			movef(0);
			pick_up(-60);
		}else{
			pick_up(0);
		}
		parking_time++;
	}

//////////////////////////

		cout<<"ball number : "<<ball_number<<endl;
		cout<<"blue ball number : "<<position->img_bx.size()<<endl;
		cout<<"motor1_vel="<<data[1]<<endl;
		cout<<"motor2_vel="<<data[2]<<endl;
		cout<<"motor3_vel="<<data[3]<<endl;
		cout<<"motor4_vel="<<data[4]<<endl;
		cout<<"motor5_vel="<<data[5]<<endl;
		cout<<"360 = release  : "<<theta_release<<endl;
		cout<<"------------------"<<endl;

		write(c_socket, data, sizeof(data));		//전체 루프가 한번 끝나고 결정된 myRIO로 입력해줄 data 값들을 입력한다.
}



int main(int argc, char **argv) //5/3 - distance, setting a target need edit. motor value doesn't work well.
																//5/7 - new error - core dumped when remove balls, Cannot recognize "move" order(only align)
																//5/8 - pick up state dicision(when two blue balls) problem, self off problem .
																//5/12 - Build whole codes Except Release(green ball recognize have to need).
																//5/12 - PD control with theta. Circular move test must be needed.
{
	  ros::init(argc, argv, "data_integation");		//node 선언.
    ros::NodeHandle n;
		dataInit();				//myRIO로 전송하는 data 입력값들을 초기화해준다.
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);	//웹캠에서 ball position node를 통한 메세지로 공의 정보가 들어올 경우 콜백함수가 작동하고
																																																			//본 platform의 모든 ros 알고리즘은 이 콜백함수에서 작동한다.
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
		ros::spin();
		return 0;
}
