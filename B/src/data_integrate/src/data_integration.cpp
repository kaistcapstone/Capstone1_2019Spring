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
#include "core_msgs/ball_pos.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)
#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress
//콜백 함수가 진행되는 동안 데이터를 받지 않기 위한 함수
boost::mutex map_mutex;
//각 색깔의 공은 최대 3개이므로, 공의 x, y 좌표를 저장할 배열 정의
float ball_X[]={
	1000,
	1000,
	1000
	};
float ball_Y[]={
	1000,
	1000,
	1000
	};
//tcp/ip통신을 통해 myRIO에게 보내줄 데이터
int data1[]={0,0,1};//픽업 프레임을 올림
int data2[]={0,0,-1};//픽업 프레임을 내림
int data3[]={1,0,1};//전진하면서 픽업 프레임을 올림
int data4[]={1,0,-1};//전진하면서 픽업 프레임을 내림
int data5[]={0,10,0};//rotation clockwise
int data6[]={0,-10,0};//rotation counterclockwise
int data7[]={1,0,0};//전진
int data8[]={0,0,0};//정지
int data9[]={2,0,0};//right
int data10[]={-2,0,0};//left
int data11[]={0,6,0};//rotation clockwise slowely
int data12[]={0,-6,0};//rotation counterclockwise slowly

int c_socket, s_socket;
struct sockaddr_in c_addr;
int rest=0;
int rotatbasket=0;

float d=0.6;//최소확보거리
float theta;//각도
int t1;//선형시간
int t2;//회전시간
int t3 = 5;//픽업모듈 들어올리는 시간
int t4;// 픽업모듈을 들고 전진하는 시간
float k1=95;//선형운동비례상수
float k2=43;//회전운동비례상수

int cb=0;//채집한 파란공 수
int cg=0;//골대앞 1.2m 정렬 여부 판단
int c80=0;//80프로 전진여부
int c100=0;//그린볼 70프로 feedback 실행 여부
int count=0;//공갯수
float theta_g;//그린볼을 디텍할때 사용할 각도

float xm;
float ym;
float x_mid;
float y_mid;
float x_p;
float y_p;
float gmin_x;
float gmin_y;
float gmax_x;
float gmax_y;
#define RAD2DEG(x) ((x)*180./M_PI)

/////////////////////////////Linear_Functions//////////////////////////////////
void linear(float x, float y){//sqrt(x^2+y^2) - d)의 80% 전진
	t1=int((sqrt(x*x+y*y)-d)*0.8*k1);//전체 거리에서 0.8(=80%)만큼 가기 위해 필요한 for문의 길이
	if(sqrt(x*x+y*y)<d){//파란공이 d보다 가까우면 전진 피드백 생략
		c80=1;
	}
	else{//파란공이 d보다 멀 경우 정상적으로 진행
		for(int i=0; i<t1;i++){
			write(c_socket, data7, sizeof(data7));
			ros::Duration(0.025).sleep();
		}
	}
}

void linear_complete(float x, float y){//파란공에서 d 만큼 떨어진 지점까지 완전히 전진
	t1=int((sqrt(x*x+y*y)-d)*k1);
		for(int i=0; i<t1;i++){
			write(c_socket, data7, sizeof(data7));
			ros::Duration(0.025).sleep();
		}
}

void linear_g(float x, float y){//목표 지점까지 70%의 거리를 전진
	if (y>4){
		write(c_socket, data7, sizeof(data7));
	}
	else{
		t1=int(sqrt(x*x+y*y)*0.7*k1);
		for(int i=0; i<t1;i++){
		write(c_socket, data7, sizeof(data7));
		ros::Duration(0.025).sleep();
		}
	}
}

void linear_g_complete(float x, float y){//목표 지점까지 완전히 전진
	t1=int(sqrt(x*x+y*y)*k1);
	for(int i=0; i<t1;i++){
	write(c_socket, data7, sizeof(data7));
	ros::Duration(0.025).sleep();
	}
}

void moveright(){//짧은 시간동안 오른쪽으로 이동
	for(int i=0; i<3;i++){
	write(c_socket, data10, sizeof(data10));
	ros::Duration(0.025).sleep();
	}
}

void moveleft(){//짧은 시간동안 왼쪽으로 이동
	for(int i=0; i<3;i++){
	write(c_socket, data9, sizeof(data9));
	ros::Duration(0.025).sleep();
	}
}
///////////////////////////Linear_Functions_End///////////////////////////////

//////////////////////////Rotational_Functions////////////////////////////////
void rotation(float x, float y){//입력받은 x,y를 정면으로 정렬하기 위해 회전하는 함수
	theta=atan(x/y);
	t2=int(abs(theta*k2));
	if(x>=0){
		for(int i=0; i<t2;i++){
			write(c_socket, data5, sizeof(data5));
			ros::Duration(0.025).sleep();
		}
	}
	else{
		for(int i=0; i<t2;i++){
			write(c_socket, data6, sizeof(data6));
			ros::Duration(0.025).sleep();
		}
	}
}

void rotation_low(float x, float y){//회전속도가 조금 느린 rotation함수
	theta=atan(x/y);
	t2=int(abs(theta*k2*104/71));
	if(x>=0){
		for(int i=0; i<t2;i++){
			write(c_socket, data11, sizeof(data11));
			ros::Duration(0.025).sleep();
		}
	}
	else{
		for(int i=0; i<t2;i++){
			write(c_socket, data12, sizeof(data12));
			ros::Duration(0.025).sleep();
		}
	}
}

void findballcw(){//단위시간씩 시계방향으로 회전하는 함수
	write(c_socket, data11, sizeof(data11));
	ros::Duration(0.025).sleep();
	}

void findgballcw(){//greenball을 탐색하기 위해 시계방향으로 25도 회전 후 0.3초 정지하는 함수
	for(int i=0; i<20;i++){
	write(c_socket, data5, sizeof(data5));
	ros::Duration(0.025).sleep();
	}
	for(int i=0; i<12;i++){
	write(c_socket, data8, sizeof(data8));
	ros::Duration(0.025).sleep();
	}
}

void findballccw(){//단위시간씩 반시계방향으로 회전하는 함수
	write(c_socket, data12, sizeof(data12));
	ros::Duration(0.025).sleep();
}

void findgballccw(){//greenball을 탐색하기 위해 반시계방향으로 25도 회전 후 0.3초 정지하는 함수
	for(int i=0; i<20;i++){
	write(c_socket, data6, sizeof(data6));
	ros::Duration(0.025).sleep();
	}
	for(int i=0; i<12;i++){
	write(c_socket, data8, sizeof(data8));
	ros::Duration(0.025).sleep();
	}
}

void CW(float Theta){//입력받은 각도 theta만큼 시계방향 회전하는 함수
	t2=int(Theta*k2);
	for(int i=0; i<t2;i++){
		write(c_socket, data5, sizeof(data5));
		ros::Duration(0.025).sleep();
	}
}

void CCW(float Theta){//입력받은 각도 theta만큼 반시계방향 회전하는 함수
	t2=int(Theta*k2);
	for(int i=0; i<t2;i++){
		write(c_socket, data6, sizeof(data6));
		ros::Duration(0.025).sleep();
	}
}

/////////////////////////////Rotation_functions_end///////////////////////////

/////////////////////////////Other_functions//////////////////////////////////
void takerest(int d){//웹캠에서 정확한 데이터를 읽기 위해 만든 함수로 for문의 길이를 입력값으로 받아 vehicle을 정지시키는 함수
	for(int i=0; i<d;i++){
		write(c_socket, data8, sizeof(data8));
		ros::Duration(0.025).sleep();
	}
}

void pick(float x,float y){//60cm 떨어져있는 파란공을 픽업하기 위해 픽업모듈을 들고 전진 후 내리는 함수
		for(int i=0; i<t3;i++){
			write(c_socket, data3, sizeof(data3));
			ros::Duration(0.025).sleep();
		}
		t4=int((d-0.15)*k1-t3);
		for(int i=0; i<t4;i++){
			write(c_socket, data7, sizeof(data7));
			ros::Duration(0.025).sleep();
		}
		for(int i=0; i<t3-1;i++){
			write(c_socket, data4, sizeof(data4));
			ros::Duration(0.025).sleep();
		}
	cb++;
}

void release(){//공을 release하기 위해 픽업프레임을 올리는 함수
	for(int i=0; i<8;i++){
		write(c_socket, data1, sizeof(data1));
		ros::Duration(0.025).sleep();
	}
	cb++;
}
/////////////////////////////////Other_functions_end///////////////////////////

void camera_Callback(const core_msgs::ball_pos::ConstPtr& position)
{
	map_mutex.lock();
	ROS_INFO("line285 <<<<<<<<<<Callback : New message is subscribed>>>>>>>>>>>");//subscribe한 공의 좌표를 기준으로 callback함수가 시작됨을 알리는 메시지

////////////////////////////실험 코드//////////////////////////////////////////
//////////새로운 함수를 만들기 전 단일 동작을 테스트 하기 위한 실험실//////////////
//
//  for(int i=0;i<360 ;i++){//
//  write(c_socket, data5, sizeof(data5));
//  ros::Duration(0.025).sleep();
// }
//
// for(int i=0;i<12000 ;i++){//
// write(c_socket, data8, sizeof(data8));
// ros::Duration(0.025).sleep();
// }
/////////////////////////////실험 코드 end/////////////////////////////////////

	for(int i=0;i<count;i++){//파란공의 좌표를 받을 배열에 디폴트 값을 입력
		ball_X[i]=1000;
		ball_Y[i]=1000;
	}

	if(cb<3){//파란공을 주울 때마다 cb값이 증가한다. 즉, 파란공을 다 주울 때까지 돌아갈 if문
		count = position->b_size;//캠에서 보이는 파란공의 갯수를 받음
		if (count==0){//공이 하나도 안 보일 경우
			findballcw();//공을 찾기 위해 시계방향으로 회전시킨다. 우리의 알고리즘은 시계방향으로 파란공들을 주워나갈것이기 때문에 무조건 시계방향으로 돌아도 문제가 없다.
			ROS_INFO("line312 cb=%d, No blueball, findballcw",cb);
		}
		else{//공이 보일경우
			float p=1000;//미니멈 저장변수
			int j = 0;//미니멈의 j번째
			for(int i = 0; i < count; i++){//공들의 좌표를 받아오면서, 이들 중 가장 왼쪽에 있는 공을 선택한다. 이 공을 주울 것이기 때문
				ball_X[i] = position->b_img_x[i];
				if (ball_X[i]<p){
					p = ball_X[i];
					j=i;
				}
				ball_Y[i] = position->b_img_y[i]-0.02;
	    		}
			//가장 왼쪽에 있는 공을 xm, ym에 저장
			xm=p;
			ym=ball_Y[j];
			//이 for문은 가장 왼쪽에 있지 않지만, 가장 왼쪽에 있는 공과의 x좌표 거리가 굉장히 가깝고, 가장 왼쪽에 있는 공보다 차체에 더 가까이 있는 경우가 있는지 찾는다.
			for(int i=0; i< count;i++){
			   if(i!=j){
				//이러한 경우가 존재할 경우, 이 공의 좌표를 타겟 xm, ym로 저장한다.
			  	 if (ball_X[i]>xm-0.3 and ball_X[i]<xm+0.3 and ball_Y[i]<ym){
			  			xm=ball_X[i];
							ym=ball_Y[i];
			   		}
			   }
		   }
			ROS_INFO("line340 cb=%d, Blueball is at (xm:%f,ym:%f)",cb,xm,ym);

			if (xm>-0.05 and xm<0.05){// xm의 좌표가 중앙정렬이 잘 되어있을 경우(오차=5cm)
					if(c80<1 and ym>1){//피드백의 횟수가 다 채워질때까지 80프로 전진을 한다.
						ROS_INFO("line344 'linear' to (xm:%f, ym:%f), distance:%f // c80: %d",xm,ym,sqrt(xm*xm+ym*ym),c80);
						linear(xm,ym);
					  takerest(12);//openloop function to xm,ym
						c80++;
					}
					else{//피드백 횟수가 다 채워지면 완전히 전진한 뒤, 픽업과정을 실행시키고, 모든 상수들을 기본값으로 바꾸어준다.
							ROS_INFO("line350 c80: %d, linear_fb to blueball at %fm",c80,sqrt(xm*xm+ym*ym));
							linear_complete(xm,ym);
							ROS_INFO("line352 c80: %d, d: %f rotation %fdeg & pick",c80,sqrt(xm*xm+ym*ym),atan(xm/ym)*57.296);
							pick(xm,ym);
							takerest(12);
							c80=0;
						}
				}

			else{// xm 좌표가 중앙정렬이 잘 되어있지 않다면, 좌표를 받아 그만큼 회전을 시켜 중앙정렬을 수정한다.
				rotation(xm,ym);
				takerest(12);
				ROS_INFO("line363 rotation %fdeg to blueball(xm: %f, ym: %f)",atan(xm/ym)*57.296,xm,ym);
			}
		}
	}
	else if(cb==3){//픽업이 다 완료되었을 경우, 이제 초록 공을 데이터로 받아 이 if문을 실행시킨다.
		count = position->g_size;
		for(int i = 0; i < count; i++){//초록색 공의 좌표들을 받아온다.
				ball_X[i] = position->g_img_x[i];
				ball_Y[i] = position->g_img_y[i]-0.02;
	    		}
		ROS_INFO("line373 cb: %d, %dgreenballs are detected",cb,count);

		if(count!=2){//카메라에 초록색 공 2개가 보이지 않을 경우 시계방향으로 회전
					findgballcw();
		}
		else{//카메라에 초록색 공이 2개 모두 보일 경우
			if (rest==0){//파란공 3개 채집 후, 카메라에 처음으로 greenball이 2개 보였을 때 바구니와의 거리가 멀기 때문에 정확한 좌표 측정을 위해 잠시 멈춤
				takerest(12);
				rest=1;
			}
			else{//정지 이후
				//두 공의 중점 좌표를 계산한다.
				x_mid=(ball_X[0]+ball_X[1])/2;
				y_mid=(ball_Y[0]+ball_Y[1])/2;
				//cg 상수는 정렬 위치로 도달 전과 후를 구분해준다. cg=0일 경우는 정렬 지점으로 도달하는 과정이다.
				if(cg==0){
					//정렬할 위치는 바구니에서 수직으로 길이 L=1.2m 떨어져 있다 이 좌표를 계산하는 과정
					theta_g=atan((ball_Y[0]-ball_Y[1])/(ball_X[0]-ball_X[1]));
					x_p=x_mid+1.2*sin(theta_g);
					y_p=y_mid-1.2*cos(theta_g);
					ROS_INFO("line395 cg:%d, 2 greenballs detected! theta_g : %fdeg",cg,theta_g*57.296);
					ROS_INFO("line396 (x_p:%f, y_p:%f), (x_mid:%f, y_mid:%f) <= g1(%f,%f), g2(%f,%f)", x_p,y_p,x_mid,y_mid,ball_X[0],ball_Y[0],ball_X[1],ball_Y[1]);

					if(c100<1){//바구니로부터 1.2m 지점까지 가기 위한 첫번째 피드백
						rotation(x_p,y_p);//이 지점으로 직진하기 전 회전을 하여 중앙정렬을 한다.
						ROS_INFO("line400 cg:%d, rotation %fdeg to (xp:%f, yp:%f)",cg,atan(x_p/y_p)*57.296,x_p,y_p);
						//중앙정렬 후 도달해야할 지점에서 70프로 거리를 직진한다.
						linear_g(x_p,y_p);
						//직진 후 두 공이 카메라의 시야에서 벗어날 경우가 많았다. 그래서 다음과 같은 계산으로 두 공이 시야에서 다시 제대로 보일 수 있도록 회전해야할 각도를 구한다.
						float alpha=atan(x_p/y_p)-atan(x_mid/y_mid);
						float theta_c = atan(sqrt(x_mid*x_mid+y_mid*y_mid)*sin(alpha)/(sqrt(x_mid*x_mid+y_mid*y_mid)*cos(alpha)-0.7*sqrt(x_p*x_p+y_p*y_p)));
						//각도가 음의 값이 나올 경우 양의 값으로 바꾼다.
						if (theta_c<0){
							theta_c=-theta_c;
						}
						ROS_INFO("line420 theta_c is %f", theta_c);
						//x_mid좌표와 x_p좌표를 사용하여 회전방향을 정해서 회전한다.
						if(x_mid<x_p){
							CCW(theta_c);
						}
						else{
							CW(theta_c);
						}
						takerest(12);
						ROS_INFO("line428 cg=%d, linear_g to (x_p:%f, y_p:%f)",cg,x_p,y_p);
						c100++;
					}
					else{//피드백 횟수가 다 채워졌을 경우
						rotation(x_p,y_p);
						linear_g_complete(x_p,y_p);//도달 지점으로 완전히 이동한다. 나머지의 코드들은 위의 코드들과 동일하다.
						ROS_INFO("line452 xp: %f yp: %f x_mid: %f y_mid: %f", x_p, y_p, x_mid,y_mid);
						float alpha=atan(x_p/y_p)-atan(x_mid/y_mid);
						float theta_c = atan(sqrt(x_mid*x_mid+y_mid*y_mid)*sin(alpha)/(sqrt(x_mid*x_mid+y_mid*y_mid)*cos(alpha)-sqrt(x_p*x_p+y_p*y_p)));
						if (theta_c<0){
							theta_c=-theta_c;
						}
						ROS_INFO("line458 theta_c is %f", theta_c);
						if(x_mid<x_p){
							CCW(theta_c);
						}
						else{
							CW(theta_c);
						}
						takerest(12);
						ROS_INFO("line466 cg=%d, linear_g_complete to (x_p:%f, y_p:%f",cg,x_p,y_p);
						cg++;
					}
				}

				else{//cg=1이 되었을 상황이다 이때부터는 바구니로 정확히 도달할 수 있도록 차체를 정렬하는 과정이다.
					if (rotatbasket==0){//정확한 값을 받도록 천천히 회전하면서 두 공의 중심 좌표로 중앙정렬한다.
						rotation_low(x_mid,y_mid);
						rotatbasket=1;
					}
					//두 공을 순서에 맞게 gmin, gmax에 저장
					if(ball_X[0]<ball_X[1]){
						gmin_x=ball_X[0];
						gmin_y=ball_Y[0];
						gmax_x=ball_X[1];
						gmax_y=ball_Y[1];
					}
					else{
						gmin_x=ball_X[1];
						gmin_y=ball_Y[1];
						gmax_x=ball_X[0];
						gmax_y=ball_Y[0];
					}
					ROS_INFO("line489 gmin_x: %f gmax_x: %f", gmin_x, gmax_x);
					ROS_INFO("line490 gmin_y: %f gmax_y: %f", gmin_y, gmax_y);
					//수평 정렬을 하기 위한 if문. 오차 = 1.5cm
					if(gmax_y-gmin_y>0.015 or gmin_y-gmax_y>0.015){
						if (gmax_y-gmin_y>0){
							for(int i = 0; i < 3; i++){
								findballccw();
							}
								takerest(12);
						}
						else{// need to modify the number of loop
							for(int i = 0; i < 3; i++){
								findballcw();
							}
								takerest(12);
						}
					}
					else{//수평 정렬이 다 되었을 경우, x_mid, y_mid(두 초록공의 중점 좌표)를 기준으로 중앙정렬을 시작한다.
						if (x_mid>0.05){//원점에서 5cm 밖에 있을 경우, 오른쪽으로 수평운동
									for(int i = 0; i < 5; i++){
										moveright();
									}
						}//
						else if(x_mid<-0.05){//원점에서 -5cm 밖에 있을 경우, 왼쪽으로 수평운동
									for(int i = 0; i < 7; i++){
										moveleft();
									}
						}//
						else{//중앙정렬까지 완료되었을 경우
								for(int i = 0; i < 12; i++){//픽업프레임을 바구니 안으로 넣기 위해 약간 들어준다.
									write(c_socket, data1, sizeof(data1));
									ros::Duration(0.025).sleep();
								}//
								linear_g_complete(x_mid,y_mid-0.13);//바구니로 전진
								release();//픽업프레임을 들어서 공을 release한다.
								ROS_INFO("line529 Mission Complete!!!");
							}
						}
					}
					rest=0;//콜백을 돌릴때마다 사용할 것이기 때문에 다시 기본값으로 설정
			}
		}
		}

	else{// when cb>3, stop after release, cb=4일 경우로 모든 과정이 다 완료됨. 정지하는 데이터를 보내준다.
		write(c_socket, data8, sizeof(data8));
		ros::Duration(0.025).sleep();
	}
	map_mutex.unlock();
	ROS_INFO("line543 >>>>>>>>>>>>>>>>>>>>>Callback End<<<<<<<<<<<<<<<<<<<<<<<<<<");//subscribe한 좌표로 구동한 callback함수가 종료되었음을 알리는 메시지
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_pos>("/position", 1, camera_Callback);
    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
			printf("Failed to connect\n");
        close(c_socket);
        return -1;
    }

    while(ros::ok){
	ros::spinOnce();
    }
    return 0;
}
