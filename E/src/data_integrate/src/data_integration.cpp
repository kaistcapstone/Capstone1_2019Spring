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
#include <algorithm>
#include <math.h>
#include <ctime>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/ball_position_back.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "opencv2/opencv.hpp"

#define PI 3.141592


#define PORT 4000 //랩뷰와의 통신 port
#define IPADDR "172.16.0.1" // host computer ip
// #define IPADDR "172.22.11.2" // myRIO ip

#define CCW 0
#define CW 1
////////////////////////////////////////////
// to find optimum
#define ROOP_RATE 0.01 // ms
#define ROOP_RATE_RETURN 0.01 // ms
#define THRESHOLD_TURN 10 // in point turn
#define THRESHOLD_TARGET_G 2 // in x align
#define THRESHOLD_Y_DIFFERENCE_G 2 // in y align
#define THRESHOLD_TURN_G 40 // in point turn - green ball
#define TURN_TIME_180 12.4 // time required for 180 deg turn
using namespace std;
/////////////////////////////////////////////
float target_x;
float target_y;
float target_distance;
int res;
int turn_direction;

float curr_time;
float flag_time;
float turn_time;
int ball_number;
int ball_number_G;
int picked_ball;
float ball_X[20];
float ball_Y[20];
float ball_X_G[20];
float ball_Y_G[20];
// float ball_distance[20];

float green_ball_center_x;
float green_ball_center_y;
float prev_ball_X_G;

int c_socket, s_socket;
struct sockaddr_in c_addr;

float data[24];
/*마이리오에게 구동 신호를 줄때 96byte의 data array의 형태로 신호를 보낸다. 본 노드에서 약속한
구동은 다음과 같다.
data[4] 좌우 병진이동
data[5] 전후 병진이동
data[9] 회전 운동
data[14] 공을 방출하는 통로 문에 포함된 모터 구동*/

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

/*본 팀의 로봇에는 세개의 웹캠이 있다. 전면, 후면, 픽업 부분에 웹캠이 있는데,
본 노드에서 사용하는 웹캠은 전면 웹캠과 후면 웹캠이다.
전면웹캠에서 보내오는 데이터는 파란공, 빨간공, 녹색공의 갯수와 각각의 픽셀좌표이고,
후면 웹캠에서 보내오는 데이터는 녹색공의 갯수와 각각의 픽셀좌표이다.
루프가 새로 시작될떄마다 공의 좌표를 업데이트하며, 계속적인 피드백 과정을 통해 로봇의 거동을 결정하게 된다*/

void ballInit(){ //ball initialization for front camera
  for(int i = 0; i < 20; i++){
    ball_X[i] = 0;
    ball_Y[i] = 0;
    /*전면 카메라로 부터 받은 파란공의 픽셀좌표는 ball_X, ball_Y array 에 저장되는데,
    각 루프가 돌때마다 웹캠으로 부터 새로운 값을 받아야 하기 때문에 각 array의 데이터를 0으로 초기화 시킨다.*/
  }
}

void ball_G_Init(){ //ball initialization for back camera
  for(int i = 0; i < 20; i++){
    ball_X_G[i] = 0;
    ball_Y_G[i] = 0;
    /*후면 카메라로 부터 받은 녹색공의 픽셀좌표는 ball_X_G, ball_Y_G array 에 저장되는데,
    각 루프가 돌때마다 웹캠으로 부터 새로운 값을 받아야 하기 때문에 각 array의 데이터를 0으로 초기화 시킨다.*/
  }
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  /*본 함수는 전면 웹캠 데이터를 subscribe 할때마다 콜백되는 함수로써,
  기능은 전면 카메라로 부터 받은 데이터의 값들을 전역변수에 저장하는데에 있다.
  core_msgs파일에 있는 position메세지 파일 형태를 취하는 메세지를 받게 된다 */
  int count_b = position->size;
  /*position 메세지 형태에서 size에 해당하는 값(size에는 파란공의 갯수값이 들어가 있다.)을 count_b 변수에 저장하였는데,
  count_b에 전면 웹캠에 보이는 파란공의 갯수를 저장한 것이다.*/
  ball_number = count_b;
  /*ball_number는 전역변수로써 전면 카메라에 보이는 파란공의 갯수를 의미한다, 이 변수는 노드 전체에서 쓰이기 때문에
  웹캠의 데이터가 업데이트 될 떄마다 본 함수가 작동되면서 같이 업데이트 된다.*/
  ballInit(); // 새로운 값들을 넣기 위하여 ball_X.ball_Y array의 이전의 값들을 모두 0으로 초기화 하였다.
  for(int i = 0; i < count_b; i++){ //for문을 전면카메라에 보이는 파란공의 갯수만큼만 돌림으로써 쓰레기값이 들어가지 않도록 하였다.
    ball_X[i] = position->img_x[i];
    ball_Y[i] = position->img_y[i];
    /*for 문을 통해서 position 메세지 형태의 img_x, img_y에 해당하는 값(파란공의 픽셀좌표값)을 ball_X. ball_Y array 에 저장한다.
    이 과정을 통해서 ball_X, ball_Y array에는 전면 카메라에 보이는 파란공의 픽셉 좌표 정보가 입력되게 된다.*/

  }
}


void back_camera_Callback(const core_msgs::ball_position_back::ConstPtr& position_back)
{
  /*본 함수는 후면 웹캠 데이터를 subscribe 할때마다 콜백되는 함수로써,
  기능은 후면 카메라로 부터 받은 데이터의 값들을 전역변수에 저장하는데에 있다.
  core_msgs파일에 있는 position_back메세지 파일 형태를 취하는 메세지를 받게 된다 */
  int count_g = position_back->size3;
  /*position_back 메세지 형태에서 size3에 해당하는 값(size3에는 녹색공의 갯수값이 들어가 있다.)을 count_g 변수에 저장하였는데,
  count_g에 후면 웹캠에 보이는 녹색공의 갯수를 저장한 것이다.*/
  ball_number_G = count_g;
  /*ball_number_G는 전역변수로써 후면 카메라에 보이는 녹색공의 갯수를 의미한다, 이 변수는 노드 전체에서 쓰이기 때문에
  웹캠의 데이터가 업데이트 될 떄마다 본 함수가 작동되면서 같이 업데이트 된다.*/
  ball_G_Init(); // 새로운 값들을 넣기 위하여 ball_X_G.ball_Y_G array의 이전의 값들을 모두 0으로 초기화 하였다.
  for(int j = 0; j < count_g; j++){//for문을 후면카메라에 보이는 녹색공의 갯수만큼만 돌림으로써 쓰레기값이 들어가지 않도록 하였다.
    ball_X_G[j] = position_back->img_x3[j];
    ball_Y_G[j] = position_back->img_y3[j];
    /*for 문을 통해서 position_back 메세지 형태의 img_x3, img_y3에 해당하는 값(녹색공의 픽셀좌표값)을 ball_X_G. ball_Y_G array 에 저장한다.
    이 과정을 통해서 ball_X_G, ball_Y_G array에는 후면 카메라에 보이는 녹색공의 픽셉 좌표 정보가 입력되게 된다.*/
  }
}

void sorting_Callback(const std_msgs::Int32::ConstPtr& ball){
  /*본 함수는 sorting_node로 부터 받는 메세지가 subscribe 될때마다 콜백되는 함수로써,
  기능은 sorting_node로 부터 받은 메세지를 전역변수인 picked_ball에 저장하는데에 있다.
  sorting_node에서 메세지를 publish할때 std_msgs의 Int32 형태의 메세지를 사용하였는데,
  보낼데이터가 int 형태의 정수형 변수 하나였기 때문에, 새로운 메세지 형태를 만들지 않고,
  std_msgs library를 이용하는 것이 편리하다고 판단해서 였다.*/
  picked_ball = ball->data;
  /*data 에는 픽업된 파란공의 갯수 데이터가 넣어져 있으며, 이 값을 전역변수인 picked_ball에 저장하였다.
  즉 picked_ball  전역 변수가 의미하는 것은 지금까지 픽업한 파란공의 갯수이다*/

}

void go_straight(float ball_x)
{/* 본 함수의 기능은 ball_x값을 입력받아 이 좌표를 이용해 공과의 수평을 맞추면서 직진을 하는데에 있다.
  myRIO에 보내는 속도 신호는 합쳐서 1로 제한하였으며,
  뒤에서 쓰일 전진속도인 0.8과 좌우 병진이동 속도인 0.2는 여러번의 실험을 통해서
  최적의 속도라 판단, 결정하였다.
  이때 ball_x 값으로는 로봇이 향해서 직진하게 될 공의 x좌표를 입력받는다. */
  dataInit(); // 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  if(ball_x > 0){
    /*입력받은 ball_x의 값이 양수이면 로봇이 공보다 왼쪽에 있다는 것을 의미하므로, 직진 신호를 주는 동시에 우측으로 이동하는 신호를 준다*/
    data[5] = 0.8; //data[5]에 해당하는 값은 앞뒤이동에 관한 명령으로 양수면 앞으로 직진을 의미한다.
    data[4] = 0.2; //data[4]에 해당하는 값은 좌우 이동에 관한 명령으로 양수면 오른쪽 병진이동을 의미한다

  }
  else{
    //ball_x값이 음수이면 로봇이 공보다 오른쪽에 있는것으로, 직진과 동시에 좌측이동 신호를 준다.
    data[5] = 0.8;
    data[4] = -0.2;//좌우이동에 관한 명령으로 음수면 왼쪽 병진이동을 의미한다.
 }
}

void go_back(float target)
{/*본 함수의 기능은 target 값을 입력받아 target이 되는 지점과의 수평을 맞추면서 후진하는데에 있다
  이 함수는 바구니로 귀환할때 쓰이게 된다 이때 target값으로는 두 녹색공의 중심의 x좌표값을 입력받게 된다*/
  dataInit(); // 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다

  if(target > 0){
    /*후면 카메라 상에서 보이는 값을 비교하는 것이기 때문에 명령이 go_straight과는 반전된다*/
    data[5] = -0.8; //data[5]에 해당하는 값은 앞뒤이동에 관한 명령으로 음수면 뒤로 후진을 의미한다.
    data[4] = -0.2; //data[4]에 해당하는 값은 좌우 이동에 관한 명령으로 음수면 왼쪽 병진이동을 의미한다
  }
  else{
    data[5] = -0.8;
    data[4] = 0.2; //오른쪽 병진이동
  }
}


void y_align(float y1, float y2){
  /* 본 함수의 기능은 로봇의 후면이 바구니를 평행하게 바라보게 하는 것이다*/
  dataInit(); // 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  if(y1 > y2){
    /*y1. y2값을 입력받는데, 이때 y2값은 후면 카메라에서 x좌표가 가장 큰 값을 가지는 녹색공의 y좌표이고,
    y1값은 상대적으로 작은 x좌표를 가지는 녹색공의 y좌표값이다.
    따라서 y1>y2인 상태에서는 오른쪽 회전을 통해서 바구니와 평행하게 위치하도록 보정한다.
    반대의 경우에는 왼쪽 회전을 통해서 바구니와 평행하게 위치하도록 한다.
    이때 y1=y2인 경우에는 바구니와 평행하게 있다고 판단하게 된다.
    (픽셀값을 이용하기 때문에 로봇의 좌우 위치와 상관없이 바구니와 평행하게 align되면 y1=y2기 된다)*/
    data[9] = 0.2; //data[9]에 해당하는 값은 회전에 관한 명령으로 양수면 오른쪽 회전을 의미한다.
  }
  else{
    data[9] = -0.2; //왼쪽 회전
  }
}

void x_align(float x_center){
  /*본 함수의 기능은 로봇이 바구니와 평행하게 align된 후에 좌우 병진이동을 통해서 로봇의 후면이 바구니를 정면으로 바라보게 하는 것에 있다
  이때 입력받는 x_center의 값은 두 녹색공의 중심의 x좌표이다*/
  dataInit(); // 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  if(x_center > 40){
    /* 이때 두 녹색공의 중심좌표에서 40만큼 더한 지점을 기준으로 잡는 이유는 로봇이 파란공을 저장하고 방출하는 통로가
    후면웹캠의 오른쪽에 치우쳐 있기 때문에, 파란공이 방출되는 통로가 바구니의 중심으로 맞춰지도록 하기 위해서이다.
    */
    data[4] = -0.3; // 왼쪽 이동 명령
  }
  else{
    data[4] = 0.3; //오른쪽 이동 명령
  }
}



/* 세 개의 함수 point_turn, point_turn_final, point_turn_slow는 각각의 기능은 동일하나,
명령하는 회전 각속도가 다르다. 이는 여러번의 실험을 통해서 각각의 과정마다 필요로 하는 최적의 각속도가 다르다고 판단하였고,
이에 따라서 각각 다른 각속도를 명령하는 함수 3개를 만들었다.
point_turn 함수는 파란공 searching 할때에 쓰이고, point_turn_final은 두 녹색공의 중심을 searching 할때,
point_turn_slow는 두 녹색공의 중심의 좌표를 이용해 로봇을 바구니 앞으로 align 할때 쓰이게 된다.
*/

void point_turn(int direction){
  /*본 함수의 기능은 0또는 1인 direction 값을 입력받아 왼쪽 혹은 오른쪽으로 회전하도록 명령하는 것이
  0이면 왼쪽 회전을, 1이면 오른쪽 회전을 의미한다*/
  dataInit(); // 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  if(direction == CCW){ // turn ccw
    data[9] = -0.5;
  }
  else if(direction == CW){ // turn cw
    data[9] = 0.5;
  }
}

void point_turn_final(int direction){
  /*본 함수의 기능은 0또는 1인 direction 값을 입력받아 왼쪽 혹은 오른쪽으로 회전하도록 명령하는 것이
  0이면 왼쪽 회전을, 1이면 오른쪽 회전을 의미한다*/
  dataInit();// 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  if(direction == CCW){ // turn ccw
    data[9] = -0.6;
  }
  else if(direction == CW){ // turn cw
    data[9] = 0.6;
  }
}

void point_turn_slow(int direction){
  /*본 함수의 기능은 0또는 1인 direction 값을 입력받아 왼쪽 혹은 오른쪽으로 회전하도록 명령하는 것이
  0이면 왼쪽 회전을, 1이면 오른쪽 회전을 의미한다*/
  dataInit();// 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  if(direction == CCW){ // turn ccw
    data[9] = -0.3;
  }
  else if(direction == CW){ // turn cw
    data[9] = 0.3;
  }
}


void release_ball()
{ /*본 함수의 기능은 로봇이 바구니 앞으로 align되었을때 공을 방출하기 위해 저장소 문에 쓰인 모터를 움직여 파란공 저장고 문을 여는 것이다*/
  dataInit(); // 새로운 명령을 저장하기 위해서 이전에 data array(myRIO에 보내는 명령을 저장한 array)에 있었던 값을 초기화 한다
  data[14]=1;
  /*data[14]에 해당하는 값은 파란공을 방출하는 통로에 달린 문에 쓰인 모터를
  통제하는 것으로 이 값이 1이 되면 문이 바깥쪽으로 열리면서 파란공이 방출된다.*/
}

void find_final_position(float x1, float y1, float x2, float y2)
{/*본 함수의 기능은 두 녹색공의 좌표를 입력받아 두 녹색공의 중심 좌표를 찾는데에 있다.
  입력받은 값을 이용해 중심의 좌표를 계산해서 각각 전역변수인 green_ball_center_x, green_ball_center_y에 저장한다.
  계산된 녹색공의 중심 좌표를 이용해서 로봇을 바구니 앞으로 위치시키게 된다*/
  printf("x1 = %f y1 = %f x2 = %f y2 = %f\n", x1, y1, x2, y2); //디버깅을 위한 print
  green_ball_center_x = 0;   green_ball_center_y = 0; //새로운 값을 저장하기 전에 원래 있던 값을 초기화 시킨다.
  green_ball_center_x = (x1 + x2)/2; //두 녹색공 중심의 x좌표
  green_ball_center_y = (y1 + y2)/2; //두 녹색공 중심의 y좌표
  printf("green_ball_center_x = %f\n", green_ball_center_x );
  printf("green_ball_center_y = %f\n", green_ball_center_y ); //디버깅을 위한 print
}

void green_ball_sorting(){
  /*본 함수의 기능은 웹캠에서 무작위로 받은 녹색공에 대한 좌표를 재배열하는데에 있다
  두 녹색공 중, 후면 카메라 상에서 가장 오른쪽에 위치한 녹색공의 좌표를 ball_X_G[1], ball_Y_G[1]에 저장한다.
  즉 왼쪽에 위치한 공의 좌표를 array 상의 0 자리에, 오른쪽에 위치한 공의 좌표를 array 상의 1자리에 넣어 재배열 하는 것이다*/
  if(ball_X_G[0] > ball_X_G[1]){
    // ball_X_G[0]값이 더 크다면 ball_X[0]에 해당하는 공이 더 오른쪽에 위치한다는 것으로 array 상의 0과1자리에 위치한 두 데이터 값을 바꾼다.
    float tempGX = ball_X_G[0];
    float tempGY = ball_Y_G[0];
    ball_X_G[0] = ball_X_G[1]; // ball_X_G[1]의 값을 ball_X_G[0]로 대입한다
    ball_Y_G[0] = ball_Y_G[1];
    ball_X_G[1] = tempGX; //원래 ball_X_G[0]이었던 값을 ball_X_G[1]으로 대입한다.
    ball_Y_G[1] = tempGY;
  }
}

void find_target(){
  /*본 함수의 기능은 전면 웹캠상에서 보이는 파란공의 x좌표를 비교해, 가장 오른쪽에 위치한 파란공의 좌표를
  전역변수인 target_x, target_y에 저장하는데에 있다.*/
  target_distance = -1000;
  /*target_distance값을 -1000으로 초기화하는데,
  이때 -1000으로 초기화 하는 이유는 for문을 돌려서 처음 ball_X[0]값과 비교할때
  무조건 ball_X[0]값을 target_distance 변수에 저장하기 위해서 이다.  */
  for(int i = 0; i < ball_number; i++){
    if(ball_X[i]>target_distance){
      /*저장된 target_distance 값과 ball_X값을 비교해서 ball_X값이 더 크면 ball_X[i]가 더 오른쪽에 있다는 의미가 됨으로
      이 값을 target_distance에 새롭게 저장해 다음 ball_X[i+1]값과 비교한다.*/
      target_distance = ball_X[i];
      target_x = ball_X[i];
      target_y = ball_Y[i];
      /*for문을 다 돌게 되면, 가장 큰 x좌표를 가지는 ball_X, ball_Y값이 target_x, target_y에 저장되게 된다*/
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_integration_node"); //노드명 초기화
  ros::NodeHandle nh; //통신을 위한 노드 핸들 선언
  ros::Subscriber sub1 = nh.subscribe<core_msgs::ball_position>("/position123", 1000, camera_Callback);
  /*ball_detect_node(전면 카메라)로 부터 파란공, 빨간공, 녹색공의 갯수와 픽셀좌표 데이터를 subscribe한다.
  토픽 메세지 명은 position123이고, 큐 사이즈는 1000으로 한다. 메세지를 subscribe할때마다 camera_Callback 함수를 실행시킨다. */
  ros::Subscriber sub2 = nh.subscribe<std_msgs::Int32>("/picked_ball", 1000, sorting_Callback);
  /*sorting_node로 부터 픽업된 파란공의 갯수를 subscribe한다.
  토픽 메세지 명은 picked_ball이고, 큐 사이즈는 1000으로 한다. 메세지를 subscribe할때마다 sorting_Callback 함수를 실행시킨다. */
  ros::Subscriber sub3 = nh.subscribe<core_msgs::ball_position_back>("/position_back", 1000, back_camera_Callback);
  /*ball_detect_back2_node(후면 카메라)로 부터 녹색공의 갯수와 픽셀좌표 데이터를 subscribe한다.
  토픽 메세지 명은 position_back이고, 큐 사이즈는 1000으로 한다. 메세지를 subscribe할때마다 back_camera_Callback 함수를 실행시킨다. */
  // ---------- wait for 1sec -------------
  /*while문을 돌려 각 노드로 부터 받은 메세지를 받은후에 1초동안 대기하게 하였다.
  이는 카메라에서 처음 데이터를 받을때 처음 몇개는 쓰레기값이 subscribe되는 것을 확인하였고,
  이에 쓰레기 값을 사용하지 않기위해 1초간 정지하여, reasonable한 데이터가 들어올때까지 기다리게 하였다.*/
  int o = 0;
  while(o == 0){
    o++;
    ros::Duration(1).sleep(); //1초동안 쉰다.
  }
  // ---------- tcp connection을 위한 코드, ip와 port입력 -------------
  c_socket = socket(PF_INET, SOCK_STREAM, 0);
  c_addr.sin_addr.s_addr = inet_addr(IPADDR);
  c_addr.sin_family = AF_INET;
  c_addr.sin_port = htons(PORT);

  /*labVIEW와 연결이 되지 않았을때 print된다.*/
  if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
    printf("Failed to connect\n");
		close(c_socket);
		return -1;
  }
  // ---------- drvie start -------------
  int stage_number = 0;
  /*stage_number는 구동과정에서 로봇이 위치한 stage를 규정하고, 수행해야할 활동을 결정하는 중요한 전역변수로,
  구동이 시작되기에 앞서, 출발선에 해당하는 stage_number = 0 로 초기화 시킨다. */
  turn_time = 0;
  /*turn_time변수는 로봇이 회전할때마다 실행되어 회전을 몇초동안하는지 계산하는데, 구동에 앞서서 초기화 0으로 시킨다.
turn_time 변수는 반시계 방향으로 돌때 루프주기만큼 더해지고, 시계 방향으로 돌때 루프주기만큼 빼진다 */
  while(ros::ok){
    /*break명령이 있기 전까지 루프를 반복한다.
    이 while문 안에서 stage 0에서 5까지 진행된 후에 while 문을 나가게 된다. */
    ros::spinOnce(); //메세지를 한번 subscribe해 콜백함수들을 실행시킨다.
    find_target(); //전면 카메라 상에서 가장 오른쪽에 위치한 파란공의 좌표를 받아온다.

    // ----------- stage 0 : turn ------------
    if(stage_number == 0){
      /*stage 0 의 목표는 가장 오른쪽에 있는 파란공을 향하도록 로봇을 회전시키는 것이다.
      따라서 로봇이 회전을 통해서 가장 오른쪽에 있는 파란공을 바라보게 되었을때 stage_number를 증가시켜 다음 stage로 이동시킨다. */
      printf("stage 0\n");
      if(abs(target_x) < THRESHOLD_TURN){
        /*로봇이 회전을 통해 가장 오른쪽에 위치한 파란공의 좌표를 오차범위내에 위치시키면 다음 stage로 이동시킨다.*/
        stage_number++;
      }
      else{
        if(target_x < 0){
          //가장 오른쪽에 있는 파란공의 x좌표가 음수라는 것은 로봇이 왼쪽(반시계 방향으로)으로 돌아야 함을 의미한다.
          point_turn(CCW); //왼쪽 회전
          printf("turn left : %f \n", target_x);
          turn_time += ROOP_RATE; //반시계 방향으로 돌았으므로 turn_time에 루프주기 만큼 더한다.
        }
        else{
          //가장 오른쪽에 있는 파란공의 x좌표가 양수라는 것은 로봇이 오른쪽(시계 방향으로)으로 돌아야 함을 의미한다.
          point_turn(CW); //오른 회전
          printf("turn right : %f \n", target_x);
          turn_time -= ROOP_RATE; //시계 방향으로 돌았으므로 turn_time에 루프주기 만큼 뺀다.
        }
      }
    }
    // ----------- stage 1 : go straight ------------
    /*stage 1의 목표는 현재 로봇이 바라보고 있는 파란공을 향해 직진하는 것으로, 파란공을 픽업할때 까지 직진하며,
    픽업하면 stage를 이동시킨다.*/
    if(stage_number == 1){
      printf("stage 1\n");
      if(picked_ball == 0){
        //picked_ball은 픽업된 파란공의 갯수를 의미, picked_ball == 0은 아직 파란공이 픽업되지 않았다는 뜻으로, 직진을 명령한다.
        go_straight(target_x); //파란공의 x좌표를 이용해 중심을 맞추면서 직진한다.
        printf("go straight : %f \n", target_x);
      }
      else{
        stage_number++;
        //파란공이 픽업되면 stage_number를 증가시켜 다음 stage로 이동한다.
      }
    }
    // ----------- stage 2 : turn ------------
    /*stage 2 의 목표는 다음에 픽업할 파란공의 찾는 한편, 찾은 후에는 파란공을 정면으로 바라보도록 맞추는 것이다.
    무조건 가장 오른쪽에 있는 파란공을 픽업한다는 알고리즘에 의해서 첫번째 파란공을 픽업한 후에는
    왼쪽으로 회전하면서 다음 공을 찾는 것이 가장 회전 시간을 줄일 수 있는 방법이기에,
    전면 카메라 상에서 파란공이 보이지 않을때는 무조건 왼쪽으로 회전하도록 하였다 */
    if(stage_number == 2){
      printf("stage 2\n");
      if(ball_number == 0){ //전면카메라에 파란공이 하나도 보이지 않았을때
        point_turn(CCW); //왼쪽으로 회전
        printf("searching ball\n");
        turn_time += ROOP_RATE; //반시계 방향이므로 turn_time값을 루프주기 만큼 더한다.
      }
      else{ //전면 카메라에 파란공이 하나이상 보일때
        if(abs(target_x) < THRESHOLD_TURN){
          //웹캠을 기준으로 가장 오른쪽에 위치한 파란공의 x좌표가 오차범위내에 있다면 로봇이 픽업할 파란공을 바라보고 있다고 판단한다.
          stage_number++;
        }
        else{ //웹캠을 기준으로 가장 오른쪽에 위치한 파란공의 x좌표가 오차범위내에 없다면, 왼쪽으로 회전한다.
          point_turn(CCW);//왼쪽 회전
          printf("turn left\n");
          turn_time += ROOP_RATE; //시계 방향이므로 turn_time값을 루프주기 만큼 뺀다.
        }
      }
    }
    // ----------- stage 3 : go straight ------------
    if(stage_number == 3){
      /*stage 3는 stage 2에서 찾은 파란공을 픽업할때까지 직진하도록 명령한다 */
      printf("stage 3\n");
      if(picked_ball == 2){
        //picked_ball == 2가 되면 파란공을 픽업했다고 판단, 다음 stage 로 이동시킨다.
        stage_number++;
      }
      else{ //picked_ball이 2가 아니라면, 아직 파란공을 픽업하지 못했다고 판단, 계속 직진한다.
        go_straight(target_x); //가장 오른쪽에 있는 파란공의 x좌표를 이용해 중심을 맞추면서 직진
        printf("go straight : %f \n", target_x);
      }
    }
    // ----------- stage 4 : turn ------------
    if(stage_number == 4){
      /*stage 4 의 목표는 다음에 픽업할 파란공의 찾는 한편, 찾은 후에는 파란공을 웹캠이 정면으로 바라보도록 맞추는 것이다.
      무조건 가장 오른쪽에 있는 파란공을 픽업한다는 알고리즘에 의해서 첫번째 파란공을 픽업한 후에는
      왼쪽으로 회전하면서 다음 공을 찾는 것이 가장 회전 시간을 줄일 수 있는 방법이기에,
      전면 카메라 상에서 파란공이 보이지 않을때는 무조건 왼쪽으로 회전하도록 하였다 */
      /*기능과 코드진행이 stage 2와 동일함으로 주석을 달지 않았습니다.*/
      printf("stage 4\n");
      if(ball_number == 0){
        point_turn(CCW);
        printf("searching ball\n");
        turn_time += ROOP_RATE;
      }
      else{
        if(abs(target_x) < THRESHOLD_TURN){
          stage_number++;
        }
        else{
          point_turn(CCW);
          printf("turn left\n");
          turn_time += ROOP_RATE;
        }
      }
    }
    // ----------- stage 5 : go straight ------------
    if(stage_number == 5){
      /*stage 5의 목표는 마지막 남은 파란공을 픽업할 때까지 직진하고, while문을 나가는 것이다*/
      printf("stage 5\n");
      if(picked_ball == 3){
        //picked_ball == 3 이면 마지막 파란공을 픽업했다고 판단하고 다음 stage로 이동시킨다.
        stage_number++;
        dataInit();
        //다음 구동 명령이 있을 때까지 연산시간이 걸리기 때문에 로봇을 data array(myRIO에게 보내는 데이터)를 초기화 시켜, 로봇을 정지시킨다.
        break;
       }
     }
     else{//공이 아직 픽업되지 않았을경우 공이 픽업될때까지 계속 직진한다
       go_straight(target_x);
       printf("go straight : %f \n", target_x);
     }
   }
	 write(c_socket, data, sizeof(data));//myRIO로 data를 publish 한다.
   ros::Duration(ROOP_RATE).sleep(); //코드가 진행된 후 루프주기 만큼 휴식한다.
 }

 // ------------ find the turn direction --------------


/*밑의 과정은 마지막 공을 픽업한 위치에서 회전할 방향을 결정하기 위한 연산과정이다.
시간을 줄이기 위해서 앞선 과정에서 회전이 있을때마다 루프주기만큼 더하거나 빼왔던 turn_time값을 이용하여,
마지막공을 픽업한 위치에서 바구니로 귀환하기 위해 최소로 회전을 하기 위해서는 어떤 방향으로 돌아야 하는지 결정한다.
while 문 밖에 위치시킨 이유는 연산과정이 계속 피드백 되는 것이 아닌 한번만 이루어져야 했기 때문이다. */

 if(turn_time > 0){//turn_time이 양수라는 것은 로봇이 반시계 방향으로 더 많이 회전했다는 것을 의미한다.
   res = turn_time / (2 *TURN_TIME_180);
    /*turn_time을 360도 회전하는데 걸린 시간으로 나눈다.(TURN_TIME_180은 실험을 통해 결정한 180도 도는데 걸리는 시간)*/
   turn_time = turn_time - (res * 2 * TURN_TIME_180);
   /*로봇이 stage 1~4를 진행하는 과정에서 360*n + degree만큼 돌것이다.
   따라서 res는 여기서 n, 즉 360도를 돈 횟수로, res*2*TURN_TIME_180은 360도를 res번 도는데 필요했던 시간이다.
   따라서 이 값을 turn_time에서 빼면 로봇이 회전한 정도 중 degree에 해당하는 시간이 된다.
   따라서 이 시간이 180도를 회전하는 시간보다 작다는 것은 마지막 위치에서 오른쪽으로 회전했을때 더 적게 회전할 수 있다는 것을 의미하고,
   반대로 이 시간이 180도를 회전하는 시간보다 크다는 것은 마지막 위치에서 왼쪽으로 회전했을때 더 적게 회전할 수 있다는 것을 의미한다 */
   printf("turn time : %f\n", turn_time);
   if(turn_time < TURN_TIME_180){
     turn_direction = CW;
     printf("turn right\n");
   }
   else{
     turn_direction = CCW;
     printf("turn left\n");
   }
 }
 else{
   /*만약에 turn_time이 음수라면, 로봇이 시계 방향으로 더 많이 돌았다는 것을 의미하며,
   일단 이 값을 양수로 만들어 같은 과정을 통해 계산한뒤,
   이번에는 180도를 회전하는 시간보다 작을때 왼쪽, 클때 오른쪽회전하도록 turn_direction에 저장한다. */
   turn_time = -turn_time;
   res = turn_time / (2 * TURN_TIME_180);
   turn_time = turn_time - (res * 2 * TURN_TIME_180);
   printf("turn time : %f\n", -turn_time);
   if(turn_time < TURN_TIME_180){
     turn_direction = CCW;
     printf("turn left\n");
   }
   else{
     turn_direction = CW;
     printf("turn right\n");
   }
 }

 // ------------ return phase start ---------------
 curr_time = 0;
 /*curr_time 변수는 바구니 앞에 위치하었을때 특정시간동안의 후진과 빙출을 명령하는 stage 10과 11에서 사용된다.
 따라서 루프로 들어가기 전에 이를 0으로 초기화 시켰다. */
 while(ros::ok){
   /* 일반 상황에서는 이 while 문 안에서 stage6~11이 진행되는데,
   만약에 파란공을 모두 픽업하지 못한 비상상황에서는 stage 2 혹은 stage 4로 돌아가 이전 stage가 다시 진행된다.*/
   ros::spinOnce(); //콜백 함수 실행
   find_target(); // 가장 오른쪽에 위치한 파란공을 찾는다.


   /*아래 코드는 비상상황을 대배한 코드로, 바구니로 귀환하는 모든 과정(stage 6~11)에서 전면 카메라에 파란공이 보인다면,
   그 갯수에 따라서 이전 stage로 돌아가 다시 파란공을 픽업한다.*/
   if (stage_number > 5){
     if (ball_number == 2){ // 전면 카메라에 파란공이 2개가 보인다면
       /*stage_number가 5초과 일때 전면 카메라에 2개의 파란공이 보인다는 것은 이전 stage에서 파란공을 픽업하는데
       실패했다는 뜻으로 첫번째 파란공을 픽업한 후에 다음 픽업할 파란공을 searching 하는 과정을 수행하는 stage 2로 돌아가
       프로세스를 반복한다.*/
       stage_number = 2; //전역변수인 stage_number에 2를 대입함으로써, 로봇이 손쉽게 이전 stage로 돌아갈 수 있다.
       picked_ball = 1; //전면 카메라에 파란공 2개가 보인다는 것은 픽업한 파란공이 1개라는 의미이므로 전역변수인 picked_ball에 1을 대입한다.
     }
     if (ball_number == 1){
       /*stage_number가 5초과 일때 전면 카메라에 1개의 파란공이 보인다는 것은 이전 stage에서 파란공을 픽업하는데
       실패했다는 뜻으로 두번째 파란공을 픽업한 후에 다음 픽업할 파란공을 searching 하는 과정을 수행하는 stage 4로 돌아가
       프로세스를 반복한다.*/
       stage_number = 4; //전역변수인 stage_number에 4를 대입함으로써, 로봇이 손쉽게 이전 stage로 돌아갈 수 있다.
       picked_ball = 2; //전면 카메라에 파란공 1개가 보인다는 것은 픽업한 파란공이 2개라는 의미이므로 전역변수인 picked_ball에 1을 대입한다.
     }
   }


  /*이전 while 문을 이미 나왔기에 stage 0 에서 5까지의 똑같은 코드를 다시 한번 넣어주었다.
  이전에 언급했으니, 주석은 달지 않았습니다.*/
  /////////////////////////////////////비상상황에서 실행되는 코드////////////////////////////////////////
   // ----------- stage 0 : turn ------------
   if(stage_number == 0){
     printf("stage 0\n");
     if(abs(target_x) < THRESHOLD_TURN){
       stage_number++;
     }
     else{
       if(target_x < 0){
         point_turn(CCW);
         printf("turn left : %f \n", target_x);
         turn_time += ROOP_RATE;
       }
       else{
         point_turn(CW);
         printf("turn right : %f \n", target_x);
         turn_time -= ROOP_RATE;
       }
     }
   }
   // ----------- stage 1 : go straight ------------
   if(stage_number == 1){
     printf("stage 1\n");
     if(picked_ball == 0){
       go_straight(target_x);
       printf("go straight : %f \n", target_x);
     }
     else{
       stage_number++;
     }
   }
   // ----------- stage 2 : turn ------------
   if(stage_number == 2){
     printf("stage 2\n");
     if(ball_number == 0){
       point_turn(CCW);
       printf("searching ball\n");
       turn_time += ROOP_RATE;
     }
     else{
       if(abs(target_x) < THRESHOLD_TURN){
         stage_number++;
       }
       else{
         point_turn(CCW);
         printf("turn left\n");
         turn_time += ROOP_RATE;
       }
     }
   }
   // ----------- stage 3 : go straight ------------
   if(stage_number == 3){
     printf("stage 3\n");
     if(picked_ball == 2){
       stage_number++;
     }
     else{
       go_straight(target_x);
       printf("go straight : %f \n", target_x);
     }
   }
   // ----------- stage 4 : turn ------------
   if(stage_number == 4){
     printf("stage 4\n");
     if(ball_number == 0){
       point_turn(CCW);
       printf("searching ball\n");
       turn_time += ROOP_RATE;
     }
     else{
       if(abs(target_x) < THRESHOLD_TURN){
         stage_number++;
       }
       else{
         point_turn(CCW);
         printf("turn left\n");
         turn_time += ROOP_RATE;
       }
     }
   }
   // ----------- stage 5 : go straight ------------
   if(stage_number == 5){
     printf("stage 5\n");
     if(picked_ball == 3){
       stage_number++;
       dataInit();
       break;
      }
    }
    else{
      go_straight(target_x);
      printf("go straight : %f \n", target_x);
    }
  }
////////////////////////////////////////////////////////////////////////////////


   // ----------- stage 6 : turn ------------
   if(stage_number == 6){
     /*stage 6의 목표는 로봇의 후면 카메라가 두 녹색공의 중심을 바라보게 하는데에 있다.  */
     printf("stage 6\n");
		 if(ball_number_G == 2){
       //후면 카메라에 녹색공 2개가 모두 보일 때
       find_final_position(ball_X_G[0],ball_Y_G[0],ball_X_G[1],ball_Y_G[1]);
       //두 녹색공의 중심의 좌표를 찾는다
			 if(abs(green_ball_center_x) < THRESHOLD_TURN_G){
         //중심의 좌표가 오차범위내에 있다면 로봇의 후면이 중심을 바라보고 있다고 판단, 다음 stage로 넘어간다.
         stage_number++;
         green_ball_sorting();
         //화면에 보이는 녹색공에 대한 무작위적 좌표를 왼쪽에 있는 공부터 ball_X_G, ball_Y_G array 에서 0자리에 오도록 재배열한다.
         prev_ball_X_G = ball_X_G[0];
         /*후면 카메라 기준으로 왼쪽에 보이는 녹색공의 좌표를 저장하는데,
         이때 저장한 값을 이용해서 stage 7에서 바구니 앞으로 정렬될때 돌아야하는 방향을 결정한다. */
			 }
			 else{ //녹색공 중심의 x좌표가 오차범위밖에 있다면 회전을 통해서 오차범위내에 위치하도록 한다.
         if(green_ball_center_x > 0){ //녹색공 중심이 후면 웹캠기준 오른쪽에 있기때문에 오른쪽으로 회전해 두 녹색공 중심을 바라보도록 한다
           point_turn_final(CW); //오른 회전
           printf("turn right\n");
           printf("turn time : %f\n", turn_time);
         }
         else{//녹색공 중심이 후면 웹캠기준 왼쪽에 있기 때문에 왼쪽으로 회전해 두 녹색공 중심을 바라보도록 한다
           point_turn_final(CCW);//왼쪽 회전
           printf("turn left\n");
           printf("turn time : %f\n", turn_time);
         }
       }
     }
     else{
       /*후면 웹캠에 녹색공이 1개 이하로 보일때 녹색공 2개가 모두 보일때까지 정해진 방향으로 회전한다.
       이때 turn_direction은 stage 1-4에서 회전방향에 따라서 계산한 turn_time값을 이용해서 0 혹은 1로 결정이 된다*/
       point_turn_final(turn_direction);
       printf("turn in direction\n");
       printf("turn time : %f\n", turn_time);
		 }
	 }
   // ----------- stage 7 : go back ------------
   if(stage_number == 7){
     /*stage 7의 목표는 바구니에서 150 픽셀만큼 떨어진 지점까지 후진으로 로봇을 이동시키는 데에 있다.*/
     printf("stage 7\n");
     if(ball_number_G == 2){//후면 카메라에 녹색공 두개가 모두 보일때
       green_ball_sorting();
       /*무작위적으로 subscribe된 녹색공의 좌표를 가장 왼쪽에 있는 공부터
       ball_X_G, ball_Y_G array에 0자리에 넣고 나머지를 1자리에 넣어 재배열한다.*/
       prev_ball_X_G = ball_X_G[0]; //후면 웹캠기준 가장 왼쪽에 있는 녹색공의 x좌표를 기억한다.
       find_final_position(ball_X_G[0],ball_Y_G[0],ball_X_G[1],ball_Y_G[1]); //두 녹색공의 중심의 좌표를 구한다.
       if(green_ball_center_y < 150){
         /*바구니 앞에서 150픽셀만큼 떨어진 지점에 도달할때까지 후진하고, 도달했다 판단되면 다음 stage로 이동한다
         이때 green_ball_center_y를 거리가 아닌 픽셀값으로 비교하기 때문에 로봇이 향하는 방향에 상관없이
         바구니 앞에서 150픽셀만큼 떨어진 지점에 도달할 수 있다. */
         stage_number++;
       }
       else{ //바구니 앞에서 150픽셀만큼 떨어진 지점에 도달하지 못했다고 판단, 후진 명령을 한다.
         go_back(green_ball_center_x); //두 녹색공의 중심의 x좌표를 이용해 중심을 맞추면서 후진한다.
         printf("go back : %f \n", green_ball_center_x);
         printf("turn time : %f\n", turn_time);
       }
     }
     else{ //후면카메라에 공이 한개 이하로 보일때
       if(abs(prev_ball_X_G - ball_X_G[0]) < 10){
         /*이전에 저장한 가장 왼쪽에 위치한 녹색공의 좌표와 비교해
         큰 차이가 나지 않는다면 현재 후면 카메라에 보이는 한개의 녹색공이
         가장 왼쪽에 위치한 녹색공이라고 판단 오른쪽으로 회전해 두 녹색공이 모두 보이도록 한다 */
         point_turn_slow(CW);
       }
       else{ /*이전에 저장한 값과 차이가 크면 현재 후면 카메라에 보이는 녹색공이 상대적으로 오른쪽에 위치했던 공이라고 판단,
         왼쪽으로 회전해 두개의 녹색공이 모두 보이도록 한다.*/
         point_turn_slow(CCW);
       }
       prev_ball_X_G = ball_X_G[0];//지금 후면카메라에 가장 왼쪽에 위치한 녹색공의 좌표를 기억
       printf("searching green ball\n");
     }
   }
   // ----------- stage 8 : y align ------------
   if(stage_number == 8){
     /*stage 8의 목적은 로봇이 향하는 방향을 바구니와 평행하게 align하는데에 있다.*/
     printf("stage 8\n");
     if(ball_number_G == 2){ // 후면 카메라에 두개의 녹색공이 모두 보일때
       green_ball_sorting();
       /*무작위적으로 subscribe된 녹색공의 좌표를 가장 왼쪽에 있는 공부터
       ball_X_G, ball_Y_G array에 0자리에 넣고 나머지를 1자리에 넣어 재배열한다.*/
       prev_ball_X_G = ball_X_G[0]; //후면 웹캠기준 가장 왼쪽에 있는 녹색공의 x좌표를 기억한다.
       if(abs(ball_Y_G[0] - ball_Y_G[1]) > THRESHOLD_Y_DIFFERENCE_G){
         /*두 녹색공의 y 픽셀좌표 ball_Y_G[0], ball_Y_G[1]의 차이가 오차범위보다 크다는 것은 로봇이 바구니와 평행하지 않게 있다는 뜻이다*/
         y_align(ball_Y_G[0], ball_Y_G[1]); //로봇이 바구니와 평행하게 바라보도록 한다.
         printf("y_aligning\n");
       }
       else{ //로봇이 바구니와 평행하게 있을때 다음 stage로 이동한다.
         stage_number++;
       }
     }
     else{//후면카메라에 녹색공이 한개 이하로 보일때
       if(abs(prev_ball_X_G - ball_X_G[0]) < 10){
         /*이전에 저장한 가장 왼쪽에 위치한 녹색공의 좌표와 비교해
         큰 차이가 나지 않는다면 현재 후면 카메라에 보이는 한개의 녹색공이
         가장 왼쪽에 위치한 녹색공이라고 판단 오른쪽으로 회전해 두 녹색공이 모두 보이도록 한다 */
         point_turn_slow(CW);
       }
       else{ /*이전에 저장한 값과 차이가 크면 현재 후면 카메라에 보이는 녹색공이 상대적으로 오른쪽에 위치했던 공이라고 판단,
         왼쪽으로 회전해 두개의 녹색공이 모두 보이도록 한다.*/
         point_turn_slow(CCW);
       }
       prev_ball_X_G = ball_X_G[0];//지금 후면카메라에 가장 왼쪽에 위치한 녹색공의 좌표를 기억
       printf("searching green ball\n");
     }
   }
   // ----------- stage 9 : x align ------------
   if(stage_number == 9){
     /*stage 9의 목적은 바구니와 평행하게 바라보고 있는 로봇을 좌우로 이동시켜, 바구니 앞으로 이동시키는데에 있다.*/
     printf("stage 9\n");
     if(ball_number_G == 2){//후면카메라에 녹색공이 모두 보일때
        green_ball_sorting();
        /*무작위적으로 subscribe된 녹색공의 좌표를 가장 왼쪽에 있는 공부터
        ball_X_G, ball_Y_G array에 0자리에 넣고 나머지를 1자리에 넣어 재배열한다.*/
        prev_ball_X_G = ball_X_G[0]; //후면 웹캠기준 가장 왼쪽에 있는 녹색공의 x좌표를 기억한다.
        find_final_position(ball_X_G[0],ball_Y_G[0],ball_X_G[1],ball_Y_G[1]); //두 녹색공의 중심의 좌표를 구한다.
        if(abs(green_ball_center_x - 40) < THRESHOLD_TARGET_G){
          /*두 녹색공의 중심의 x좌표(green_ball_center_x)보다 더 40픽셀만큼 더 이동하도록 하였는데,
          이는 파란공 저장고의 방출문이 후면 웹캠을 기준으로 오른쪽으로 치우쳐있기 때문이다
          후면 웹캠과 바구니를 맞추는 것이 아닌 공이 방출되는 통로와 바구니를 동일선상에 놓기 위함이다*/
          dataInit(); //새로운 명령을 입력하기 위해 data array를 초기화한다.
          stage_number++;
          flag_time = curr_time;
          /*전역변수인 curr_time값을 flag_time에 저장하는데 이는 stage 10에서 특정 시간동안 후진 명령을 줘야하기 때문에
          시간을 측정하기 위해서 저장하는 것이다.*/
        }
        else{ //녹색공 중심의 좌표가 원하는 오차범위 밖에 있을때
          x_align(green_ball_center_x); //공 방출 통로와 녹색공 중심의 좌표가 일직선상에 오도록 좌 또는 우로 이동한다
          printf("x_aligning\n");
        }
      }
      else{//후면카메라에 녹색공이 한개 이하로 보일때
        if(abs(prev_ball_X_G - ball_X_G[0]) < 10){
          /*이전에 저장한 가장 왼쪽에 위치한 녹색공의 좌표와 비교해
          큰 차이가 나지 않는다면 현재 후면 카메라에 보이는 한개의 녹색공이
          가장 왼쪽에 위치한 녹색공이라고 판단 오른쪽으로 회전해 두 녹색공이 모두 보이도록 한다 */
          point_turn_slow(CCW);
        }
        else{/*이전에 저장한 값과 차이가 크면 현재 후면 카메라에 보이는 녹색공이 상대적으로 오른쪽에 위치했던 공이라고 판단,
          왼쪽으로 회전해 두개의 녹색공이 모두 보이도록 한다.*/
          point_turn_slow(CW);
        }
        prev_ball_X_G = ball_X_G[0];//지금 후면카메라에 가장 왼쪽에 위치한 녹색공의 좌표를 기억
        printf("searching green ball\n");
      }
    }
    // ----------- stage 10 : parking ------------
    if(stage_number == 10){
      /*stage 10의 목표는 3.05초동안 후진해서 바구니 앞에 도착하는데에 있다.
      3.05초는 로봇이 바구니로부터 150픽셀만큼 떨어져 있을때 바구니 바로 앞으로 오기위해 후진해야만 하는 시간으로
      실험을 통해서 결정하였다. 루프가 반복될때마다 루프주기 만큼 더해지는 curr_time과 이전 stage 에서 저장한
      flag_time의 값을 비교해 시간이 얼마나 지났는지 판단한다. */
      printf("stage 10\n");
      if((curr_time - flag_time) < 3.05){ //stage 10이 시작되고 3.05초동안 후진을 명령
        data[5] = -0.8; //후진 명령
        printf("parking\n");
      }
      else{ //3.05초간 후진을 한 후 파란공을 방출하기 위해 다음 stage로 넘어간다
        dataInit(); //다음 명령을 위하여 data array 값을 초기화한다.
        stage_number++;
        flag_time = curr_time;
        /*stage 11에서 쓰이기 위해서 flag_time에 curr_time값을 저장한다.
        curr_time값은 while 문이 반복될때마다 루프주기 값만큼 더해지기 때문에
        전역변수인 flag_time과 curr_time값을 비교함으로써 시간이 얼마나 지났는지 계산할 수 있다.*/
      }
    }
    // ----------- stage 11 : release ------------
    if(stage_number == 11){
      /*stage 11 의 목표는 파란공 저장고의 문을 열어 공을 방출시키는데에 있다.
      while roop가 반복될때마다 루프주기 만큼 더해지는 curr_time과 stage 10에서 저장한 flag_time의
      값을 비교해서 3초동안만 저장고 문을 여는 신호를 보낸후 while문을 빠져나가 모든 구동을 끝낸다.*/
      printf("stage 11\n");
      if((curr_time - flag_time) < 3){ //3초간 if문 안의 과정을 실행한다
        release_ball(); //파란공이 저장된 저장고의 문을 연다.
        printf("release\n");
      }
      else{
        dataInit(); //3초간 문을 열고, myRIO로 보내는 데이터를 초기화해 구동과정을 끝낸다
        break;//while 문을 나가고 모든 과정을 마친다.
      }
    }
    ros::Duration(ROOP_RATE_RETURN).sleep(); //루프 주기 만큼 쉰다
    write(c_socket, data, sizeof(data)); //myRIO로 data[24] array를 보낸다.
    curr_time += ROOP_RATE_RETURN; //루프주기만큼 변수 curr_time을 증가시킨다.
  }
  close(c_socket); //socket을 닫는다.
  return 0;
}
