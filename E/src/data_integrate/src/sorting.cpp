
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
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "std_msgs/Int32.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"


#include "opencv2/opencv.hpp"


#define OFFSET 190
#define FRQUENT 5
#define PORT 3000 //독립적인 통신을 위해 새로운 port를 마련하였다.
#define IPADDR "172.16.0.1" // myRIO ip
// #define IPADDR "127.0.0.1" // host computer ip

using namespace std;
int ball_number;
int ball_number_R;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
float ball_X_R[20];
float ball_Y_R[20];
float ball_distance_R[20];
float near_ball_y, near_ball_x;
float near_ball_y_R, near_ball_x_R;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int picked_ball_num=0;
float data[2];
/*myRIO로 보내는 신호 형태로, data[0]자리의 신호만을 사용하는데, 이는 가까운 공의 색깔에 따라서 저장소 앞에
위치한 패널을 회전시키기 위해 사용된다.*/
int past_pick_w = 0;
float past_near_ball = 1000.000;
int picked_ball_num_R=0;
int past_pick_w_R = 0;
float past_near_ball_R = 1000.000;


#define OFFSET2 390
/*본 노드는 픽업 부분 바로 위에 있는 카메라로 부터 정보를 받아 myRIO에게 신호를 보내는 한편,
현재까지 픽업한 파란공의 갯수를 판단해 data_integration_final4_node로 보내는 역할을 한다.
myRIO에게는 data array의 형태로 신호를 보내지만 실제로 제어에 사용되는 부분은 0자리에 위치한 데이터 만이다.
data[0] = 0이면 저장고 앞쪽에 위치한 패널이 공이 파란공 저장소쪽으로 가도록 회전되며,
data[0] = 1이면 저장고 앞쪽에 위치한 패널이 공이 빨간공 저장소쪽으로 가도록 회전된다.
이때 카메라에 빨간 혹은 파란공이 보이지 않는다면 패널은 항상 파란공 공이 저장소쪽으로 가도록 회전된 상태로 유지된다 */

void dataInit(){
  /*data array의 데이터를 모두 0으로 초기화시킨다.
  실제로 사용하는 값은 data[0]뿐이며, data[1]은 sorting_node를 이용한 또 다른 모터 제어가 필요할때를 대비해서 만들어 놓았다*/
  data[0]=0;
  data[1]=0;
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    /*본 함수의 기능은 픽업부 바로 위에 위치한 카메라로 부터 메세지를 subscribe하면,
    sorting_node의 전역변수에 데이터를 저장하는 것이다 */
    int count = position->size; // position 메세지 형태의 size 자리의 값(카메라에 보이는 파란공의 갯수)을 저장한다.
		int count2 = position->size2; // position 메세지 형태의 size2 자리의 값(카메라에 보이는 빨간공의 갯수)을 저장한다.
    ball_number = count; //ball_number는 카메라에 보이는 파란공의 갯수를 의미하는 전역변수로써, 이 변수에 메세지로부터 받은 값을 저장한다.
		ball_number_R = count2;
    //ball_number_R은 카메라에 보이는 빨간공의 갯수를 의미하는 전역변수로써, 이 변수에 메세지로부터 받은 값을 저장한다.

    for(int i = 0; i < count; i++)
    {//for문을 카메라에 보이는 파란공의 갯수 만큼 반복한다.
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        ball_distance[i] = ball_Y[i];
        /*전역변수인 ball_X, ball_Y, ball_distance에 메세지로 부터 받은 값들을 저장한다.
        이때 ball_X는 카메라에 보이는 파란공의 픽셀 x좌표, ball_Y와 ball_distance는 카메라에 보이는 파란공의 픽셀 y좌표이다*/
    }
		for(int i = 0; i < count2; i++)
    {//for문을 카메라에 보이는 빨간공의 갯수 만큼 반복한다.
				ball_X_R[i] = position->img_x2[i];
        ball_Y_R[i] = position->img_y2[i];
		    ball_distance_R[i] = ball_Y_R[i];
        /*전역변수인 ball_X_R, ball_Y_R, ball_distance_R에 메세지로 부터 받은 값들을 저장한다.
        이때 ball_X_R는 카메라에 보이는 빨간공의 픽셀 x좌표, ball_Y_R와 ball_distance_R은 카메라에 보이는 빨간공의 픽셀 y좌표이다*/
    }
}

void ball_distanceInit()
{
  for(int i = 0 ; i < 20 ;i++){
  ball_distance[i]=0;
  ball_distance_R[i]=0;
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sorting_node");//노드명 초기화
    ros::NodeHandle nh;//노드 핸들 선언
    ros::Subscriber sub1 = nh.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    /*토픽 메세지 subscribe  선언, 토픽 명은 position 이고 뮤 사이즈는 1000, 메세지가 subscribe될 때마다 camera_Callback 함수가
    콜백된다. 이때 position 에서는 파란공, 빨간공의 개수와 좌표값을 받는다*/
    ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("picked_ball", 1000);
    /*퍼블리셔 선언, 메세지명은 picked_ball이고 큐 사이즈를 1000으로 한다. std_msgs의 Int32형식을
    사용하였으며, 새로 msg 파일을 만들지 않은 이유는 보낼 데이터가 int형 자료 1개 뿐이어서 새로 메세지 형식을 만드는 것이 불필요하다고
    판단하였다.*/
    std_msgs::Int32 picked_ball; //메세지 선언

    dataInit(); //마이리오로 보내는 데이터 array의 데이터들을 0으로 초기화 한다.
    /*아래 코드는 labVIEW와의 TCPIP통신을 위해 사용된다. ip정보와 port를 입력한다*/
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

		float near_ball, near_ball_R;
    int judge_constant = 0;
    int pick_w = 0;
    int judge_constant_R = 0;
    int pick_w_R = 0;

    while(ros::ok){
			near_ball = 5; 		near_ball_R = 5;
      //두 변수를 5로 초기화 시킨다. 가장 가까운 공을 찾기 위해 쓰이는 변수이기 때문에 초기값이 0을 가지면 안된다.
      ros::spinOnce(); //콜백함수 실행

// -------------------sorting------------------------------
      ball_distanceInit();
      /*픽업부 위에 위치한 웹캠에서 보이는 공의 좌표들을 비교해서 가장 가까이에 위치한 파란공의 픽셀y좌표와
      빨간공의 픽셀 y좌표를 각각 저장한후, 두 값을 비교해서 로봇과 가장 가까이에 위치한 공의 색깔이 무슨색인지 확인한다*/
      for(int i = 0; i < ball_number; i++)
			{ //for문을 파란공의 갯수만큼 돌려서 웹캠에 보이는 모든 파란공의 y좌표를 비교한다.
				if (ball_distance[i] < near_ball){
          //더 가까이에 있는 파란공이 있다면 그 공의 y좌표값을 near_ball에 저장해서 i+1번째 파란공과 다시 비교한다.
					near_ball = ball_distance[i];
				}
			}
			for(int i = 0; i < ball_number_R; i++)
			{ //for문을 빨간공의 갯수만큼 돌려서 웹캠에 보이는 모든 빨간공의 y좌표를 비교한다.
				if (ball_distance_R[i] < near_ball_R){
          //더 가까이에 있는 빨간공이 있다면 그 공의 y좌표값을 near_ball_R에 저장해서 i+1번째 삘긴공과 다시 비교한다.
          near_ball_R = ball_distance_R[i];
        }
			}

      if(near_ball == 0 && near_ball_R == 0){
        //웹캠에 빨간공, 파란공중 그 어떤공도 보이지 않는다면 저장소 앞에 위치한 패널을 파란공 저장소 쪽으로 열어 놓는다.
        data[0] = 0; //파란공 저장소쪽으로 패널을 연다.
      }
			else{
        if(near_ball_R < near_ball){
          //가장 가까이에 위치한 공의 색깔이 빨간색일때
          data[0] = 1; //공 저장고 앞에 위치한 패널을 빨간공 저장소쪽으로 연다

        } // blue ball right
        else{//가장 가까이에 위치한 공의 색깔이 파란색일때
          data[0] = 0; //공 저장고 앞에 위치한 패널을 파란공 저장소쪽으로 연다

        } // red ball left
      }

// ----------------------------picked ball count-----------------------------------
/*아래의 코드는 로봇이 현재까지 픽업한 파란공의 갯수를 count하는 것이 목적으로,
가장 가까운 파란공의 픽셀 y좌표가 일정시간동안 감소하고, 특정한 y좌표이하의 값을 가질때
파란공이 픽업되었다고 판단한다. */

  float near_distance = 100000;
  /*near_distance 값은 가장 가까운 파란공의 픽셀 y좌표값을 저장하는 변수로 1000으로 초기화하는 이유는
  for문에서 p가 0일때 무조건 ball_X[0]값을 near_distance에 저장하기 위해서 이다*/
  for(int p = 0; p < ball_number; p++){
    //웹캠에 보이는 파란공의 갯수만큼 for문을 반복한다.
    if(near_distance > ball_Y[p]){
      /*near_distance는 p-1까지 비교한 ball_Y값중 가장 작은 값을 저장한 것으로, 이 값보다 ball_Y[p]값이 더작다면
      ball_Y[p]값을 near_distance에 새롭게 저장한다.*/
      near_distance = ball_Y[p];
      near_ball_x = ball_X[p];
      near_ball_y = ball_Y[p];
      /*near_ball_x, near_ball_y는 가장 가까이 위치한 파란공의 x,y좌표를 저장하는 변수이다. */
    }
  }

  if(near_ball_y!=0){
    /*웹캠에서 가끔 0값을 잘못보낼때를 대비해서 좌표가 0인 값은 의사결정 과정에서 사용하지 않도록 하였다.*/
    if(ball_number!=0){ //웸캠에 보이는 파란공의 갯수가 0이 아닐때
      if(abs(near_ball_y)<abs(past_near_ball)){
        /*이전 루프에서 저장된 가장 가까운 파란공의 y좌표 값인 past_near_ball과 현재의 가장 가까운 파란공의
        y좌표 값인 near_ball_y를 비교함으로써 파란공의 y좌표가 지속적으로 줄어들었는지 확인한다*/
        if (abs(past_near_ball - near_ball_y) > 2){
          /*이전의 좌표와 비교해 2픽셀이상 줄어들었을때는 judge_constant라는 변수의 값을 증가시키는데,
          이때 2픽셀 이상 할때만 이 변수의 값을 증가시키는 이유는 웹캠에서 보내오는 정보가 공이 같은 위치에 있더라도
          1픽셀이하의 오차로 계속해서 다르게 측정되기 때문에 실제로 공이 가까워지는 것과, 정지해 있으나,
          웹캠의 부정확성으로 인해서 그 좌표값에 차이가 나는 것을 구분하기 위함이다*/
          judge_constant++;
          /*judge_constant는 가장 가까운 파란공의 좌표가 얼마동안 줄어들었는지의 척도가 되는 변수로,
          이 값이 크면 클수록 오랜 시간동안 파란공이 가까워지고 있음을 나타낸다.
          이 변수를 사용하는 이유는 파란공의 좌표가 지속적으로 감소하고 있는 경우와, 한두번 감소하고 증가하는 경우를
          판단하기 위해서다*/
        }
        past_near_ball = near_ball_y; //현재 가장 가까운 파란공의 y좌표를 past_near_ball에 저장해 다음 루프때 비교과정에서 사용한다
      }
    }
  }
  printf("judg_Con= %d\n\n", judge_constant);
  printf("past_near=after  %f\n\n", past_near_ball);

  if (near_ball_y<OFFSET && near_ball_y != 0 ){
    /*가장 가까운 파란공의 y좌표가 OFFSET이하 이고 영이 아닐때 실행되는 코드이다
     OFFSET 값은 실험을 통해 결정하였는데, 여러번의 실험을 통해서 파란공이 픽업되었을때 가장 마지막으로 저장되는
     near_distance값이 최대 190까지 였기에, OFFSET 값은 190으로 저장하였다.
     또한 웹캠이 가끔씩 0.0000과 같은 신뢰할 수 없는 값을 보내는 것이 확인되어 이를
     의사결정 과정에서 배제하기 위해 다음과 같은 조건을 추가하였다*/
    if(judge_constant >= FRQUENT){
      /*파란공의 y좌표가 FREQUENT만큼은 지속적으로 감소했는지 확인하고, 만약에 그렇다면, pick_w 변수에 1을 저장한다.
      이때 이 상수값도 실험을 통해서 결정했는데, 파란공이 픽업되었을때 가질 수 있는 최소의 judge_constant값을 관찰하였고,
      이를 바탕으로 FREQUENT값을 5로 결정하였다*/
      pick_w=1;
    }
  }
  else{ //가장 가까운 파란공의 y좌표가 OFFSET값보다 크거나 웹캠상에서 파란공이 보이지 않을때
    pick_w=0;
    /*pick_w변수에 0을 저장하는데,
    이때 pick_w 변수는 픽업된 파란공의 갯수를 계산하는데 사용하는 변수로서
    평소에는 0값을 가지다가 파란공이 픽업되는 순간에 1이 되었다 다시 0으로 돌아온다
    따라서 pick_w 변수값은 루프가 돌때마다 비교해 0에서 1이 되는 순간에 파란공이 픽업되었다고 판단하였다
    예를 들어
    pick_w : 00000001111000000000011100000
    이면 2개의 공이 픽업되었다고 판단하는 것이다*/
  }
  if(near_ball_y > OFFSET2){
    /*웹캠에서 보이는 가장 가까운 파란공의 좌표가 OFFSET2보다 크면 past_near_ball 값을 1000으로 초기화 한다.
    past_near_ball에는 전 루프에서 가장 가까운 파란공의 y좌표가 저장되기 때문에 공이 픽업되고 난 후에는
    가장 가까운 공의 좌표가 OFFSET2보다 클때 이를 새롭게 접근하는 공이라고 판단하고, 이 새로운 공의 좌표가
    변화하는 것을 확인하기 위해서, 이전의 값을 초기화를 해줘야만 한다.*/
    past_near_ball=1000;
  }

  if (past_pick_w-pick_w<0){
    //이조건을 만족하는 경우는 pick_w가 0에서 1로 변하는 경우 밖에 없으므로 그 순간 공이 픽업 되었다고 판단한다.
    picked_ball_num++; // 픽업된 파란공의 갯수에 해당하는 변수를 증가시킨다.
    picked_ball.data = picked_ball_num; // 메세지의 data 자리에 picked_ball_num 변수값을 저장한다
    judge_constant = 0; //공이 픽업되었으므로, 픽업 여부에 사용되었던 변수인 judge_constant를 초기화한다
    pub1.publish(picked_ball); //메세지를 publish한다. 이 메세지는 integration node가 받는다.

}

  printf("picked_ball_num = %d\n\n", picked_ball_num);

  past_pick_w = pick_w; //현재 루프에서는 pick_w값을 past_pick_w에 저장한다.

  //red ball picked ball
/*픽업된 red ball 의 갯수를 세는 방법은 파란 공의 경우와 동일하기에 따로 주석을 달지 않았습니다.*/

  float near_distance_R = 100000;
  for(int p = 0; p < ball_number_R; p++){
    if(near_distance_R > ball_Y_R[p]){
      near_distance_R = ball_Y_R[p];
      near_ball_x_R = ball_X_R[p];
      near_ball_y_R = ball_Y_R[p];
    }
  }


  if(near_ball_y_R!=0){
    if(ball_number_R!=0){
      if(abs(near_ball_y_R)<abs(past_near_ball_R)){
        if (abs(past_near_ball_R)-abs(near_ball_y_R) > 2){judge_constant_R++;}
        else{}
        past_near_ball_R = near_ball_y_R;
      }
    }
  }


  if (near_ball_y_R<OFFSET){
    if(judge_constant_R >= FRQUENT){
      pick_w_R=1;
    }
  }
  else{
    pick_w_R=0;
  }
  if(ball_number_R==0){past_near_ball_R=1000;}


  if (past_pick_w_R-pick_w_R<0){
    picked_ball_num_R++;
    judge_constant_R = 0;
    /*빨간공이 픽업 되는 순간  while 문을 이용해 지연시간을 주어 저장소 앞의 패널이 빨간공 저장소 쪽으로
    계속 열린채로 유지해, 빨간공이 저장소 안으로 성공적으로 들어가조록 한다.*/
    int k = 0;
    while(k < 1){
    ros::Duration(0.5).sleep();
    k++;
    }


  }

  past_pick_w_R = pick_w_R;


  write(c_socket, data, sizeof(data)); //myRIO에게 data array 형태로 신호를 보낸다.
  printf("data is = %f\n", data[0]);

  ros::Duration(0.025).sleep(); //루프 주기만큼 쉰다
}
  return 0;
}
