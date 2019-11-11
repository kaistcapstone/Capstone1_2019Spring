#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

extern "C" {
	#include "xbox_ctrl/gamepad.h"
}

using namespace std;

#define PORT 4000
#define IPADDR "172.16.0.1"
//#define IPADDR "127.0.0.1"
#define resol_x 640
#define resol_y 480

#define RANGE_DEAD1 50
#define RANGE_STATE_CHANGE 300

static const char* button_names[] = {
	"d-pad up",
	"d-pad down",
	"d-pad left",
	"d-pad right",
	"start",
	"back",
	"left thumb",
	"right thumb",
	"left shoulder",
	"right shoulder",
	"???",
	"???",
	"A",
	"B",
	"X",
	"Y"
};

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];

static int line = 0;

static void logevent(const char* format, ...) {
	va_list va;

	move(9 + line, 0);
	clrtoeol();

	move(8 + line, 0);
	clrtoeol();

	va_start(va, format);
	vwprintw(stdscr, format, va);
	va_end(va);

	if (++line == 14) {
		line = 0;
	}
}


static void update(GAMEPAD_DEVICE dev) {

	float lx, ly, rx, ry;
	move(dev, 0);

	if (!GamepadIsConnected(dev)) {
		printw("%d) n/a\n", dev);
		return;
	}

	GamepadStickNormXY(dev, STICK_LEFT, &lx, &ly);
	GamepadStickNormXY(dev, STICK_RIGHT, &rx, &ry);

	printw("%d) L:(%+.3f,%+.3f :: %+.3f,%+.3f) R:(%+.	3f, %+.3f :: %+.3f,%+.3f) LT:%+.3f RT:%+.3f ",
		dev,
		lx, ly,
		GamepadStickAngle(dev, STICK_LEFT),
		GamepadStickLength(dev, STICK_LEFT),
		rx, ry,
		GamepadStickAngle(dev, STICK_RIGHT),
		GamepadStickLength(dev, STICK_RIGHT),
		GamepadTriggerLength(dev, TRIGGER_LEFT),
		GamepadTriggerLength(dev, TRIGGER_RIGHT));
	printw("U:%d D:%d L:%d R:%d ",
		GamepadButtonDown(dev, BUTTON_DPAD_UP),
		GamepadButtonDown(dev, BUTTON_DPAD_DOWN),
		GamepadButtonDown(dev, BUTTON_DPAD_LEFT),
		GamepadButtonDown(dev, BUTTON_DPAD_RIGHT));
	printw("A:%d B:%d X:%d Y:%d Bk:%d St:%d ",
		GamepadButtonDown(dev, BUTTON_A),
		GamepadButtonDown(dev, BUTTON_B),
		GamepadButtonDown(dev, BUTTON_X),
		GamepadButtonDown(dev, BUTTON_Y),
		GamepadButtonDown(dev, BUTTON_BACK),
		GamepadButtonDown(dev, BUTTON_START));
	printw("LB:%d RB:%d LS:%d RS:%d\n",
		GamepadButtonDown(dev, BUTTON_LEFT_SHOULDER),
		GamepadButtonDown(dev, BUTTON_RIGHT_SHOULDER),
		GamepadButtonDown(dev, BUTTON_LEFT_THUMB),
		GamepadButtonDown(dev, BUTTON_RIGHT_THUMB));
}
uint8_t tchar = 0xFF;
//00000000 00000000 00000000 11111111 (0xFF)


bool flag_auto = false;
float c1y = 0.6;
float c1z = atan2(270,200);

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

int flag_get_ball = 0;

void autoBallTrack(const geometry_msgs::Vector3& msg)
{
	int x = (int)msg.x;
	int dy = (int)msg.y;
	int dx;
	float v_x, v_z, v_y, p_l;

	if(x==-1)
	{
		dataInit();
		data[8] = 0.5;
		write(c_socket, data, sizeof(data));
		return;
	}

	flag_get_ball++;

	if(flag_get_ball >= 5)
	{
		dataInit();
		data[14] = 0;
		write(c_socket, data, sizeof(data));
		flag_get_ball = 0;
		return;
	}

	dataInit();

	if(dy < RANGE_STATE_CHANGE)
	{
		if( (x>(resol_x/2-RANGE_DEAD1)) && (x<(resol_x/2+RANGE_DEAD1)) )
		{
			dx = 0;
			v_x =0;
			data[1] = 1.0;
		}
		else if(x>resol_x/2)
		{
			dx = max(0, x-resol_x/2-RANGE_DEAD1);
			v_x = 0.7*min(max(-1+dx/(60-0.1*dy),-0.5),0.0);
			// v_x = min(-1+dx/(60-0.1*dy),0.0);
			v_y = c1y+(RANGE_STATE_CHANGE-dy)/RANGE_STATE_CHANGE*0.4;
			data[1] = v_y;
		}
		else
		{
			dx = min(0, x-resol_x/2+RANGE_DEAD1);
			v_x = 0.7*max(min(1+dx/(60-0.1*dy),0.5),0.0);
			// v_x = max(1+dx/(60-0.1*dy),0.0);
			v_y = c1y+(RANGE_STATE_CHANGE-dy)/RANGE_STATE_CHANGE*0.4;
			data[1] = v_y;
		}

		v_z = 0.6*min(0.85*abs(atan2(dx,dy))/c1z+0.15,1.0);
		data[0] = v_x;
		if(x<resol_x/2) data[8] = v_z;
		else data[9] = v_z;
	}
	else
	{
		dy = dy - RANGE_STATE_CHANGE;
		if(x>resol_x/2)
		{
			dx = x-resol_x/2;
			if(dx>dy)
			{
				data[0] = 0.5;
				data[1] = 0.5/dx*dy;
			}
			else
			{
				data[0] = 0.5/dy*dx;
				data[1] = 0.5;
			}
		}
		else
		{
			dx = resol_x/2-x;
			if(dx>dy)
			{
				data[0] = -0.5;
				data[1] = 0.5/dx*dy;
			}
			else
			{
				data[0] = -0.5/dy*dx;
				data[1] = 0.5;
			}
		}
		data[4] = 1.2*(x-resol_x/2)/(resol_x/2);
		data[14] = 1;
		flag_get_ball = 0;
	}
	write(c_socket, data, sizeof(data));
	cout << "send1" << endl;
	ros::Duration(0.5).sleep();
	dataInit();
	write(c_socket, data, sizeof(data));
	cout << "send2" << endl;
	ros::Duration(0.05).sleep();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "XboxCtrl");
    ros::start();

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/realsense/biggest_ball", 1, autoBallTrack);

	int ch, i, j, k;
	float lx, ly, rx, ry;
	initscr();//ncurse, The initscr() function determines the terminal type and initialises all implementation data structures.
//initscr() 함수는 curses mode 로 터미널을 초기화 한다. 몇몇 구현에서는 화면을 클리어하고 빈 화면을 보여주기도 한다. curses 패키지를 이용해서 스크린 처리를 할려면 이 함수를 반드시 먼저 콜해야 한다. 이 함수는 curses 시스템을 초기화 하고 'stdscr' 이라 불리는 우리의 현재 윈도우와 몇몇 데이터들-구조체- 들을 위한 메모리를 할당한다. 아주 극한 상황에서나 이 함수는 curses 라이브러리의 데이터 구조체를 위한 메모리를 할당실패로 에러는 낼 것이다. 	
	cbreak();// ncurse, CBREAK가 on일 때, 읽기로 부터의 입력은 즉시 프로그램에서 사용가능하고 off일 때, 입력은 newline이 발생할 때까지 버퍼에 저장될 것이다. 
	noecho();// ncurse
	timeout(1);//

	GamepadInit();

	c_socket = socket(PF_INET, SOCK_STREAM, 0);
//PF_INET 을 넣어주면 IPV4 타입을 사용한다는 것이다.
//SOCK_STREAM 을 넣어주면 연결지향형 소켓을 만들겠다는 의미이다.
//세번째 인자는 protocol이 들어간다. protocol은 통신규약을 말한다.
//IPPROTO_TCP 는 TCP를 사용하겠다고 지정해주는것이다.

	printw("socket created\n");
	c_addr.sin_addr.s_addr = inet_addr(IPADDR);
	c_addr.sin_family = AF_INET;
//인자들에 대한 설명중 sin_family 는 반드시 AF_INET 이어야 함을 알 수 있다.
	c_addr.sin_port = htons(PORT);
//네트워크 표준은 빅엔디안을 활용한다.htons : host to network short 의 약자다. 이 함수를 거치면 무조건 빅엔디안 방식으로 데이터를 변환하여 설정한다.

	if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
//sizeof = byte 크기를 알려줌
		printf("Failed to connect\n");
		close(c_socket);
		return -1;
	}
//https://kevinthegrey.tistory.com/26

	while ((ch = getch()) != 'q') {
		GamepadUpdate();

		double dev = GAMEPAD_0;
		GAMEPAD_DEVICE _dev = static_cast<GAMEPAD_DEVICE>(dev);
		GamepadStickNormXY(_dev, STICK_LEFT, &lx, &ly);
		GamepadStickNormXY(_dev, STICK_RIGHT, &rx, &ry);

		data[2] = GamepadStickAngle(_dev, STICK_LEFT);
		data[3] = GamepadStickLength(_dev, STICK_LEFT);
		data[6] = GamepadStickAngle(_dev, STICK_RIGHT);
		data[7] = GamepadStickLength(_dev, STICK_RIGHT);
		data[10] = GamepadButtonDown(_dev, BUTTON_DPAD_UP);
		data[11] = GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
		data[12] = GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
		data[13] = GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
		data[15] = GamepadButtonDown(_dev, BUTTON_B);
		data[16] = GamepadButtonDown(_dev, BUTTON_X);
		data[17] = GamepadButtonDown(_dev, BUTTON_Y);
		data[18] = GamepadButtonDown(_dev, BUTTON_BACK);
		data[19] = GamepadButtonDown(_dev, BUTTON_START);
		data[20] = GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
		data[21] = GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
		data[22] = GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
		data[23] = GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);

		// Auto Moving
		if(data[19] > 0.9)
			flag_auto = !flag_auto;

		if(!flag_auto)
		{
			data[0] = lx*data[3];
			data[1] = ly*data[3];
			data[4] = rx*data[7];
			data[5] = ry*data[7];
			data[8] = GamepadTriggerLength(_dev, TRIGGER_LEFT);
			data[9] = GamepadTriggerLength(_dev, TRIGGER_RIGHT);
			data[14] = GamepadButtonDown(_dev, BUTTON_A); // duct on/off
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}
		else
		{
			for(i=0;i<20;i++){
			data[5]=1;
			data[14]=1;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
			}
			for(i=0;i<20;i++){
			data[5]=1;
			data[14]=0;
			data[17]=1;
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
			}
			flag_auto = !flag_auto;
		}

		if (ch == 'r') {
			for (i = 0; i != GAMEPAD_COUNT; ++i) {
				GamepadSetRumble(static_cast<GAMEPAD_DEVICE>(i), 0.25f, 0.25f);
			}
		}

		update(GAMEPAD_0);
		//update(GAMEPAD_1);
		//update(GAMEPAD_2);
		//update(GAMEPAD_3);

		//for (i = 0; i != GAMEPAD_COUNT; ++i)
		{
			GAMEPAD_DEVICE _i = static_cast<GAMEPAD_DEVICE>(0);
			if (GamepadIsConnected(_i)) {
				for (j = 0; j != BUTTON_COUNT; ++j) {
					GAMEPAD_BUTTON _j = static_cast<GAMEPAD_BUTTON>(j);
					if (GamepadButtonTriggered(_i, _j)) {
						logevent("[%d] button triggered: %s", i, button_names[j]);
					} else if (GamepadButtonReleased(_i, _j)) {
						logevent("[%d] button released:  %s", i, button_names[j]);
					}
				}
				for (j = 0; j != TRIGGER_COUNT; ++j) {
					GAMEPAD_TRIGGER _j = static_cast<GAMEPAD_TRIGGER>(j);
					if (GamepadTriggerTriggered(_i, _j)) {
						logevent("[%d] trigger pressed:  %d", i, j);
					} else if (GamepadTriggerReleased(_i, _j)) {
						logevent("[%d] trigger released: %d", i, j);
					}
				}
				for (j = 0; j != STICK_COUNT; ++j) {
					GAMEPAD_STICK _j = static_cast<GAMEPAD_STICK>(j);
					for (k = 0; k != STICKDIR_COUNT; ++k) {
						GAMEPAD_STICKDIR _k = static_cast<GAMEPAD_STICKDIR>(k);
						if (GamepadStickDirTriggered(_i, _j, _k)) {
							logevent("[%d] stick direction:  %d -> %d", i, j, k);
						}
					}
				}
			}
		}

		move(6, 0);
		printw("(q)uit (r)umble");

		refresh();
	}
	close(c_socket);

	ros::shutdown();

	endwin();

	return 0;
}
