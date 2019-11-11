#include <iostream>
#include <ctime>
#include <math.h>
#include <string.h>

#define PI 3.14159265

using namespace std;

int carx = 30, cary = 40;  // car size
int saferange = 100;

double tcost[6] = {0, };

double t_max = 100000;
double straight = 20;   // cm/s
double rotate = 10;     // degree/s
double diagonal = 10;   // cm/s
double stood = 1;
double dtod = 3;

double base_x = 0, base_y = 0;

int size_r=3, size_b=3, size_g=2;
double b_ball_x[3] ={0, }, r_ball_x[3] ={0, }, g_ball_x[2] ={0, };
double b_ball_y[3] ={0, }, r_ball_y[3] ={0, }, g_ball_y[2] ={0, };

double angle1 = 0; //base to b1;
double angle2 = 0; //b1 to b2;
double angle3 = 0; //b2 to b3;
double angle4 = 0; //b3 to base;
double angle5 = 0; //base to goal;

double prep = 0;

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
	double dis = mindis(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_x[b2]);
	if(dis < saferange) { return true; }

	dis = mindis(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b3], b_ball_x[b3]);
	if(dis < saferange) { return true; }

	dis = mindis(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_x[b2], b_ball_x[b3], b_ball_x[b3]);
	if(dis < saferange) { return true; }

	return false;
}

double angle(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double in = inner(x2-x1, y2-y1, x3-x2, y3-y2);
	double dis12 = pdis(x1, y1, x2, y2);
	double dis23 = pdis(x2, y2, x3, y3);
	double theta = acos(in/dis12/dis23)*180/PI;
	return theta;
}

double pathgen(int b1, int b2, int b3)
{
	//cout << "pathgen_start" << endl;
	double t = 0;

	// if(b_ball_in_path(b1, b2, b3))
	// {	t = t_max;
	// 	return t;	}

	t+=pdis(base_x, base_y, b_ball_x[b1], b_ball_y[b1])/straight;
	t+=angle(base_x, base_y-10, base_x, base_y, b_ball_x[b1], b_ball_y[b1])/rotate;
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
	t+=angle(base_x, base_y, b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2])/rotate;
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
	t+=angle(b_ball_x[b1], b_ball_y[b1], b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3])/rotate;
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
	t+=angle(b_ball_x[b2], b_ball_y[b2], b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep)/rotate;
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

	t+=angle(b_ball_x[b3], b_ball_y[b3], base_x, base_y+prep, base_x, base_y-10)/rotate;

	return t;

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

int main(int argc, char** argv) {
	//cout << angle(0,-10,0,0,1,1) <<endl;
	b_ball_x[0] = -60; b_ball_x[1] = 5;   b_ball_x[2] = 80;
	b_ball_y[0] = 380; b_ball_y[1] = 290; b_ball_y[2] = 350;

	r_ball_x[0] = -75; r_ball_x[1] = 0;   r_ball_x[2] = 75;
	r_ball_y[0] = 280; r_ball_y[1] = 370; r_ball_y[2] = 280;

  cout << "ball position (x, y) [cm]" << endl;
	cout << "blue 0 = ( "<< b_ball_x[0] << ", " << b_ball_y[0] << " )" <<endl;
	cout << "blue 1 = ( "<< b_ball_x[1] << ", " << b_ball_y[1] << " )" <<endl;
	cout << "blue 2 = ( "<< b_ball_x[2] << ", " << b_ball_y[2] << " )" <<endl;

	cout << "red  0 = ( "<< r_ball_x[0] << ", " << r_ball_y[0] << " )" <<endl;
	cout << "red  1 = ( "<< r_ball_x[1] << ", " << r_ball_y[1] << " )" <<endl;
	cout << "red  2 = ( "<< r_ball_x[2] << ", " << r_ball_y[2] << " )" <<endl;


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

  cout << endl;
	cout << "angle1 : " << angle1 << endl;
	cout << "angle2 : " << angle2 << endl;
	cout << "angle3 : " << angle3 << endl;
	cout << "angle4 : " << angle4 << endl;
	cout << "angle5 : " << angle5 << endl;


	return 0;
}
