#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ball_position.h"

#include "opencv2/opencv.hpp"

float MAP_CX = 200.5;
float MAP_CY = 200.5;
float MAP_RESOL = 0.05;             // Map resoultion [cm]
int MAP_WIDTH = 400;
int MAP_HEIGHT = 400;
int MAP_CENTER = 50;
int OBSTACLE_PADDING = 2;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int init_ball;
int init_lidar;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];


int ball_number_1_red;
int ball_number_1_blue;
int ball_number_1_green;
float ball_X_1_red[20];
float ball_Y_1_red[20];
float ball_X_1_blue[20];
float ball_Y_1_blue[20];
float ball_X_1_green[20];
float ball_Y_1_green[20];

int ball_number_2_red;
int ball_number_2_blue;
int ball_number_2_green;
float ball_X_2_red[20];
float ball_Y_2_red[20];
float ball_X_2_blue[20];
float ball_Y_2_blue[20];
float ball_X_2_green[20];
float ball_Y_2_green[20];

int ball_number_3_red;
int ball_number_3_blue;
int ball_number_3_green;
float ball_X_3_red[20];
float ball_Y_3_red[20];
float ball_X_3_blue[20];
float ball_Y_3_blue[20];
float ball_X_3_green[20];
float ball_Y_3_green[20];

boost::mutex map_mutex;


#define RAD2DEG(x) ((x)*180./M_PI)

bool check_point_range(int cx, int cy)
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}



void camera1_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  map_mutex.lock();
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
  }
  for(int i = 0; i < count_blue; i++)
  {
    ball_X_1_blue[i] = position->img_x_blue[i];
    ball_Y_1_blue[i] = position->img_y_blue[i];
  }
  for(int i = 0; i < count_green; i++)
  {
    ball_X_1_green[i] = position->img_x_green[i];
    ball_Y_1_green[i] = position->img_y_green[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl
  }
  map_mutex.lock();
}

void camera2_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  map_mutex.lock();
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
  }
  for(int i = 0; i < count_blue; i++)
  {
    ball_X_2_blue[i] = position->img_x_blue[i];
    ball_Y_2_blue[i] = position->img_y_blue[i];
  }
  for(int i = 0; i < count_green; i++)
  {
    ball_X_2_green[i] = position->img_x_green[i];
    ball_Y_2_green[i] = position->img_y_green[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl
  }
  map_mutex.lock();
}

void camera3_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  map_mutex.lock();
  int count_red = position -> size_red;
  int count_blue = position -> size_blue;
  int count_green = position -> size_green;

  ball_number_3_red=position->size_red;
  ball_number_3_blue=position->size_blue;
  ball_number_3_green=position->size_green;
  for(int i = 0; i < count_red; i++)
  {
    ball_X_3_red[i] = position->img_x_red[i];
    ball_Y_3_red[i] = position->img_y_red[i];
  }
  for(int i = 0; i < count_blue; i++)
  {
    ball_X_3_blue[i] = position->img_x_blue[i];
    ball_Y_3_blue[i] = position->img_y_blue[i];
  }
  for(int i = 0; i < count_green; i++)
  {
    ball_X_3_green[i] = position->img_x_green[i];
    ball_Y_3_green[i] = position->img_y_green[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl
  }
  map_mutex.lock();
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    map_mutex.lock();
    int count = scan->scan_time / scan->time_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
    }
    map_mutex.unlock();

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera1_Callback);
    ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera2_Callback);
    ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera3_Callback);

    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
        // Drawing Lidar data
        float obstacle_x, obstacle_y;
        int cx, cy;
        int cx1, cx2, cy1, cy2;
        for(int i = 0; i < lidar_size; i++)
        {
            obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
            obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);
            cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
            cy = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
            cx1 = cx-OBSTACLE_PADDING;
            cy1 = cy-OBSTACLE_PADDING;
            cx2 = cx+OBSTACLE_PADDING;
            cy2 = cy+OBSTACLE_PADDING;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::line(map, cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),cv::Point(cx, cy),cv::Scalar(63,63,0), 1);
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(255,255,0), -1);
            }
        }
        // Drawing ball
        for(int i = 0; i < ball_number_1_blue; i++)
        {
            cx = MAP_WIDTH/2 + (int)(ball_X_1_blue[i]/MAP_RESOL);
            cy = MAP_HEIGHT/2 - (int)(ball_Y_1_blue[i]/MAP_RESOL);
            cx1 = cx-OBSTACLE_PADDING*2;
            cy1 = cy-OBSTACLE_PADDING*2;
            cx2 = cx+OBSTACLE_PADDING*2;
            cy2 = cy+OBSTACLE_PADDING*2;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(0,0,255), -1);
            }
        }
        // Drawing ROBOT
        cv::circle(map,cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),3,cv::Scalar(255,0,0),-1);

        cv::imshow("Frame",map);
        cv::waitKey(50);

        if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
            return 0;
        }
        ros::spinOnce();
    }



    return 0;
}
