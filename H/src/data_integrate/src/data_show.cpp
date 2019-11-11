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

int ball_number;
float ball_rX[20];
float ball_rY[20];
float ball_bX[20];
float ball_bY[20];

boost::mutex map_mutex;


#define RAD2DEG(x) ((x)*180./M_PI)

bool check_point_range(int cx, int cy)
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}



void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    map_mutex.lock();
    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_rX[i] = position->img_rx[i];
        ball_rY[i]=position->img_ry[i];
        ball_bX[i] = position->img_bx[i];
        ball_bY[i]=position->img_by[i];
    }
    map_mutex.unlock();
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
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);

    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
        // Drawing Lidar data
        float obstacle_x, obstacle_y;
        int crx, cry,cbx, cby;
        int crx1, crx2, cry1, cry2,cbx1, cbx2, cby1, cby2;
        for(int i = 0; i < lidar_size; i++)
        {
            obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
            obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);
            crx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
            cry = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
            crx1 = crx-OBSTACLE_PADDING;
            cry1 = cry-OBSTACLE_PADDING;
            crx2 = crx+OBSTACLE_PADDING;
            cry2 = cry+OBSTACLE_PADDING;
            cbx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
            cby = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
            cbx1 = cbx-OBSTACLE_PADDING;
            cby1 = cby-OBSTACLE_PADDING;
            cbx2 = cbx+OBSTACLE_PADDING;
            cby2 = cby+OBSTACLE_PADDING;

            if(check_point_range(cbx,cby) && check_point_range(cbx1,cby1) && check_point_range(cbx2,cby2))
            {
                cv::line(map, cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),cv::Point(cbx, cby),cv::Scalar(63,63,0), 1);
                cv::rectangle(map,cv::Point(cbx1, cby1),cv::Point(cbx2, cby2), cv::Scalar(255,255,0), -1);
            }
        }
        // Drawing ball
        for(int i = 0; i < ball_number; i++)
        {
            crx = MAP_WIDTH/2 + (int)(ball_rX[i]/MAP_RESOL);
            cbx = MAP_WIDTH/2 + (int)(ball_bX[i]/MAP_RESOL);
            cry = MAP_HEIGHT/2 - (int)(ball_rY[i]/MAP_RESOL);
            cby = MAP_HEIGHT/2 - (int)(ball_bY[i]/MAP_RESOL);
            crx1 = crx-OBSTACLE_PADDING*2;
            cry1 = cry-OBSTACLE_PADDING*2;
            cbx1 = cbx-OBSTACLE_PADDING*2;
            cby1 = cby-OBSTACLE_PADDING*2;
            crx2 = crx+OBSTACLE_PADDING*2;
            cry2 = cry+OBSTACLE_PADDING*2;
            cbx2 = cbx+OBSTACLE_PADDING*2;
            cby2 = cby+OBSTACLE_PADDING*2;

            if(check_point_range(cbx,cby) && check_point_range(cbx1,cby1) && check_point_range(cbx2,cby2))
            {
                cv::rectangle(map,cv::Point(cbx1, cby1),cv::Point(cbx2, cby2), cv::Scalar(0,0,255), -1);
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
