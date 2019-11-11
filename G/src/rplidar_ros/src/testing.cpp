#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
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
#include <ros/ros.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)
int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_zero_degree;
int zero_degree;
int least_distance;

void go_straight_lidar(float distance){
  ros::spinOnce();
  float initial_distance = lidar_zero_degree;
  float moved_distance = initial_distance-lidar_zero_degree;
  while(moved_distance < distance){
    ros::spinOnce();
    moved_distance = initial_distance-lidar_zero_degree;
    ROS_INFO("Moved : %f",moved_distance);
    ros::Duration(0.025).sleep();
  }
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    lidar_size=count;
    zero_degree = 0;
    least_distance = 0;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(static_cast<int>(lidar_degree[i]+0.5) == 0){zero_degree = i;}
        lidar_distance[i]=scan->ranges[i];
        if(lidar_distance[i]<lidar_distance[least_distance]){least_distance = i;}
    }
    if(isinf(scan->ranges[zero_degree])){
      lidar_zero_degree = lidar_zero_degree;
    }
    else{
      lidar_zero_degree = scan->ranges[zero_degree];
    }
    ROS_INFO("Distance : %f",lidar_zero_degree);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "lidar_test");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
  // ros::spin();
  ros::Duration(1).sleep();
  go_straight_lidar(1000);
}
