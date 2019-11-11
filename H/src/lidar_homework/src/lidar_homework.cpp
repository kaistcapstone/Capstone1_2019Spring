#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>

#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

// Global variable
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new::pcl::PointCloud<pcl::PointXYZ>); // prev_cloud msg 선언 to Euclidean xyz type variable
Eigen::Matrix4f transMtx_now;     //transformation matrix들을 Matrix4f의 형식으로 선언
Eigen::Matrix4f transMtx_prev;
Eigen::Matrix4f transMtx_delta;

double pos[3];// x position, y position, theta information

void lidar_cb(sensor_msgs::LaserScan msg){ //call back function

    // angle in radian
    float angle_min = msg.angle_min;            //subscribe한 메세지 variable들을 가공하기 위해 새로 변수 선언
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;      //detect된 position까지의 거리들로 구성된 벡터 array

    // size of range vector
    int len = range.size();
    float angle_temp;

    /// 1. LaserScan msg to PCL::PointXYZ

    // initializae pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new::pcl::PointCloud<pcl::PointXYZ>);// new_cloud msg 선언 to Euclidean xyz type variable
    pcl::ExtractIndices<pcl::PointXYZ> extract; //extracts a set of indices from a point cloud.
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());// inf_points msg 선언

    new_cloud->is_dense = false;//message에 있는 variable에 값 저장.
    new_cloud->width = len;
    new_cloud->height = 1;
    new_cloud->points.resize(len);

    // fill the pointcloud
    for(int i = 0; i < len; i++){

        // TO DO START
        angle_temp=angle_min+i*angle_increment; //i번째 라이더 정보의 각도. 즉 현재 range를 측정중인 지점의 각도를 뜻함.

        // TO DO END

        if (std::isinf(range[i])==false){ //i번째 라이더의 range 정보가 무한이 아닐때,

            // TO DO START

            new_cloud->points[i].x = -range[i]*cos(angle_temp);//극 좌표 정보를 직교좌표로 변환. angle은 -x축에서 y축 방향으로 회전할 때 +.
            new_cloud->points[i].y = range[i]*sin(angle_temp);
            new_cloud->points[i].z = 0;//변하지 않는 상수 값이라 0으로 넣어줌.

            // TO DO END

        }
        else{ //라이더의 range정보가 무한일 때,
            // indices of infinite distance points
            inf_points->indices.push_back(i);//extract 해주기 위해 한번에 모아서 제거하기 위해 indices 메세지 variable에 추가.

        }
    }

    // Remove infinite distance points from new_cloud
    extract.setInputCloud(new_cloud); //Provide a pointer to the input dataset.
    extract.setIndices(inf_points);//Provide a pointer to the vector of indices that represents the input data.
    extract.setNegative(true);//Set whether the indices should be returned, or all points _except_ the indices.
    extract.filter(*new_cloud);//Calls the filtering method and returns the filtered dataset in new_cloud.

    // 2. Get transformation between previous pointcloud and current pointcloud

    // transMtx_prev : transformation matrix at time (t-1)
    // transMtx_now : transformation matrix at time (t)
    // 4X4 transformation matrix (3X3: rotation matrix, 3X1: translation vector)

    if(prev_cloud->width == 0){

        // initialize transformation matrix. initial posiiton: x = 0, y = 0, theta = 0;
        transMtx_prev << cos(0), -sin(0), 0, 0,
                        sin(0), cos(0), 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
    }

    else{

        // ICP algorithm
        // http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
        // http://pointclouds.org/documentation/tutorials/interactive_icp.php#interactive-icp
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //creates an instance of an IterativeClosestPoint .
        icp.setInputSource(prev_cloud); // sets prev_cloud as the PointCloud to begin from.
        icp.setInputTarget(new_cloud);  // sets new_cloud as the PointCloud which we want prev_cloud to look like.
        pcl::PointCloud<pcl::PointXYZ> Final;//Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm.
        icp.align(Final); //Perform alignment, transformation matrix 예측해서 내보냄(registration).
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<    //If the two PointClouds align correctly, then icp.hasConverged() = 1 (true).
        icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;//It then outputs the fitness score of the final transformation and some information about it.

        transMtx_delta = icp.getFinalTransformation(); //얻어낸 변화된 final transformation matrix를 transMtx_delta에 담는다.

    // 3. Get current transformation matrix using previous transformation and ICP result
    //  (Odometry calculation)

        //TO DO START -- transformation matrix 곱하기로 현재 표현
        transMtx_delta(0,0)=round(transMtx_delta(0,0)*10000)/10000;
        transMtx_delta(0,1)=round(transMtx_delta(0,1)*10000)/10000;
        transMtx_delta(1,0)=round(transMtx_delta(1,0)*10000)/10000;
        transMtx_delta(1,1)=round(transMtx_delta(1,1)*10000)/10000;
        transMtx_delta(0,3)=round(transMtx_delta(0,3)*10000)/10000;
        transMtx_delta(1,3)=round(transMtx_delta(1,3)*10000)/10000;
        transMtx_delta(2,3)=round(transMtx_delta(2,3)*10000)/10000;





        transMtx_now = transMtx_prev * transMtx_delta; // time t에서의 transformation matrix는 t-1 때와 그 사이 delta만큼의 matrix의 곱과 같다.
        std::cout<<transMtx_delta(0,0)<<std::endl;
        // TO DO END

    // 4. Get current position from transformation matrix

        // TO DO START

        pos[0] = transMtx_now(0,3); // 현재의 transformation matrix의 1행 4열 요소가 x position을 의미하고, 2행 4열 요소가 y position을 의미한다.
        pos[1] = transMtx_now(1,3);
        if(transMtx_now(1,0)>=0){   // 1행 1열 요소가 cos(theta) 정보를 갖는데, 함수 acos가 항상 양의 값을 리턴해 sin(theta)의 음양을 통해 theta의 올바른 부호를 결정.
          pos[2] = abs(acos(transMtx_now(0,0)));
        }else{
          pos[2] = -abs(acos(transMtx_now(0,0)));
        }

        // TO DO END

        transMtx_prev = transMtx_now; // Save current transformation matrix in transMtx_prev

    }
    // 5. Save new_cloud in prev_cloud

    prev_cloud = new_cloud;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_homework_node"); //node 선언
    ros::NodeHandle nh;

    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb); //subscriber 선언, topic, 큐, 콜백함수

    ros::Rate loop_rate(5);

    // initialize prev_cloud
    prev_cloud->width = 0;

    // File write

    // TO DO START
    string filePath = "/home/hjexpress/Desktop/hw4_result.txt"; //<- Change to your own directory
    // TO DO END

    ofstream txtFile(filePath); //파일 쓰기( 위 경로에 있는 파일)

    while(ros::ok()){
        ros::spinOnce();

        ROS_INFO("pos : x = %f | y = %f | theta = %f", pos[0], pos[1], pos[2]);
        txtFile << "position: " << pos[0] << "\t" << pos[1] << "\t" << pos[2] << endl;// x position, tab, y position, tab, theta 기록

        loop_rate.sleep();
    }

    txtFile.close();//열린 파일 닫기
    return 0;
}
