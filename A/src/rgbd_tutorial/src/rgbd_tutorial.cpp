#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

// global variable
Mat mask;

// void pointcloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg){
void pointcloud_cb (sensor_msgs::PointCloud2 msg){
  	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		msg.fields[3].datatype = 7;
		pcl::fromROSMsg (msg, cloud);

		float x = cloud.points[1].x;
		float y = cloud.points[1].y;
		float z = cloud.points[1].z;
		float r = cloud.points[1].r;
		float g = cloud.points[1].g;
		float b = cloud.points[1].b;

}

void depthImage_cb(const sensor_msgs::ImageConstPtr &msg_depth){

	try{
		cout<<"7"<<endl;
		cv_bridge::CvImageConstPtr depth_img_cv;
		Mat depth_mat;
		// Get the ROS image to opencv
		cout<<"8"<<endl;
		depth_img_cv = cv_bridge::toCvShare (msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
		depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001);

		cout<<"9"<<endl;
		Scalar mean_value;
		mean_value = mean(depth_mat, mask);		//Mean distance value of red pixels

		cout<<"10"<<endl;
		float mean_val = sum(mean_value)[0];

		cout<<"11"<<endl;
		ROS_INFO("%f", mean_val);

		cout<<"12"<<endl;
		imshow("view", depth_mat);
		cv::waitKey(1);

	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'z16'.", msg_depth->encoding.c_str());
	}

}

void colorImage_cb(const sensor_msgs::ImageConstPtr &msg){

	try{
			cout<<"1"<<endl;
	    Mat frame;
	    frame = cv_bridge::toCvShare(msg, "bgr8")->image;		//msg data to cv::Mat type image

			cout<<"2"<<endl;
	    Mat hsv;
			cvtColor(frame, hsv, COLOR_BGR2HSV);	//convert RBG to HSV

			cout<<"3"<<endl;
			Mat mask1,mask2,mask;		//mask variables

			cout<<"4"<<endl;
			// Creating masks to detect the upper and lower red color.
			inRange(hsv, Scalar(0, 30, 30, 0), Scalar(10, 255, 255, 0), mask1);
	 		inRange(hsv, Scalar(150, 30, 30, 0), Scalar(180, 255, 255, 0), mask2);

			cout<<"5"<<endl;
  		mask = mask1 | mask2;

			cout<<"6"<<endl;
  		imshow("view2", mask);
  		cv::waitKey(1);

	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv){
	  cout<<"1"<<endl;

    ros::init(argc, argv, "rgbd_tutorial_node");
    ros::NodeHandle nh;

		image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder

		cout<<"2"<<endl;
		image_transport::Subscriber sub_color = it.subscribe("camera/image", 1, colorImage_cb);
		cout<<"3"<<endl;
		image_transport::Subscriber sub_depth = it.subscribe("camera/image_depth", 1, depthImage_cb);

    ros::Rate loop_rate(30);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
