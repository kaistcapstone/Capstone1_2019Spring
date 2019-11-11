#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <sstream>

using namespace std;
using namespace cv;
using namespace rs2;

int main(int argc, char** argv){
  cout<<"1"<<endl;
  ros::init(argc, argv, "webcam");   //init ros node. The name of node is decided here.
  ros::NodeHandle nh;  // create node handler
  image_transport::ImageTransport it(nh);  //create image transport and connect it to the node handler created above
  cout<<"2"<<endl;
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cout<<"3"<<endl;  //create a publisher that is connected to image traport(it), the topic name and queue size are decided.
  image_transport::Publisher pub_depth = it.advertise("camera/image_depth", 1);
  cout<<"4"<<endl;

  ros::NodeHandle nh_private("~"); //create a private node handler to handle parameters related to the node
  bool reduced = false; //boolean variable that decides whether you want to use reduced image or not-reduced image. If there is slow-down caused by big-sized data, then set it true.
  nh_private.param<bool>("reduced", reduced, false); //declare ros parameter named "reduced", the value of ros parameter will be saved in the variable 'reduced'
  bool show = false; //boolean variable that decides whether you want to see the image via new window
  nh_private.param<bool>("show",show,false); //declare ros parameter named "show",

  //cv::VideoCapture cap = cv::VideoCapture(0); //create cv::VideoCapture (*opencv function that capture images from camera) //used for logitec cam
  rs2::colorizer color_map;
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8,30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16,30);
  pipe.start(cfg);
  rs2::frameset frames;
  cv::Mat frame;  //assign a memory to save images with variable name 'frame'
  cv::Mat buffer; //assign a memory to save images with variable name 'frame'
  cv::Mat buffer_depth;
  sensor_msgs::ImagePtr msg; //declare a pointer of sensor_msgs::Image.
  sensor_msgs::ImagePtr msg_depth;

  ros::Rate loop_rate(30); //set loop rate. you can set hz here. The maximum hz of webcam device is 30hz. If you set the hz here larger than 30, it is meaningless.
  while (nh.ok()){
    //cap >> frame;  //transfer image captured by 'cap' to 'frame'
    frames=pipe.wait_for_frames().apply_filter(color_map);
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();
    Mat color(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    cout<<"5"<<endl;
    Mat depth(Size(1280, 720), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    cout<<"6"<<endl;
    //이미지 데이터의 크기를 줄이기 위해 원본 이미지보다 낮은 320*240 화질로 변경함
    if(reduced==true){
      cv::resize(buffer, color, cv::Size(320, 240)); //reduced the size of the image
      cv::resize(buffer_depth, depth, cv::Size(320, 240)); //reduced the size of the image
    }
    else{
      cout<<"7"<<endl;
      buffer = color;
      buffer_depth = depth;
    }
    if(show==true){
      cout<<"8"<<endl;
      cv::imshow("show",color);  //create a window that shows the image
      if(cv::waitKey(50)==113) return 0; //wait for a key command. if 'q' is pressed, then program will be terminated.
    }
    cout<<"9"<<endl;
    msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", buffer_depth).toImageMsg();
    //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer).toImageMsg(); //converting a image 'buffer' to ros message
    cout<<"10"<<endl;
    //msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", buffer_depth).toImageMsg();
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer).toImageMsg();
    cout<<"11"<<endl;
    //리사이즈한 이미지를 publish
    //pub.publish(msg);  //publish a message
    pub_depth.publish(msg_depth);
    cout<<"12"<<endl;
    //pub_depth.publish(msg_depth);
    pub.publish(msg);
    cout<<"13"<<endl;
  }
  ros::spinOnce();
  loop_rate.sleep(); //this will sleep the loop to satisfy hz you decided in the above line ros::Rate loop_rate(N)
}
