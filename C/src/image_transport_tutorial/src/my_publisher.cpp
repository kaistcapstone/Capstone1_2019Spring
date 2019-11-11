#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



int main(int argc, char** argv)
{
  cv::VideoCapture cap_1;
  cv::VideoCapture cap_2;
  cv::VideoCapture cap_3;
  cv::Mat frame_1;
  cv::Mat buffer_1;
  cv::Mat frame_2;
  cv::Mat buffer_2;
  cv::Mat frame_3;
  cv::Mat buffer_3;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_1 = it.advertise("camera/image_1", 1);
  image_transport::Publisher pub_2 = it.advertise("camera/image_2", 1);
  image_transport::Publisher pub_3 = it.advertise("camera/image_3", 1);
  sensor_msgs::ImagePtr msg_1;
  sensor_msgs::ImagePtr msg_2;
  sensor_msgs::ImagePtr msg_3;
  //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  cap_1 = cv::VideoCapture(0);
  cap_2 = cv::VideoCapture(1);
  cap_3 = cv::VideoCapture(2);
  ros::Rate loop_rate(100);
  while (nh.ok()) {
    cap_1 >> frame_1;
    //이미지 데이터의 크기를 줄이기 위해 원본 이미지보다 낮은 320*240 화질로 변경함
    cv::resize(frame_1, buffer_1, cv::Size(320, 240));
    msg_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer_1).toImageMsg();
    //리사이즈한 이미지를 publish
    pub_1.publish(msg_1);
	ROS_INFO("send cam1");

    //50ms 마다 publish(20Hz)

    cap_2 >> frame_2;
    //이미지 데이터의 크기를 줄이기 위해 원본 이미지보다 낮은 320*240 화질로 변경함
    cv::resize(frame_2, buffer_2, cv::Size(320, 240));
    msg_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer_2).toImageMsg();
    //리사이즈한 이미지를 publish
    pub_2.publish(msg_2);
	ROS_INFO("send cam2");
    //50ms 마다 publish(20Hz)

    cap_3 >> frame_3;
    //이미지 데이터의 크기를 줄이기 위해 원본 이미지보다 낮은 320*240 화질로 변경함
    cv::resize(frame_3, buffer_3, cv::Size(320, 240));
    msg_3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer_3).toImageMsg();
    //리사이즈한 이미지를 publish
    pub_3.publish(msg_3);
	ROS_INFO("send cam3");
    //50ms 마다 publish(20Hz)

    cv::waitKey(100);

  }
  ros::spinOnce();
  loop_rate.sleep();
}
