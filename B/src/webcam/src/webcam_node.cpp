#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "webcam");   //init ros node. The name of node is decided here.
  ros::NodeHandle nh;  // create node handler
  image_transport::ImageTransport it(nh);  //create image transport and connect it to the node handler created above
  image_transport::Publisher pub = it.advertise("camera/image", 1);  //create a publisher that is connected to image traport(it), the topic name and queue size are decided.

  ros::NodeHandle nh_private("~"); //create a private node handler to handle parameters related to the node
  bool reduced = true; //boolean variable that decides whether you want to use reduced image or not-reduced image. If there is slow-down caused by big-sized data, then set it true.
  nh_private.param<bool>("reduced", reduced, false); //declare ros parameter named "reduced", the value of ros parameter will be saved in the variable 'reduced'
  bool show = false; //boolean variable that decides whether you want to see the image via new window
  nh_private.param<bool>("show",show,false); //declare ros parameter named "show",

  cv::VideoCapture cap1 = cv::VideoCapture(0); //create cv::VideoCapture (*opencv function that capture images from camera)
  cv::VideoCapture cap2 = cv::VideoCapture(1);
  cv::Mat frame_1;  //assign a memory to save images with variable name 'frame'
  cv::Mat frame_2;
  cv::Mat buffer_1; //assign a memory to save images with variable name 'frame'
  cv::Mat buffer_2;
  cv::Mat buffer;
  sensor_msgs::ImagePtr msg; //declare a pointer of sensor_msgs::Image.

  ros::Rate loop_rate(30); //set loop rate. you can set hz here. The maximum hz of webcam device is 30hz. If you set the hz here larger than 30, it is meaningless.
  while (nh.ok()) {
    cap1 >> frame_1;  //transfer image captured by 'cap' to 'frame'
    cap2 >> frame_2;
    //이미지 데이터의 크기를 줄이기 위해 원본 이미지보다 낮은 320*240 화질로 변경함
    if(reduced==true){
	cv::resize(frame_1, buffer_1, cv::Size(320, 240)); //reduced the size of the image
  //cv::resize(frame_2, buffer_2, cv::Size(320, 240));
    }
    else{
	buffer_1 = frame_1;
  buffer_2 = frame_2;
    }

    if(show==true){
 	cv::imshow("show_1",buffer_1);  //create a window that shows the image
  //cv::imshow("show_2",buffer_2);
        if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
		return 0;
	};

    }
    cv::hconcat(buffer_1,buffer_2,buffer);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer).toImageMsg(); //converting a image 'buffer' to ros message
        //리사이즈한 이미지를 publish
    pub.publish(msg);  //publish a message
    loop_rate.sleep(); //this will sleep the loop to satisfy hz you decided in the above line ros::Rate loop_rate(N)
  }
}
