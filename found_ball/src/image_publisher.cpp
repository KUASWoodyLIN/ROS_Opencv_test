#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    //Image Transport setting
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    //Get image setting
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) return -1;

    ros::Rate loop_rate(30);
    while (nh.ok()) 
    {
	cap >> frame;
	if(!frame.empty()) 
	{
	    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	    pub.publish(msg);
	    cv::waitKey(1);
	}
    	ros::spinOnce();
    	loop_rate.sleep();
    }
}

