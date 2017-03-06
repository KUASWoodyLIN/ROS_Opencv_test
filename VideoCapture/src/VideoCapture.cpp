#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

// Subscriber to bottom camera
image_transport::Subscriber sub;

// Time control
ros::Time lastTime;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg,"bgr8" )->image;
        cv::imshow("view", InImage);
        cv::waitKey(35);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
//    cv::namedWindow("view",CV_WINDOW_NORMAL);
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
//    sub = it.subscribe("/camera/image", 1, imageCallback);
    sub = it.subscribe("/camera/image", 1, imageCallback,ros::VoidPtr(),image_transport::TransportHints("compressed"));
    ros::spin();
    cv::destroyWindow("view");
}

