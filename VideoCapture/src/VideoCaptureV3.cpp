  GNU nano 2.2.6            File: VideoCapture.cpp                              

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include <sensor_msgs/CompressedImage.h>

// Subscriber to bottom camera
image_transport::Subscriber sub;

// Time control
ros::Time lastTime;

void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    try
    {
        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8" )->image;
        cv::imshow("view", InImage);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str($
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("camera/image/compressed", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}



