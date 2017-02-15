#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


int main(int argc, char **argv)
{
    //ros::init(argc, argv, "image_listener");
    //ros::NodeHandle nh;
    //cv::namedWindow("view");
    cv::Mat img= cv::imread("/home/woodylin/圖片/1.png");
        cv::imshow("view", img);
        cv::waitKey(60000);
    //ros::spin();
}
