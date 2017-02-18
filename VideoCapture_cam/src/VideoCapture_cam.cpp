#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


int main(int argc, char **argv)
{
    cv::Mat frame;
    cv::VideoCapture cap(0);
    if(!cap.isOpened()){
        return -1;
    }
    while(true){   
        if(!cap.read(frame))
            break;
        cv::Mat src = cv::Mat(frame);
        cv::imshow( "window",  src );
        cv::waitKey(30);
    }
}
