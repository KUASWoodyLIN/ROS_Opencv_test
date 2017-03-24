#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc,char **argv)
{
	cv::Videocapture capture(0);
	if(!capture.isOpened())
	{
		return -1;
	}
	Size videoSize = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	cv::VideoWriter writer;
	writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize);
}
