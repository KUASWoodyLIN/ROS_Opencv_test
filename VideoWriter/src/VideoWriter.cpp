#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

int main()
{
    VideoCapture capture(0);
    if(!capture.isOpened())
    {  
        return -1;
    }
    Size videoSize = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));	//++
    VideoWriter writer;		//++
    writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize);		//++
    namedWindow("show image",0);

    while(true)
    {
        Mat frame;
        capture >> frame; 
        if(!frame.empty()){
            writer.write(frame);	//++
            imshow("show image", frame);
            if(waitKey(33) == 27){ 
                break;
            }
        }
    }
    return 0;
}
