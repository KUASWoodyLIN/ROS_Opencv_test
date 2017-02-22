#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <highgui.h>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2\video\background_segm.hpp>

#include <Windows.h>

using namespace cv;
using namespace std;

/// Global variables
int hist = 0;
int const histMax = 10;
int thresh = 0;
int const threshMax = 5000;
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

int MIN_NUM_OBJECTS = 1;
int MAX_NUM_OBJECTS=100;
int MIN_OBJECT_AREA = 1*1;
int MAX_OBJECT_AREA = 5000*5000;

Mat curVid;         // live feed from IP Cam
Mat imgHSV;
Mat erosion_dst;
Mat dilation_dst;
Mat mask;           // final filtered image
Mat test;           // test for draw objects

/** Function Headers */
void setwindowSettings();
void HSV();
void Erosion( int, void* );
void Dilation( int, void* );
IplImage* GetThresholdedImage(IplImage* imgHSV);
void trackFilteredObject();
void drawObject(int x,int y,Mat &frame);
string intToString(int number);


int main(int argc, char *argv[])
{
    // get cam feed and show in new window
    VideoCapture cap(0);
     if (!cap.isOpened())  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    setwindowSettings();

    // background subtraction init
    BackgroundSubtractorMOG2 bg = BackgroundSubtractorMOG2(hist,thresh,false);

    cap.read(curVid);

    Sleep(5000);

    // show images in windows
    while(1)
    {
        // show camera feed
        bool bSuccess = cap.read(curVid); // read a new frame from video
        if (!bSuccess) // if not success, break loop
        {
            cout << "Cannot read a frame from video file" << endl;
            break;
        }

        // start morph functions then draw
        HSV();
        Erosion( 0, 0 );
        Dilation( 0, 0 );

        // background subtraction
        bg(dilation_dst,mask,-1);
        imshow("mask",mask);

        // draw to curVid
        trackFilteredObject();

        imshow("test", test);

        // escape program with keystroke
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break; 
        }

    }
    return 0;
}


void setwindowSettings()
{
    namedWindow("Settings", CV_WINDOW_AUTOSIZE);
    resizeWindow("Settings",500,600);
    namedWindow("mask", CV_WINDOW_AUTOSIZE);
    namedWindow("test", CV_WINDOW_AUTOSIZE);

    // Create Erosion Trackbar
    createTrackbar( "EroElem", "Settings",
                    &erosion_elem, max_elem,
                    Erosion );
    createTrackbar( "EroSize", "Settings",
                    &erosion_size, max_kernel_size,
                    Erosion );

    /// Create Dilation Trackbar
    createTrackbar( "DialElem", "Settings",
                    &dilation_elem, max_elem,
                    Dilation );
    createTrackbar( "DialSize", "Settings",
                    &dilation_size, max_kernel_size,
                    Dilation );

    /*/// Create GaussianBlur Trackbars
    createTrackbar( "blurType", "Settings",
                    &blurType, max_blurType,
                    Blur );*/

    /// Create History Trackbar
    createTrackbar( "ImgHist", "Settings",
                    &hist, histMax );

    /// Create Threshold Trackbar
    createTrackbar( "Threshold", "Settings",
                    &thresh, threshMax );

    // Create NUM_OBJECTS Trackbar
    createTrackbar("MIN_NUM_OBJECTS", "Settings", 
                    &MIN_NUM_OBJECTS, 100);
    createTrackbar("MAX_NUM_OBJECTS", "Settings", 
                    &MAX_NUM_OBJECTS, 100);

    // Create NUM_OBJECTS Trackbar
    createTrackbar("MIN_OBJECT_AREA", "Settings", 
                    &MIN_OBJECT_AREA, 5000*5000);
    createTrackbar("MAX_OBJECT_AREA", "Settings", 
                    &MAX_OBJECT_AREA, 5000*5000);

}

/**  @function HSV  */
void HSV()
{
    //cvGetSize(curVid);
    //Mat* img_HSV = cvCreateImage(cvGetSize(curVid), IPL_DEPTH_8U, 3); 
    cvtColor(curVid,imgHSV,CV_BGR2HSV);
    //imgHSV = GetThresholdImage(img_HSV);
    //imshow("HSV", imgHSV);
}

/**  @function Erosion  */
void Erosion( int, void* )
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( imgHSV, erosion_dst, element );
}

/** @function Dilation */
void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( erosion_dst, dilation_dst, element );
}

void trackFilteredObject(){

    int x,y;

    Mat temp;
    mask.copyTo(temp);
    //inRange(mask,Scalar(MIN_OBJECT_AREA),Scalar(MAX_OBJECT_AREA),temp);
    curVid.copyTo(test);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );


    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&0, rand()&0, rand()&255 );
        drawContours( test, contours, idx, color, CV_FILLED, 8, hierarchy );
    }


    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    cout << hierarchy.size() << endl;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //this is setup to get the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration. DO NOT WANT THIS!
                if(area>MIN_OBJECT_AREA){
                    x = moment.m10/area;
                    y = moment.m01/area;



                    objectFound = true;

                }else objectFound = false;


            }
            //let user know you found an object
            if(objectFound ==true){
                //draw object location on screen
                drawObject(x,y,test);}

        }else putText(test,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}


void drawObject(int x,int y,Mat &frame){

    circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255),3);
    putText(frame,intToString(x)+ " , " + intToString(y),cv::Point(x,y+20),1,1,Scalar(0,255,0));

}

string intToString(int number){


    std::stringstream ss;
    ss << number;
    return ss.str();
}
