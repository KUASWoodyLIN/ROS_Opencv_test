#include <ros/ros.h>

//ROS image
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

//found_ball
#include "found_ball/Ballinfo.h"

//Output
#include <iostream>

//Vector
#include <vector>

using namespace std;

//函數聲明
void ball_tracking(void);

// >>>>> Color to be tracked
#define MIN_H_BLUE 200	//200
#define MAX_H_BLUE 300	//300

#define MIN_H_ORANGE 0
#define MAX_H_ORANGE 60

#define MIN_H_RED 0
#define MAX_H_RED 30

// Publisher to bottom camera
image_transport::Publisher pub;

// Distance publisher
ros::Publisher Distance_pub;

//Image center
float ImageX, ImageY;

 // Ball center
float BallX, BallY;

//param
string ball_color ; //= "red"

char ch = 0;
double ticks = 0;
bool found = false;
bool first_setup = true;
int notFoundCount = 0 , FoundCount = 0 , BallCount = 0;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ball_tracking");
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle("~");
    image_transport::ImageTransport it(nh);

    // Publishe
    image_transport::Publisher pub = it.advertise("ball_tracking/image", 1);

    // Subscribe
    Distance_pub = nh.advertise<found_ball::Ballinfo>("/ball/info",1);

    //Get param
    privateHandle.getParam("ball_color", ball_color );

    ball_tracking();
}

void ball_tracking(void)
{

    // Messages
    found_ball::Ballinfo distance_msg;

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // Camera Capture
    cv::VideoCapture cap(0);

    // >>>>> Camera Settings
    if (!cap.isOpened())
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        //return EXIT_FAILURE;
    }
    // <<<<< Camera Settings

    // Camera frame
    cv::Mat frame;
    cv::Mat res;

    // Pub Image
    sensor_msgs::ImagePtr image_msg;

    while (ch != 'q' && ch != 'Q')
    {

        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Get the msg image
        cap >> frame;        
        frame.copyTo( res );

        if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            //cout << "dT:" << endl << dT << endl;

            state = kf.predict();
	
            //cout << "State post:" << endl << state << endl;

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
        }

        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);

        // >>>>> HSV conversion
        cv::Mat frmHsv;
        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);

        // >>>>> Color Thresholding
        // Note: change parameters for different colors
        cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
	if( ball_color == "blue" )
	{
            cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
                        cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
	}
	else if( ball_color == "orange" )
	{
            cv::inRange(frmHsv, cv::Scalar(MIN_H_ORANGE / 2, 100, 80),
                        cv::Scalar(MAX_H_ORANGE / 2, 255, 255), rangeRes);
	}
	else if( ball_color == "red" )
	{
            cv::inRange(frmHsv, cv::Scalar(MIN_H_RED / 2, 100, 80),
                        cv::Scalar(MAX_H_RED / 2, 255, 255), rangeRes);
	}
        // <<<<< Color Thresholding

        // >>>>> Improving the result
        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        // <<<<< Improving the result

        // Thresholding viewing
        //cv::imshow("Threshold", rangeRes);

        // >>>>> Contours detection
        vector<vector<cv::Point> > contours;
        cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        // <<<<< Contours detection

        // >>>>> Filtering
        vector<vector<cv::Point> > balls;
        vector<cv::Rect> ballsBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect bBox;
            bBox = cv::boundingRect(contours[i]);	//！！！！

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                balls.push_back(contours[i]);
                ballsBox.push_back(bBox);
            }
        }
        // <<<<< Filtering

BallCount++;
if(BallCount >= 30)
{
        cout << "Balls found:" << ballsBox.size() << endl;
BallCount = 0;
}

        // >>>>> Detection result
        for (size_t i = 0; i < balls.size(); i++)
        {
            cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
            cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(res, sstr.str(),
                        cv::Point(center.x + 3, center.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        }
        // <<<<< Detection result

        // >>>>> Kalman Update
        if (balls.size() == 0)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
	    if( notFoundCount == 30 && first_setup == false )
            {
		// Found ball false
	    	distance_msg.ball_state = false ;

		// Publish message
        	Distance_pub.publish(distance_msg);
	    }

            if( notFoundCount >= 100 )
            {
                found = false;


            }
            else
                kf.statePost = state;
        }
        else
        {
            notFoundCount = 0;

            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            meas.at<float>(2) = (float)ballsBox[0].width;
            meas.at<float>(3) = (float)ballsBox[0].height;

            if (!found) // First detection!
            {
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialization

                kf.statePost = state;
                
                found = true;
            }
            else
                kf.correct(meas); // Kalman Correction


	    FoundCount++;
	    if(FoundCount >= 15)
	    {
		// when first time found ball turn to false
		first_setup = false;

        	// Image center
        	ImageX = res.cols / 2.0f;
        	ImageY = res.rows / 2.0f;

		// Ball center
            	BallX = meas.at<float>(0);
            	BallY = meas.at<float>(1);

		// Distance between Image and Ball
            	distance_msg.x = ImageX - BallX;
            	distance_msg.y = ImageY - BallY;
		distance_msg.ball_state = true ;

		// Publish message
        	Distance_pub.publish(distance_msg);
		FoundCount = 0;
                cout << "Measure matrix:" << endl << meas << endl;
	    }
        }
        // <<<<< Kalman Update

        // Final result
        //cv::imshow("Tracking", res);
	image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res).toImageMsg();
	pub.publish(image_msg);
        // User key
        ch = cv::waitKey(35);
    }
}


