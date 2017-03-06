#include <ros/ros.h>

//ROS image
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

//Output
#include <iostream>

//Vector
#include <vector>

using namespace std;

// >>>>> Color to be tracked
#define MIN_H_BLUE 200	//200
#define MAX_H_BLUE 300	//300

#define MIN_H_ORANGE 0
#define MAX_H_ORANGE 60

#define MIN_H_RED 0
#define MAX_H_RED 30

// Subscriber to bottom camera
image_transport::Subscriber sub;

//Distance publisher
ros::Publisher Distance_pub;

//Image center
float ImageX, ImageY;

 // Ball center
float BallX, BallY;

// Error between Image and Mark
float ErX = 0.0 , ErY = 0.0;

char ch = 0;

double ticks = 0;
bool found = false;

int notFoundCount = 0 , FoundCount = 0 , BallCount = 0;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ball tracking");
    ros::NodeHandle nh;
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("image", 1 , imageCallback);
    //Distance_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    //Get param
    ros::param::get("ball_color", ball_color );
    ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {

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

        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Get the msg image
        cv::Mat res;
	cv::Mat frame;
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
	frame.copyTo( res );

        // Get the Image center
        ImageX = res.cols / 2.0f;
        ImageY = res.rows / 2.0f;

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
	if( ball_color = "blue" )
	{
            cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
                        cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
	}
	else if( ball_color = "orange" )
	{
            cv::inRange(frmHsv, cv::Scalar(MIN_H_ORANGE / 2, 100, 80),
                        cv::Scalar(MAX_H_ORANGE / 2, 255, 255), rangeRes);
	}
	else if( ball_color = "red" )
	{
            cv::inRange(frmHsv, cv::Scalar(MIN_H_RED / 2, 100, 80),
                        cv::Scalar(MAX_H_RED / 2, 255, 255), rangeRes);
	}
        // <<<<< Color Thresholding

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

	    // >>>> Publish msg
	    FoundCount++;
	    if(FoundCount >= 30)
	    {
		//Point2D msg
          	ros_erle_found_ball::Point2D point_msg;

        	//Image center
        	ImageX = res.cols / 2.0f;
        	ImageY = res.rows / 2.0f;

		//Ball center
            	BallX = meas.at<float>(0);
            	BallY = meas.at<float>(1);

		//The distance between Image and Ball
            	ErX = ImageX - BallX;
            	ErY = ImageY - BallY;
          	point_msg.x = ErX;
          	point_msg.y = ErY;
		FoundCount = 0;
		Distance_pub.publish( point_msg );
	    }
	    // <<<< Publish msg
		
                cout << "Measure matrix:" << endl << meas << endl;
        }
        // <<<<< Kalman Update

        // Final result
        cv::imshow("Tracking", res);

        // User key
        ch = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


