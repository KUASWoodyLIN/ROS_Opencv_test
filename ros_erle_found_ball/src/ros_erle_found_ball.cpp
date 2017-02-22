#include <ros/ros.h>

//MAVROS
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>

//OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

// Output
#include <iostream>

// Vector
#include <vector>

using namespace std;

//函數聲明
void found_ball(void);
void mavrosStateCb(const mavros_msgs::StateConstPtr &msg);

// >>>>> Color to be tracked
#define MIN_H_BLUE 200	//200
#define MAX_H_BLUE 300	//300

#define FACTOR  0.6
#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

// Subscriber to flight mode
ros::Subscriber mavros_state_sub;

// RC publisher
ros::Publisher rc_pub;

//MODE Service
ros::ServiceClient mode_ser;

// Time control
ros::Time lastTime;

//Image center
float ImageX = 320 , ImageY = 240 ;

float BallX, BallY; // Ball center
// Error between Image and Mark
float ErX = 0.0;
float ErY = 0.0;

double Roll, Pitch;

// Flight mode
std::string mode;
bool guided;
bool armed;
bool MODE = true;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "found_ball");
    ros::NodeHandle nh;
    lastTime = ros::Time::now();
    mavros_state_sub = nh.subscribe("/mavros/state", 1, mavrosStateCb);
    rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    mode_ser = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;

    //set AUTO MODE
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "AUTO";
    if(mode_ser.call(srv_setMode))
    {
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    }
    else
    {
        ROS_ERROR("Failed SetMode");
        //return -1;
    }

    found_ball();

}

void found_ball(void)
{

    // Camera frame
    cv::Mat frame;

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    // Transition State Matrix A
    cv::setIdentity(kf.transitionMatrix);
    // Measure Matrix H
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;
    // Process Noise Covariance Matrix Q
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // Camera Index
    int idx = 0;

    // Camera Capture
    cv::VideoCapture cap;

    // >>>>> Camera Settings
    if (!cap.open(idx))
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        //return EXIT_FAILURE;
    }
    // <<<<< Camera Settings

    cout << "\nHit 'q' to exit...\n";

    char ch = 0;

    double ticks = 0;
    bool found = false;

    int notFoundCount = 0 , FoundCount = 0 , StateCount = 0 ,BallCount = 0;
    cv::Mat res;

    while (ch != 'q' && ch != 'Q')
    {
        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Frame acquisition
	
        cap >> frame;        
        frame.copyTo( res );

        if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A
StateCount++;
if(StateCount >= 30)
{
            cout << "dT:" << endl << dT << endl;

            state = kf.predict();
	
            cout << "State post:" << endl << state << endl;
StateCount = 0;
}
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
        // <<<<< Noise smoothing

        // >>>>> HSV conversion
        cv::Mat frmHsv;
        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
        // <<<<< HSV conversion

        // >>>>> Color Thresholding
        // Note: change parameters for different colors
        cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
        //cv::inRange(frmHsv, cv::Scalar(18, 100, 80),
        //            cv::Scalar(35, 240, 255), rangeRes);
        //cv::inRange(frmHsv, cv::Scalar(18, 200, 170),
        //            cv::Scalar(35, 240, 240), rangeRes);

        cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
                   cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);

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
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            /*else
                kf.statePost = state;*/
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
	    if(FoundCount >= 30)
	    {
		//LOITER MODE
		if ( MODE == true )
		{
		    // Create SetMode msg
    		    mavros_msgs::SetMode srv_setMode;
    		    srv_setMode.request.base_mode = 0;
    		    srv_setMode.request.custom_mode = "LOITER";
    		    if(mode_ser.call(srv_setMode))
		    {
        		ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    		    }
		    else
		    {
        		ROS_ERROR("Failed SetMode");
    		    }
		    MODE = false;
		}
            	BallX = meas.at<float>(0);
            	BallY = meas.at<float>(1);
            	ErX = ImageX - BallX;
            	ErY = ImageY - BallY;

        	// Calculate Roll and Pitch depending on the mode
        	//if (mode == "LOITER")
		//{
            	    Roll = BASERC - ErX * FACTOR;
            	    Pitch = BASERC + ErY * FACTOR;
        	//}
		//else
		//{
            	//    Roll = BASERC;
            	//    Pitch = BASERC;
        	//}  
        	// Limit the Roll
        	if (Roll > MAXRC)
        	{
            	    Roll = MAXRC;
        	} 
		else if (Roll < MINRC)
        	{
            	    Roll = MINRC;
        	}

        	// Limit the Pitch
        	if (Pitch > MAXRC)
        	{
            	    Pitch = MAXRC;
        	} 
		else if (Pitch < MINRC)
        	{
            	    Pitch = MINRC;
        	}
        	// Create RC msg
        	mavros_msgs::OverrideRCIn msg;
        	msg.channels[0] = Roll;     //Roll
        	msg.channels[1] = Pitch;    //Pitch
        	msg.channels[2] = BASERC;   //Throttle
        	msg.channels[3] = 0;        //Yaw
        	msg.channels[4] = 0;
        	msg.channels[5] = 0;
       	 	msg.channels[6] = 0;
        	msg.channels[7] = 0;

        	rc_pub.publish(msg);
        	cout << "Pitch:" << Pitch << endl;
        	cout << "Roll:" << Roll << endl;
        	cout << "Measure matrix:" << endl << meas << endl;
		FoundCount = 0;
	    }
        }
        // <<<<< Kalman Update

        // Final result
        cv::imshow("Tracking", res);

        // User key
        ch = cv::waitKey(50);
    }
    // <<<<< Main loop
    //return EXIT_SUCCESS;

}

void mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided==128;
    armed = msg->armed==128;
}



