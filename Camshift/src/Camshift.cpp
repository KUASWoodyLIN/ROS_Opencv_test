#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <ctype.h>

cv::Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
cv::Point origin;
cv::Rect selection;
int vmin = 10, vmax = 256, smin = 30;

// Subscriber to bottom camera
image_transport::Subscriber sub;

// Time control
ros::Time lastTime;

static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= cv::Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cv::Point(x,y);
        selection = cv::Rect(x,y,0,0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;
        break;
    }
}

static void help()
{
    cout << "\nThis is a demo that shows mean-shift based tracking\n"
            "You select a color objects such as your face and it tracks it.\n"
            "This reads from video camera (0 by default, or the camera number the user enters\n"
            "Usage: \n"
            "   ./camshiftdemo [camera number]\n";

    cout << "\n\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tc - stop the tracking\n"
            "\tb - switch to/from backprojection view\n"
            "\th - show/hide object histogram\n"
            "\tp - pause video\n"
            "To initialize tracking, select the object with mouse\n";
}

const char* keys =
{
    "{1|  | 0 | camera number}"
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        if( !paused )
        {
            // Get the msg image
            image = cv_bridge::toCvShare(msg,"bgr8" )->image;
            cv::cvtColor(image, hsv, COLOR_BGR2HSV);
            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;

                cv::inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                            Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
            cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);
                if( trackObject < 0 )
                {
                    // Object has been selected by user, set up CAMShift search properties once
                    cv::Mat roi(hue, selection), maskroi(mask, selection);
                    cv::calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    cv::normalize(hist, hist, 0, 255, NORM_MINMAX);

                    trackWindow = selection;
                    trackObject = 1; // Don't set up again, unless user selects new ROI

                    histimg = Scalar::all(0);
                    int binW = histimg.cols / hsize;
                    cv::Mat buf(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                    buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cv::cvtColor(buf, buf, COLOR_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                        cv::rectangle( histimg, cv::Point(i*binW,histimg.rows),
                                       cv::Point((i+1)*binW,histimg.rows - val),
                                       cv::Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }
                }
                 cv::calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                 backproj &= mask;
                 cv::RotatedRect trackBox = cv::CamShift(backproj, trackWindow,
                                                TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                 if( trackWindow.area() <= 1 )
                 {
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = cv::Rect(trackWindow.x - r, trackWindow.y - r,
                                           trackWindow.x + r, trackWindow.y + r) &
                                  cv::Rect(0, 0, cols, rows);
                 }

                 if( backprojMode )
                     cv::cvtColor( backproj, image, COLOR_GRAY2BGR );
                 cv::ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
            }
        }
        else if( trackObject < 0 )
            paused = false;
        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            cv::Mat roi(image, selection);
            cv::bitwise_not(roi, roi);
        }

        cv::imshow("CamShift Demo", image);
        imshow( "Histogram", histimg );

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
        {
        case 'b':
            backprojMode = !backprojMode;
            break;
        case 'c':
            trackObject = 0;
            histimg = Scalar::all(0);
            break;
        case 'h':
            showHist = !showHist;
            if( !showHist )
                destroyWindow( "Histogram" );
            else
                namedWindow( "Histogram", 1 );
            break;
        case 'p':
            paused = !paused;
            break;
        default:
            ;
        }

//        cv::waitKey(0);

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    help();

    cv::Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    CommandLineParser parser(argc, argv, keys);
    int camNum = parser.get<int>("1");

    cv::namedWindow( "Histogram", 0 );	//CV_WINDOW_NORMAL
    cv::namedWindow( "CamShift Demo", 0 );
    cv::setMouseCallback( "CamShift Demo", onMouse, 0 );
    cv::createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
    cv::createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
    cv::createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );
    cv::startWindowThread();

    cv::Mat frame, hsv, hue, mask, hist, histimg = cv::Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;

    ros::init(argc, argv, "CamShift");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    sub = it.subscribe("camera/image", 1, imageCallback);

    ros::spin();
    cv::destroyWindow("view");
}


