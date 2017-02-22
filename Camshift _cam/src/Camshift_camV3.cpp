#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;

/*
static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;
        break;
    }
}
*/

static void help()
{
    std::cout << "\nThis is a demo that shows mean-shift based tracking\n"
                 "You select a color objects such as your face and it tracks it.\n"
                 "This reads from video camera (0 by default, or the camera number the user enters\n"
                 "Usage: \n"
                 "   ./camshiftdemo [camera number]\n";

    std::cout << "\n\nHot keys: \n"
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

int main( int argc, char** argv )
{
    help();

    VideoCapture cap(0);
    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;

    CommandLineParser parser(argc, argv, keys);
    int camNum = parser.get<int>("1");
    /********input image******/
    Mat img = imread("output_picture.jpg",CV_LOAD_IMAGE_COLOR);
    imshow("Display window", img); 

    cap.open(camNum);

    if( !cap.isOpened() )
    {
        help();
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: \n";
	parser.printMessage();
        //parser.printParams();
        return -1;
    }


    ros::init(argc, argv, "CamShift_cam");
    ros::NodeHandle nh;
    //namedWindow( "hist", 1 );
    namedWindow( "Histogram", 1 );
    namedWindow( "CamShift Demo", 1 );
    //setMouseCallback( "CamShift Demo", onMouse, 0 );
    createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
    createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
    createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false , first = true;

    for(;;)
    {
        if( !paused )
        {
            cap >> frame;
            if( frame.empty() )
                break;
        }

        frame.copyTo(image);
	
	if (first == true)
	{
	    //Mat image2;
	    //frame.copyTo(image);
	    Mat imageadd = image(Rect(0,0,img.cols,img.rows));
	    addWeighted(imageadd,0,img,1,0.,imageadd);
	    imshow("imageadd", image); 
	    selection = Rect(0,0,img.cols,img.rows);
	    trackObject = -1;
            first = false;
	}

        if( !paused )
        {
            cvtColor(image, hsv, COLOR_BGR2HSV);
            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;
		//inRange用来检查元素的取值范围是否在另两个矩阵的元素取值之间，返
		//回验证矩阵mask（0-1矩阵），这里用于制作掩膜板，只处理像素值为
		//H:0~180，S:smin~256, V:vmin~vmax之间的部分。mask是要求的，单通道  
                inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                        Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
		//将H分量拷贝到hue中，其他分量不拷贝。
                mixChannels(&hsv, 1, &hue, 1, ch, 1);
		/**********/
                if( trackObject < 0 )
                {
		    //roi为选中区域的矩阵，maskroi为0-1矩阵
                    Mat roi(hue, selection), maskroi(mask, selection);
		    //可得到影像的直方圖 輸入圖、幾張輸入、直方圖通道清單、可有可無遮罩、輸出直
                    //方圖、直方圖維度、直方圖橫軸數目、直方圖強度範圍//绘制色调直方图hist，仅限
                    //于用户选定的目标矩形区域 
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    //必须是单通道，hist是单通道。归一化，范围为0-255
                    normalize(hist, hist, 0, 255, CV_MINMAX);

                    trackWindow = selection;
                    trackObject = 1;//trackObject置1，接下来就不需要再执行这个if块了

                    histimg = Scalar::all(0);//用于显示直方图
		    //计算每个直方的宽度
                    int binW = histimg.cols / hsize;//hsize为16，共显示16个  
                    Mat buf(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
			//直方图每一项的颜色是根据项数变化的
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf, buf, CV_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
			//获取直方图每一项的高
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
			//画直方图(矩形)。opencv中左上角为坐标原点
                        rectangle( histimg, Point(i*binW,histimg.rows),
                                   Point((i+1)*binW,histimg.rows - val),
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }
                }
		/*****/
                calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                backproj &= mask;
                RotatedRect trackBox = CamShift(backproj, trackWindow,
                                    TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                if( trackWindow.area() <= 1 )
                {
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       trackWindow.x + r, trackWindow.y + r) &
                                  Rect(0, 0, cols, rows);
                }

                if( backprojMode )
                    cvtColor( backproj, image, COLOR_GRAY2BGR );
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
            }
        }
        else if( trackObject < 0 )
            paused = false;

        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }
	//imshow("hist",hist);
        imshow( "CamShift Demo", image );
        imshow( "Histogram", histimg );

        //char c = waitKey(30);
	char c = (char)waitKey(30);
        if( c == 27 )break;
        switch(c)
        {
	case 't':	//take picture
	    imwrite("output_picture.jpg",image);
	    break;
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
    }

    return 0;
}

