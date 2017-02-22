#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;
/** @function main */
const char* keys =
{
    "{1|  | 0 | camera number}"
};
int main(int argc, char** argv)
{
    Mat src, src_gray;
    /// Read the image
    VideoCapture cap(0);

    CommandLineParser parser(argc, argv, keys);
    int camNum = parser.get<int>("1");
    namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    //imshow("Display window", img); 

    cap.open(camNum);

    if( !cap.isOpened() )
    {
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: \n";
	parser.printMessage();
        //parser.printParams();
        return -1;
    }

    for(;;)
    {
        if(!cap.read(src))
            break;

  /// Convert it to gray
  cvtColor( src, src_gray, CV_BGR2GRAY );
  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  vector<Vec3f> circles;
  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }
  /// Show your results
  imshow( "Hough Circle Transform Demo", src );
	char c = (char)waitKey(30);
        if( c == 27 )break;
  }
  return 0;
}

