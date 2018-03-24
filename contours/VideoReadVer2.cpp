#include <iostream>
#include <math.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int iLowH = 150;
int iHighH = 179;
int iLowS = 90;
int iHighS = 255;
int iLowV = 90;
int iHighV = 255;

static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    vector<vector<Point> > contours;
    //vector<Mat> hsvSplit;
    Mat imgThresholded;
    //Mat srcImg;
    //image.copyTo(srcImg);
    cvtColor(image,imgThresholded,COLOR_BGR2HSV);

    //split(image,hsvSplit);
    //equalizeHist(hsvSplit[2],hsvSplit[2]);
    //merge(hsvSplit,image);

    inRange(imgThresholded,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);

    Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);//open
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);//close


    // find contours and store them all as a list
    findContours(imgThresholded, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    //vector<Point> approx;
    vector<vector<Point> > contours_poly(contours.size());
    //vector<Rect> boundRect(contours.size());

    // test each contour
   for( size_t i = 0; i < contours.size(); i++ )
    {	
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);

        if(fabs(contourArea(Mat(contours_poly[i]))) > 1000 && isContourConvex(Mat(contours_poly[i])))
        {
	  
          //boundRect[i] = boundingRect(Mat(contours[i]));
          float rate_err=0.2;
          float rateLow=2.5-rate_err;
          float rateHigh=2.5+rate_err;
          //Rect r0=boundingRect(Mat(contours[i]));
          RotatedRect boundRect = minAreaRect(Mat(contours[i]));
          float rate = (float) boundRect.size.width / (float) boundRect.size.height;//ratio of width/height
          if(rate > rateLow && rate < rateHigh)
           {
              squares.push_back( contours_poly[i]);
              Point2f vertices[4];
              cout<<"center X: "<<boundRect.center.x<<"   center Y: "<<boundRect.center.y<<endl;
              boundRect.points(vertices);//rotated rectangle vertices calculation
              line(image,vertices[0],vertices[1],Scalar(0,0,255));
              line(image,vertices[1],vertices[2],Scalar(0,0,255));
              line(image,vertices[2],vertices[3],Scalar(0,0,255));
              line(image,vertices[3],vertices[0],Scalar(0,0,255));
            }
        }
     }

}

static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];

        int n = (int)squares[i].size();
        //dont detect the border
        if (p-> x > 3 && p->y > 3)
          polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
    }

    imshow("detected",image);
}




int main(int argc, char** argv)
{	
    VideoCapture cap(1);

	while(cap.isOpened())
	{
		Mat src;//=imread("RedRect.png");
        //imshow("l",src);
		vector<vector<Point> > squares;
        //vector<Point> approx;
		cap>>src;

        //medianBlur(src,src,3);
        findSquares(src,squares);
        drawSquares(src,squares);


	if(waitKey(30)>=0) break;
	}

	cap.release();
    //waitKey(0);
	return 0;
}
