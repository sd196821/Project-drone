#include <iostream>
#include <math.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;

static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void RED_hsv2binary(Mat & src,Mat & dst)
{
     int dilation_size=2;
     int erosion_size=1;
    //dst=Mat(src.rows,src.cols,CV_8UC1);
    dst=Mat::zeros(src.rows,src.cols,CV_8UC1);
    uchar h,s,v;

    for(int i=0;i<src.rows;i++)
    {

        for(int j=0;j<src.cols;j++)
        {
            h=src.at<uchar>(i,j*3);
            s=src.at<uchar>(i,j*3+1);
            v=src.at<uchar>(i,j*3+2);
            if(((h<15)||(h>220))&&((s>100)&&(v>100)))
            {
                dst.at<uchar>(i,j)=0;
            }
            else

             if((v>180)||(v<60))
            {
                dst.at<uchar>(i,j)=255;
            }
            else
            {
                dst.at<uchar>(i,j)=255;
            }
        }
    }
    //Mat elementofdilate = getStructuringElement( MORPH_RECT,
                                         //Size( 3, 3 )      );
    ///膨胀操作
    //dilate( dst, dst, elementofdilate );
    //dilate( dst, dst, elementofdilate );
    //dilate( dst, dst, elementofdilate );

    //Mat elementoferode = getStructuringElement( MORPH_RECT,
                                        //Size( 3, 3 ));

    /// 腐蚀操作
    //erode( dst, dst, elementoferode );
    //erode( dst, dst, elementoferode );
//    imshow( "Dilation Demo", dst );

}

static void findSquares( const Mat& image, vector<vector<Point> >& squares )  //void
{
    squares.clear();
    Point3f ret_val; 
    float detected_flag;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //vector<Mat> hsvSplit;
    Mat imgThresholded;
    Mat dst;
    int OutContourIdx;//Index of OutContours
    int InsideContourIdx;//Index of InsideContours

    //  --------------------------------------
    //  |       OutContour                   |
    //  |                                    |
    //  |     ---------------------          |
    //  |     |                   |          |
    //  |     | InsideContour     |          |
    //  |     |                   |          |
    //  |     ---------------------          |
    //  --------------------------------------
    //Mat srcImg;
    //image.copyTo(srcImg);
    cvtColor(image,dst,CV_BGR2HSV_FULL);

    //split(image,hsvSplit);
    //equalizeHist(hsvSplit[2],hsvSplit[2]);
    //merge(hsvSplit,image);

    RED_hsv2binary(dst,imgThresholded);
    imshow("filter image",imgThresholded);

    //imshow("filter image",dst);
    //inRange(imgThresholded,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
    //medianBlur(imgThresholded,imgThresholded,3);
    //imshow("blured image",imgThresholded);

    Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);//open
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);//close
    imshow("Morphed image",imgThresholded);

    // find contours and store them all as a list
    findContours(imgThresholded,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);

    //vector<Point> approx;
    vector<vector<Point> > contours_poly(contours.size());

    // test each contour
   for( int i = 0; i < contours.size(); i++ )
    {   
        approxPolyDP(Mat(contours[i]), contours_poly[i], arcLength(Mat(contours[i]), true)*0.02, true);

        if(contours_poly[i].size() == 4 && fabs(contourArea(Mat(contours_poly[i]))) > 1000 ) //&& isContourConvex(Mat(contours_poly[i]))
        {
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
               // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(contours_poly[i][j%4], contours_poly[i][j-2], contours_poly[i][j-1]));
                maxCosine = MAX(maxCosine, cosine);
             }

               // if cosines of all angles are small
               // (all angles are ~90 degree) then write quandrange
               // vertices to resultant sequence
            if( maxCosine < 0.3 )
            {
                //area=countArea(Mat(contours_poly[i]));
                //minArea=MIN(area,minArea);
                if (hierarchy[i][2] == -1 && hierarchy[i][3] != -1 )
                      squares.push_back(contours_poly[i]);
            }
      
            /*if (hierarchy[i][2] != -1 && hierarchy[i][3] == -1 )
                OutContourIdx = i;
            if (hierarchy[i][2] == -1 && hierarchy[i][3] != -1 )
                InsideContourIdx = i;*/

          //boundRect[i] = boundingRect(Mat(contours[i]));
          /*float rate_err=0.4;
          float rateLow=3-rate_err;
          float rateHigh=3+rate_err;
          //Rect r0=boundingRect(Mat(contours[i]));
          RotatedRect boundRect = minAreaRect(Mat(contours[i]));
          float rate = (float) boundRect.size.width / (float) boundRect.size.height;//ratio of width/height
          if(rate > rateLow && rate < rateHigh)
          {
              squares.push_back( contours_poly[i]);
              Point2f vertices[4];
              detected_flag = 1;
              //cout<<"center X: "<<boundRect.center.x<<"   center Y: "<<boundRect.center.y<<endl;
              boundRect.points(vertices);//rotated rectangle vertices calculation
              line(image,vertices[0],vertices[1],Scalar(0,255,0),5);
              line(image,vertices[1],vertices[2],Scalar(0,255,0),5);
              line(image,vertices[2],vertices[3],Scalar(0,255,0),5);
              line(image,vertices[3],vertices[0],Scalar(0,255,0),5);
              //ret_val.x = boundRect.center.x;
              //ret_val.y = boundRect.center.y;
              //ret_val.z = detected_flag * boundRect.size.width;
                //return ret_val;
           }*/
          //else
         //{
            //ret_val.x = 99999;
            //ret_val.y = 99999;
         //   detected_flag = 0;
            //return ret_val;
         //}
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
          polylines(image, &p, &n, 1, true, Scalar(255,0,0), 3, LINE_AA);
    }

    imshow("detected",image);
}

int main()
{
    vector<vector<Point> > squares;
    Mat img;
    //img=imread("/home/long/FlightPictures/saved/picture800.png");
  VideoCapture cap(1);

  while(cap.isOpened())
  {
    cap>>img;
    findSquares(img,squares);
    drawSquares(img,squares);
        
    imshow("out",img);
    waitKey(30);
  }
  cap.release();
    //waitKey(0);
  return 0;

}

