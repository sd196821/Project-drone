#include <iostream>
#include <math.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

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
    //                                     Size( 3, 3 ),
    //                                     Point( dilation_size, dilation_size ) );
    ///膨胀操作
    //dilate( dst, dst, elementofdilate );

    //Mat elementoferode = getStructuringElement( MORPH_RECT,
    //                                     Size( 3, 2*erosion_size+1 ),
    //                                     Point( erosion_size, erosion_size ) );

    /// 腐蚀操作
//    erode( dst, dst, elementoferode );
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
    //Mat srcImg;
    //image.copyTo(srcImg);
    cvtColor(image,dst,CV_BGR2HSV_FULL);

    //split(image,hsvSplit);
    //equalizeHist(hsvSplit[2],hsvSplit[2]);
    //merge(hsvSplit,image);

    RED_hsv2binary(dst,imgThresholded);
    //imshow("filter image",dst);
    //inRange(imgThresholded,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
    Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);//open
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);//close
    //imshow("filter image",imgThresholded);

    // find contours and store them all as a list
    findContours(imgThresholded, contours,hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

    //vector<Point> approx;
    vector<vector<Point> > contours_poly(contours.size());
    //vector<Rect> boundRect(contours.size());

    // test each contour
   for( size_t i = 0; i < contours.size(); i++ )
    {   
        approxPolyDP(Mat(contours[i]), contours_poly[i], arcLength(Mat(contours[i]), true)*0.02,true);

       
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
                if (hierarchy[i][2] == -1 && hierarchy[i][3] != -1 )
                squares.push_back(contours_poly[i]);
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

    //imshow("detected",image);
    
}

int main()
{
    int size=2510;  //saved 2080; saved0 2510; saved1 1770
    Mat img;
    vector<vector<Point> > squares;

    for(int counter=10;counter <= size;counter=counter+10)
    {
        string imgName;
        imgName = "/home/long/FlightPictures/saved0/picture";
        stringstream ss0;
        string str0;
        ss0 << counter;
        ss0 >> str0;
        imgName = imgName + str0 + ".png";

        string outName;
        outName = "/home/long/FlightPictures/saved0/processed";
        outName = outName + str0 + ".png";

        img=imread(imgName);

        findSquares(img,squares);
        drawSquares(img,squares);
        
        imwrite(outName,img);

        if( waitKey(1) >=27 ) break;

       
    }
     return 0;
}
