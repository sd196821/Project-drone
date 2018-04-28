/******************************************
Gap recognition and flying-through code
*******************************************/
//Image : Corner counter need to be done检测角点个数，确定// 

#include <iostream>
#include <math.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>

//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "djicam.h"

using namespace DJI;
using namespace dji_sdk;
using namespace DJI::onboardSDK;
using namespace std;
using namespace cv;

#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3

unsigned char zbuffer[FRAME_SIZE] = {0};
unsigned int frame_size = 0;
unsigned int nframe = 0;

double C_EARTH=6378137.0;
double C_PI=3.1415926;

//double TarLat = 22.0;
//double TarLon = 40.000;
double thresh_pixel;
double thresh_gap;
double k_p;
double k_d;
double k_i;
double v_max;
double iLowH;
double iHighH;
double iLowS;
double iHighS;
double iLowV;
double iHighV;
/*double thresh_pixel = 20.0;
double thresh_gap = 1;
double k_p = 1 ;
double v_max = 0.8;*/

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

/*int iLowH = 150;
int iHighH = 179;
int iLowS = 90;
int iHighS = 255;
int iLowV = 90;
int iHighV = 255;*/

// not detected == 0,target detected == 1 ,approaching target == 2, ready to fly == 3
//int detected_flag = 0;

int manifold_cam_read(unsigned char *buffer, unsigned int *nframe, unsigned int block);
int manifold_cam_init(int mode);
int manifold_cam_exit();

bool obtain_control(DJIDrone *drone)
{   
    uint8_t ctrlFlag;
    drone->request_sdk_permission_control();
    FlightControlInfo ctrlInfo = drone->flight_control_info;
    ctrlFlag = ctrlInfo.cur_ctrl_dev_in_navi_mode;
    if(ctrlFlag != 0) 
        cout << "Control Permission Obtained"<<endl;
}


float deltaX_Local(float TarX, float OriX)
{
    float ans;
    ans = TarX-OriX;
    return ans;
}

float deltaY_Local(float TarY, float OriY)
{
    float ans;
    ans = TarY - OriY;
    return ans;
}

double deltaX_Global(double TarLat, double OriLat)
{
    double ans;
    ans=C_EARTH*(TarLat-OriLat)*deg2rad;
    return ans;
}

double deltaY_Global(double TarLat, double TarLon, double OriLon)
{
    double ans,deltaLon;
    TarLat = TarLat*deg2rad;
    deltaLon = TarLon - OriLon;
    ans = C_EARTH * cos(TarLat) * deltaLon * deg2rad;
    return ans;
}


/*Mat imageRead()
{
      Mat image,camera;
      ret = manifold_cam_read(zbuffer, &nframe, 1);
      if(ret==-1)
          return;
      camera = Mat(IMAGE_H * 3 / 2, IMAGE_W, CV_8UC1, zbuffer);
      if(camera.empty())
          return;
      cvtColor(camera,camera,CV_YUV2BGR_NV12);
      image = camera(Range(0, 720), Range(160, 1120));
      return image;
}*/

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
    Mat elementofdilate = getStructuringElement( MORPH_RECT,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    ///膨胀操作
    dilate( dst, dst, elementofdilate );

    Mat elementoferode = getStructuringElement( MORPH_RECT,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    /// 腐蚀操作
//    erode( dst, dst, elementoferode );
//    imshow( "Dilation Demo", dst );

}

static Point3f findSquares( const Mat& image, vector<vector<Point> >& squares )  //void
{
    squares.clear();
    Point3f ret_val; 
    float detected_flag;
    vector<vector<Point> > contours;
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

    //Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
    //morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);//open
    //morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);//close
    //imshow("filter image",imgThresholded);

    // find contours and store them all as a list
    findContours(imgThresholded, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    //vector<Point> approx;
    vector<vector<Point> > contours_poly(contours.size());
    //vector<Rect> boundRect(contours.size());

    // test each contour
   for( size_t i = 0; i < contours.size(); i++ )
    {	
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);

        if(fabs(contourArea(Mat(contours_poly[i]))) > 1000 ) //&& isContourConvex(Mat(contours_poly[i]))
        {
	  
          //boundRect[i] = boundingRect(Mat(contours[i]));
          float rate_err=0.4;
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
              line(image,vertices[0],vertices[1],Scalar(0,0,255));
              line(image,vertices[1],vertices[2],Scalar(0,0,255));
              line(image,vertices[2],vertices[3],Scalar(0,0,255));
              line(image,vertices[3],vertices[0],Scalar(0,0,255));
              cout<<"center X: "<<boundRect.center.x<<"  Center Y:  "<<boundRect.center.y<<endl;
              ret_val.x = boundRect.center.x;
              ret_val.y = boundRect.center.y;
              ret_val.z = detected_flag ;
              return ret_val;
          }
         else
         {
            ret_val.x = 99999;
            ret_val.y = 99999;
            detected_flag = 0;
            return ret_val;
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

    //imshow("detected",image);
    //waitKey(1);
}

FILE *file;
ostringstream stream;
string str;
void addColumnTitle()
{
    stream <<
              ///data from flight controllor
              "timeStamp"
           << "\t"
           << "imgCounter"
           << "\t"
           << "contours center X"
           << "\t"
           << "contours center Y"
           << "\t"
           <<"detected flag "
           << "\t"
           << endl;
    str = string(stream.str());
    fputs(str.c_str(), file);
    stream.str("");
}

void CreateFile()
{
    ros::Time now=ros::Time::now();
    boost::posix_time::ptime now1=now.toBoost();
    string str2=to_simple_string(now1);
    file = fopen(str2.c_str(),"a");
}

void saveData(int counter, Point3f data)
{
    clock_t start=clock();
    stream << setprecision(10) <<
              ///data from flight controllor
              start*1000/CLOCKS_PER_SEC  << "\t" << counter << "\t" << data.x << "\t" << data.y << "\t" << data.z << "\t"
              <<endl;
    str = string(stream.str());
    fputs(str.c_str(), file);
    stream.str("");
}
//PID controller
/*static float error_x =0,last_error_x = 0,error_y = 0,last_error_y = 0,I_error_x = 0,I_error_y = 0;
static float K_p = 0.8,K_i = 0.001,K_d = 0.1;
pair<float,float> PID_error(float x,float y)
{
    float p = 0, q = 0;
    pair<float,float> error(0.0,0.0);
    error_x = x;
    error_y = y;
    I_error_x += error_x;
    I_error_y += error_y;
    p = K_p * error_x + K_i * I_error_x + K_d * (error_x - last_error_x);
    q = K_p * error_y + K_i * I_error_y + K_d * (error_y - last_error_y);
    last_error_x = error_x;
    last_error_y = error_y;
    error.first = q;
    error.second = p;
    return error;
}*/





int main(int argc, char** argv)
{ 

    float p_x = 1981.56522387247 * 6 / 25.0;//pixel center x
    float p_y = 1482.94787976315 * 6 / 25.0;//pixel center y
    Mat src;
    vector<vector<Point> > squares;
    float detected_flag;
    float last_detected_flag = 0;
    float tmp_flag = 0;

    bool control_flag = false;
    bool err_flag = false;
    double vx = 0;
    double vy = 0;
    double vz = 0;
    float e_X = 0,e_Y = 0,e_Z = 0;
    float e_X_last,e_Y_last,e_Z_last;
    float e_X_sum = 0,e_Y_sum = 0,e_Z_sum = 0;
    double GapVel;//distance to fly through Gap
    Point3f contoursCenter;//z equals to detected_flag
    contoursCenter.x = 99999;
    contoursCenter.y = 99999;
    contoursCenter.z = 0;
    //detected_flag = contoursCenter.z;

    //parameters read from .yaml file
    /*FileStorage fs2("param.yaml",FileStorage::READ);
    fs2["GapRoute"] >> GapRoute;
    fs2["thresh_pixel0"] >> thresh_pixel;
    fs2["thresh_gap"] >> thresh_gap;
    fs2["k_p"] >> k_p;
    fs2["v_max"] >> v_max;
    fs2["iLowH"] >> iLowH;
    fs2["iHighH"] >> iHighH;
    fs2["iLowS"] >> iLowS;
    fs2["iHighS"] >> iHighS;
    fs2["iLowV"] >> iLowV;
    fs2["iHighV"] >> iHighV;*/
   
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");

    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);  //dynamic storation space allocation
    unsigned char flag =
            Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_RATE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_DISABLE;

    ros::Rate loop_rate(50);

    //FileStorage fs("/home/ubuntu/catkin_ws/src/dji_sdk_demo/src/status.yaml",FileStorage:: WRITE);
    CreateFile();
    addColumnTitle();

    ros::NodeHandle nh1("~");
    nh1.param("thresh_gap",thresh_gap,1.0);
    nh1.param("thresh_pixel",thresh_pixel,20.0);
    nh1.param("k_p",k_p,0.5);
    nh1.param("k_d",k_d,0.05);
    nh1.param("k_i",k_i,0.005);
    nh1.param("v_max",v_max,0.3);
    nh1.param("GapVel",GapVel,0.1);
    nh1.param("iLowH",iLowH,150.0);
    nh1.param("iHighH",iHighH,179.0);
    nh1.param("iLowS",iLowS,90.0);
    nh1.param("iHighS",iHighS,255.0);
    nh1.param("iLowV",iLowV,90.0);
    nh1.param("iHighV",iHighV,255.0);
    
    control_flag = obtain_control(drone);
    //drone->check_version();
    //for(int i=0;i<6;i++) drone->takeoff();
    //sleep(1);

    int ret,mode;
    mode = GETBUFFER_MODE|TRANSFER_MODE;
    ret = manifold_cam_init(mode);

    int imgCounter=1;

    while(ros::ok())
    {
      imgCounter++;

      string imgName;
      imgName = "/media/ubuntu/3132-3731/saved/picture";
      stringstream ss0;
      string str0;
      ss0 << imgCounter;
      ss0 >> str0;

      imgName = imgName + str0 + ".png";
      //imageRead().copyTo(src);
      //medianBlur(src,src,3);
      //int ret;
      drone->request_sdk_permission_control();
      usleep(10);
      Mat image,camera;
      ret = manifold_cam_read(zbuffer, &nframe, 1);
      if(ret==-1)
          cout<<"Manifold initialization failed."<<endl;
      camera = Mat(IMAGE_H * 3 / 2, IMAGE_W, CV_8UC1, zbuffer);
      if(camera.empty())
          cout<<"Fail to acquire picture."<<endl;
      cvtColor(camera,camera,CV_YUV2BGR_NV12);
      image = camera(Range(0, 720), Range(160, 1120));
      if (imgCounter % 50 == 0)
          imwrite(imgName,image);

      last_detected_flag += tmp_flag;

      contoursCenter = findSquares(image,squares);
      detected_flag = contoursCenter.z;
      tmp_flag = detected_flag;
      drawSquares(image,squares);  
      saveData(imgCounter, contoursCenter);

      //fs<<"contours center X:  "<<contoursCenter.x;
      //fs<<"                Y:  "<<contoursCenter.y;
          
      //**********flight through***********//
      //``````step 1 : target not detected, keeping hovering state
      if((detected_flag == 0) && (last_detected_flag <= 10))
      {
        //cout<<"No target detected."<<endl;
        drone->attitude_control(flag, 0, 0, 0, 0);
        usleep(2);
        //fs<<"detected flag: "<<detected_flag;
      }
      else 
      {//``````step 2: target detected, adjust drone to aim at contours center
          cout<<"Target found."<<endl;

          e_Y_last = e_Y;
          e_Z_last = e_Z;

          e_Y = deltaX_Local(contoursCenter.x, p_x);
          e_Z = -1 * deltaY_Local(contoursCenter.y, p_y);//coordinate frame transformation
          
          e_Y_sum += e_Y;
          e_Z_sum += e_Z; 

          if((abs(e_Y) > thresh_pixel) && (abs(e_Z) > thresh_pixel) )
            {
                detected_flag = 2;
                vy = k_p * e_Y + k_d * (e_Y - e_Y_last) + k_i * e_Y_sum;
                vz = k_p * e_Z + k_d * (e_Z - e_Z_last) + k_i * e_Z_sum;
                vy = (abs(vy) > v_max)? v_max*abs(vy)/vy : vy;
                vz = (abs(vz) > v_max)? v_max*abs(vz)/vz : vz;
                drone->attitude_control(flag, 0, vy, vz, 0);
                //fs<<"detected flag: "<<detected_flag;
                usleep(2);              
            }
            else 
            {//`````````step 3:ready to fly through gap
                detected_flag = 3;
                //drone->attitude_control(flag, 0, 0, 0, 0);
                //usleep(100000);
                cout << "Ready to fly through " <<endl;
                //``````step 4:fly through
                //fs<<"detected flag: "<<detected_flag;
                //float start_position = drone->local_position.x;
                //float end_position = drone->local_position.x + GapRoute;
                //e_X = GapRoute;//deltaX_Local(GapRoute, drone->local_position.x);some problem?

                //if(abs(e_X) > thresh_gap)
                //{
                    //vx = k_p * e_X;
                    //vx = (vx > v_max)? v_max : vx;
                drone->attitude_control(flag, GapVel, 0, 0, 0);
                    //usleep(80000);
                    //e_X = 10 - (drone->local_position.x - start_position);
                //}//step 5: recover to hover state
                //drone->attitude_control(flag, 0, 0, 0, 0);
                cout<<"Flying through gap ..."<<endl;
            }
      }

      //fs2.release();


      if(waitKey(1) == 27) break;
      ros::spinOnce();
      loop_rate.sleep();
    }

  //fs.release();
  manifold_cam_exit();
  return 0;
}
