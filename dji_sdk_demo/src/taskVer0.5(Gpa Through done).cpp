/******************************************
Gap recognition and flying-through code
*******************************************/
//Image : Corner counter need to be done检测角点个数，确定// 

#include <iostream>
#include <math.h>
#include <string.h>

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

double TarLat = 22.0;
double TarLon = 40.000;
double thresh_pixel = 20.0;
double thresh_gap = 1;
double k_p = 1 ;
double v_max = 0.8;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

int iLowH = 150;
int iHighH = 179;
int iLowS = 90;
int iHighS = 255;
int iLowV = 90;
int iHighV = 255;

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

static Point3f findSquares( const Mat& image, vector<vector<Point> >& squares )  //void
{
    squares.clear();
    Point3f ret_val; 
    float detected_flag;
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
    //morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);//close
    imshow("filter image",imgThresholded);

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
          float rate_err=0.4;
          float rateLow=2.5-rate_err;
          float rateHigh=2.5+rate_err;
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

    imshow("detected",image);
    waitKey(1);
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
    p = K_p * error_x + K_i * I_error_x + K_d * error_x - last_error_x);
    q = K_p * error_y + K_i * I_error_y + K_d * error_y - last_error_y);
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

    bool control_flag = false;
    bool err_flag = false;
    double vx = 0;
    double vy = 0;
    double vz = 0;
    float e_X,e_Y, e_Z;
    float GapRoute = 20;//distance to fly through Gap
    Point3f contoursCenter;//z equals to detected_flag
    contoursCenter.x = 99999;
    contoursCenter.y = 99999;
    contoursCenter.z = 0;
    //detected_flag = contoursCenter.z;
    
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
    
    control_flag = obtain_control(drone);
    //drone->check_version();
    //for(int i=0;i<6;i++) drone->takeoff();
    sleep(1);

    int ret,mode;
    mode = GETBUFFER_MODE|TRANSFER_MODE;
    ret = manifold_cam_init(mode);

    while(ros::ok())
    {
      //imageRead().copyTo(src);
      //medianBlur(src,src,3);
      //int ret;
      Mat image,camera;
      ret = manifold_cam_read(zbuffer, &nframe, 1);
      if(ret==-1)
          cout<<"Manifold initialization failed."<<endl;
      camera = Mat(IMAGE_H * 3 / 2, IMAGE_W, CV_8UC1, zbuffer);
      if(camera.empty())
          cout<<"Fail to acquire picture."<<endl;
      cvtColor(camera,camera,CV_YUV2BGR_NV12);
      image = camera(Range(0, 720), Range(160, 1120));

      contoursCenter = findSquares(image,squares);
      detected_flag = contoursCenter.z;
      drawSquares(image,squares);
    
      e_Y = deltaX_Local(contoursCenter.x, p_x);
      e_Z = -1 * deltaY_Local(contoursCenter.y, p_y);//coordinate frame transformation
      
      //**********flight through***********//
      //step 1 : target not detected, keeping hovering state
      if(detected_flag == 0)
      {
        cout<<"No target detected."<<endl;
        drone->attitude_control(flag, 0, 0, 0, 0);
        usleep(2);
      }
      else 
      {//step 2: target detected, adjust drone to aim at contours center
          if((abs(e_Y) > thresh_pixel) && (abs(e_Z) > thresh_pixel) )
            {
                detected_flag = 2;
                vy = k_p * e_Y;
                vz = k_p * e_Z;
                vy = (vy > v_max)? v_max : vy;
                vz = (vz > v_max)? v_max : vz;
                drone->attitude_control(flag, 0, vy, vz, 0);
                usleep(2);              
            }
            else 
            {//step 3:ready to fly through gap
                for(int counter = 1 ; counter < 11 ; counter++ )
                {
                    detected_flag = 3;
                    drone->attitude_control(flag, 0, 0, 0, 0);
                    usleep(100000);
                    cout << "Ready to fly through in %d us" << 1000-counter*100 <<endl;
                }//step 4:fly through
                float start_position = drone->local_position.x;
                float end_position = drone->local_position.x + GapRoute;
                e_X = GapRoute;//deltaX_Local(GapRoute, drone->local_position.x);//***some problem***//
                if(abs(e_X) > thresh_gap)
                {
                    vx = k_p * e_X;
                    vx = (vx > v_max)? v_max : vx;
                    drone->attitude_control(flag, vx, 0, 0, 0);
                    usleep(200000);
                    e_X = 10 - (drone->local_position.x - start_position);
                }//step 5: recover to hover state
                drone->attitude_control(flag, 0, 0, 0, 0);
                cout<<"Flying through gap done..."<<endl;
            }
      }
      

      if(waitKey(1) == 27) break;
      ros::spinOnce();
      loop_rate.sleep();
    }


  manifold_cam_exit();
  return 0;
}
