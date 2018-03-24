#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI;
using namespace dji_sdk;
using namespace DJI::onboardSDK;
using namespace std;

double C_EARTH=6378137.0;
double C_PI=3.1415926;

double TarLat = 22.0;
double TarLon = 40.000;
double thresh = 2.0;
double k_p = 1 ;
double v_max = 10;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

bool obtain_control(DJIDrone *drone)
{   
    uint8_t ctrlFlag;
    drone->request_sdk_permission_control();
    FlightControlInfo ctrlInfo = drone->flight_control_info;
    ctrlFlag = ctrlInfo.cur_ctrl_dev_in_navi_mode;
    if(ctrlFlag) 
        cout << "Control Permission Obtained"<<endl;
}

/*bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}*/

double deltaX(double TarLat, double OriLat)
{
    double ans;
    ans=C_EARTH*(TarLat-OriLat)*deg2rad;
    return ans;
}

double deltaY(double TarLat, double TarLon, double OriLon)
{
    double ans,deltaLon;
    TarLat = TarLat*deg2rad;
    deltaLon = TarLon - OriLon;
    ans = C_EARTH * cos(TarLat) * deltaLon * deg2rad;
    return ans;
}

double deltaDist(double Lat1, double Lon1, double Lat2, double Lon2)
{
    double ans;
    ans = C_EARTH * acos(sin(Lat1*deg2rad)*sin(Lat2*deg2rad) + cos(Lat1*deg2rad)*cos(Lat2*deg2rad) * cos(Lon1*deg2rad - Lon2*deg2rad));
    return ans;
}

void flightCtrl(DJIDrone *drone)
{
    double vx;
    double vy;
    unsigned char flag =
            Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_RATE |
            Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
            Flight::SmoothMode::SMOOTH_DISABLE;

    double errX, errY;
    errX = deltaX(TarLat, drone->global_position.latitude);
    errY = deltaY(TarLat, TarLon, drone->global_position.longitude);
    if((abs(errX) > thresh) && (abs(errY) > thresh) )
        {
            vx = k_p * errX;
            vy = k_p * errY;
            vx = (vx > v_max)? v_max : vx;
            vy = (vy > v_max)? v_max : vy;

            drone->attitude_control(flag, vx, vy, 0, 0);
            //usleep(2);
        }
    else   
        //drone->attitude_control(flag, 0, 0, 0 ,0);
        drone->landing();
}


int main(int argc, char *argv[])
{
	bool control_flag = false;
	bool err_flag = false;
    double vx;
    double vy;
    double errX, errY;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");

    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);
    unsigned char flag =
            Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_RATE |
            Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
            Flight::SmoothMode::SMOOTH_DISABLE;

    ros::Rate loop_rate(50);
    
    control_flag = obtain_control(drone);
    //drone->check_version();
    for(int i=0;i<3;i++) drone->takeoff();
    sleep(8);
     
    while(ros::ok())
    {
        /*static ros::Time start_time = ros::Time::now();
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        if (elapsed_time > ros::Duration(0.02))
        {
        start_time = ros::Time::now(); */
        //control_flag = obtain_control(drone);

        flightCtrl(drone);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
