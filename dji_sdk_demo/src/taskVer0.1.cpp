#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

#define C_EARTH (double) 6378137.0;

double TarLat = 22.0;
double TarLon = 40.000;
double thresh = 2.0;
double k_p = 1 ;
double v_max = 10;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

double deltaX(double TarLat, double OriLat)
{
    double ans;
    abs = C_EARTH * (TarLat - OriLat) * deg2rad;
    return ans;
}

double deltaY(double TarLat, double TarLon, double OriLon)
{
    double ans,deltaLon;
    Lat = Lat*deg2rad;
	deltaLon = TarLon - OriLon;
    ans = C_EARTH * cos(TarLat) * deltaLon * deg2rad;
    return ans;
}

double deltaDist(double Lat1, double Lon1, double Lat2, double Lon2)
{
	double ans;
	ans = C_EARTH * acos(sin(lat1*deg2rad) * sin(lat2*deg2rad) + cos(lat1*deg2rad) * cos(lat2*deg2rad) * cos(lng1*deg2rad - lng2*deg2rad));
	return ans;
}

int flightCtrl()
{
	unsigned char flag =
            Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_RATE |
            Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
            Flight::SmoothMode::SMOOTH_DISABLE;
	
	drone->check_version;
	drone->request_sdk_permission_control();
	drone->takeoff();
	sleep(8);

	double errX, errY;
        errX = deltaX(TarLat, drone->global_position.latitude);
	errY = deltaY(TarLat, TarLon, drone->global_position.longitude);
	if((std::abs(errX) > thresh) && (std::abs(errY) > thresh) )
	{
		vx = k_p * errX;
		vy = k_p * errY;
		vx = (vx > v_max)? v_max : vx;
		vy = (vy > v_max)? v_max : vy;

		drone->attitude_control(flag, vx, vy, 0, 0);
		usleep();
	}
	else drone->attitude_control(flag, 0, 0, 0 ,0);
	drone->landing();
	//drone->drone_disarm();

}

int main(int argc, char *argv[])
{
	bool valid_flag = false;
	bool err_flag = false;
	ros::init(argc, argv, "sdk_client");
	ROS_INFO("sdk_service_client_test");
	ros::NodeHandle nh;
	DJIDrone* drone = new DJIDrone(nh);

	ros::spinOnce();

	flightCtrl();

	ros::spin();

	return 0;
	
}
