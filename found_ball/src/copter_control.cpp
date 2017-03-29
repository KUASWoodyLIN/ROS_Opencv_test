#include <ros/ros.h>
#include <signal.h>

//MAVROS
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

//found_ball
#include "found_ball/Ballinfo.h"

// Output
#include <iostream>

using namespace std;

//函數聲明
void imagedistance(const found_ball::BallinfoConstPtr &msg);
void mavrosStateCb(const mavros_msgs::StateConstPtr &msg);
void mavrosglobal(const sensor_msgs::NavSatFixConstPtr &msg);
void mavrosrelalti(const std_msgs::Float64ConstPtr &msg);
void exit_control (int sig);

#define FACTOR  0.65
#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

// Subscriber Relative Altitude
ros::Subscriber mavros_Altitude;

// Subscriber to global
ros::Subscriber mavros_global_sub;

// Subscriber to flight mode
ros::Subscriber mavros_state_sub;

// Subscriber image distance
ros::Subscriber Distance_sub;

// RC publisher
ros::Publisher rc_pub;

//MODE Service
ros::ServiceClient mode_ser;

//arm throttle
ros::ServiceClient arming_ser;

//takeoff
ros::ServiceClient takeoff_ser;

// Time control
ros::Time lastTime;

// mavros msgs
mavros_msgs::CommandTOL srv_takeoff;	// takeoff
mavros_msgs::CommandBool srv;		// arm throttle
mavros_msgs::SetMode srv_setMode;	// set MODE
mavros_msgs::OverrideRCIn rc_msg;	// rc override

//Throttle direction
double Roll, Pitch;

//Global position message
float latitude = 0 ,longitude = 0 ,altitude = 0 ;

//Relative Altitude
float Rel_altitude = 0;

//image info message
float ErX = 0.0 , ErY = 0.0;
bool ball_state = false ;

// Flight mode
std::string mode;
bool guided;
bool armed;
bool MODE = true;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "copter_control");
    ros::NodeHandle nh;

    // Subscribe
    Distance_sub = nh.subscribe("/ball/info", 1, imagedistance);
    mavros_state_sub = nh.subscribe("/mavros/state", 1, mavrosStateCb);
    mavros_global_sub = nh.subscribe("/mavros/global_position/global", 1, mavrosglobal);
    mavros_Altitude = nh.subscribe("/mavros/global_position/rel_alt", 1, mavrosrelalti);

    // Publishe
    rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    //Service
    mode_ser = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_ser = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_ser = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    // set GUIDED MODE
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if(mode_ser.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    // arm throttle
    srv.request.value = true;
    if(arming_ser.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    // takeoff
    srv_takeoff.request.altitude = 5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_ser.call(srv_takeoff)){
        ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    while( Rel_altitude < 5 ){
	sleep(1);
	ROS_ERROR("Altitude = %f" , Rel_altitude);
	ros::spinOnce();
    }
    ROS_ERROR("Altitude = %f" , Rel_altitude);

    // set AUTO MODE
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "AUTO";
    if(mode_ser.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        //return -1;
    }

    signal(SIGINT,exit_control);

    ros::spin();

}

void imagedistance(const found_ball::BallinfoConstPtr &msg)
{
    // Get message
    ErX = msg->x;
    ErY = msg->y;
    ball_state = msg->ball_state;

    // Found Ball
    if ( ball_state == true )
    {
	if( mode == "AUTO" )
	{
            // set LOITER MODE
    	    srv_setMode.request.base_mode = 0;
    	    srv_setMode.request.custom_mode = "LOITER";
            if(mode_ser.call(srv_setMode))
	    {
            	ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    	    }
	    else
	    {
       	    	ROS_ERROR("Failed SetMode");
    	    }
	}

        // Calculate Roll and Pitch depending on the mode
        Roll = BASERC - ErX * FACTOR;
        Pitch = BASERC - ErY * FACTOR;

        // Limit the Roll
        if (Roll > MAXRC)
        {
            Roll = MAXRC;
        }
	else if (Roll < MINRC)
        {
            Roll = MINRC;
        }

        // Limit the Pitch
        if (Pitch > MAXRC)
        {
            Pitch = MAXRC;
        }
	else if (Pitch < MINRC)
        {
            Pitch = MINRC;
        }

        rc_msg.channels[0] = Roll;     //Roll
        rc_msg.channels[1] = Pitch;    //Pitch
        rc_msg.channels[2] = BASERC;   //Throttle
        rc_msg.channels[3] = 0;        //Yaw
        rc_msg.channels[4] = 0;
        rc_msg.channels[5] = 0;
       	rc_msg.channels[6] = 0;
        rc_msg.channels[7] = 0;

        rc_pub.publish(rc_msg);
        cout << "Pitch:" << Pitch << endl;
        cout << "Roll:" << Roll << endl;
	ROS_INFO("Longitude: %f Latitude: %f Altitude: %f ", longitude , latitude , altitude );
    }
    else if( ball_state == false && mode == "LOITER" )
    {
    	// set AUTO MODE
    	srv_setMode.request.base_mode = 0;
    	srv_setMode.request.custom_mode = "AUTO";
    	if(mode_ser.call(srv_setMode)){
            ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
	    rc_msg.channels[0] = 0; //Roll
            rc_msg.channels[1] = 0; //Pitch
            rc_msg.channels[2] = 0; //Throttle
            rc_msg.channels[3] = 0; //Yaw
            rc_msg.channels[4] = 0;
            rc_msg.channels[5] = 0;
            rc_msg.channels[6] = 0;
            rc_msg.channels[7] = 0;
            rc_pub.publish(rc_msg);
    	}else{
       	    ROS_ERROR("Failed SetMode");
    	}
    }
    else if ( mode == "RTL"  || mode == "STABILIZE" || mode == "LAND" )
    {
	    rc_msg.channels[0] = 0; //Roll
            rc_msg.channels[1] = 0; //Pitch
            rc_msg.channels[2] = 0; //Throttle
            rc_msg.channels[3] = 0; //Yaw
            rc_msg.channels[4] = 0;
            rc_msg.channels[5] = 0;
            rc_msg.channels[6] = 0;
            rc_msg.channels[7] = 0;
            rc_pub.publish(rc_msg);
    }
}

void mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("Heard State: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided==128;
    armed = msg->armed==128;
}

void mavrosglobal(const sensor_msgs::NavSatFixConstPtr &msg)
{
    longitude = msg->longitude;
    latitude = msg->latitude;
    altitude = msg->altitude;
    //ROS_INFO("Heard lobal: [%f] [%f] [%f]", longitude, latitude, altitude);
}

void mavrosrelalti(const std_msgs::Float64ConstPtr &msg)
{
    Rel_altitude = msg->data;
}

void exit_control (int sig)
{
        cout<<"exit_control"<<endl;

        rc_msg.channels[0] = 0; //Roll
        rc_msg.channels[1] = 0; //Pitch
        rc_msg.channels[2] = 0; //Throttle
        rc_msg.channels[3] = 0; //Yaw
        rc_msg.channels[4] = 0;
        rc_msg.channels[5] = 0;
        rc_msg.channels[6] = 0;
        rc_msg.channels[7] = 0;

        rc_pub.publish(rc_msg);

        ros::shutdown();
}
