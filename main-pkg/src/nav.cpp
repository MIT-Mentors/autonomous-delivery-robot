// ****************************************************************************************************************************************
// Title            : main.cpp
// Description      : The script is responsible for navigating the robot to the setpoint after subscribing to it. It configures the
//                    raspberry pi and sends PWM signals to motor drivers to drive the motors.                
// Author           : Sowbhagya Lakshmi H T
// Last revised on  : 20/05/2023
// ****************************************************************************************************************************************

#include <cmath>
#include <ctime>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "softPwm.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "wiringPi.h"


#include <iostream>
#include <vector>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

class motorControl
{
public:
    int m_dirPin;
    int m_pwmPin;
    int m_speed;

    motorControl(int dirPin, int pwmPin)
    {
        // To control each motor, 2 signals are required
        // Dir signal = 0 or 1
        // PWM signal = 0 to 100

        m_dirPin = dirPin;
        m_pwmPin = pwmPin;

        // Set pin mode
        pinMode(m_dirPin, OUTPUT);
        pinMode(m_pwmPin, OUTPUT);

        // Create soft pwm
        int pwmInitialVal = 0;
        int pwmRange = 100;
        softPwmCreate(m_pwmPin, pwmInitialVal, pwmRange);
    }
};

// class SetPoint
// {
// public:
//     ros::Publisher m_reachedSetpointPub{};
//     ros::Subscriber m_setpointSub{};
//     int m_setpoint{-1};
    
//     SetPoint(ros::NodeHandle* n)
//     {
//         int queue_size = 1000;
//         m_reachedSetpointPub = n->advertise<std_msgs::Bool>("isReachedSetPoint",queue_size);
//         m_setpointSub = n->subscribe("setpoint", 1, &SetPoint::setpoint_callback, this);
//     }

//     void setpoint_callback(const std_msgs::Int8::ConstPtr &val)
//     {
//         m_setpoint = val->data;
//     }

//     void reachedSetpointStatus(int boolVal)
//     {
//         std_msgs::Bool value;
//         value.data = boolVal;
//         m_reachedSetpointPub.publish(value);
//     }
// };

class Navigation
{
public:
    const double m_maxSpeed {20};
    const double wheelRadius {0.165};
    const double lengthBtnWheels {0.8};
    
    int m_rightVelocity{0};
    int m_leftVelocity{0};

    double m_speed = 0.0;
    double m_steer = 0.0;

    motorControl* m_fwRight;
    motorControl* m_fwLeft;
    motorControl* m_bwRight;
    motorControl* m_bwLeft;

    Navigation(ros::NodeHandle* n, motorControl* fwRight, motorControl* fwLeft, motorControl* bwRight, motorControl* bwLeft)
    {
        m_fwRight = fwRight;
        m_fwLeft = fwLeft;
        m_bwRight = bwRight;
        m_bwLeft = bwLeft;

        ros::Subscriber speed_sub = n->subscribe("/speed",1,&Navigation::speed_callback, this);
        ros::Subscriber steer_sub = n->subscribe("/steer",1,&Navigation::steer_callback, this);
    }

    void navigate_to_point()
    {
       double desiredYaw = m_steer;

        double phiErrorUncorrected {-m_steer};
        double phiError {atan2(sin(phiErrorUncorrected), cos(phiErrorUncorrected))};

        // // Differential drive velocities
        double rightVelocity {((2.0*m_speed + phiError*lengthBtnWheels)/2.0*wheelRadius)*60/(2*3.14159)};
        double leftVelocity  {((2.0*m_speed - phiError*lengthBtnWheels)/2.0*wheelRadius)*60/(2*3.14159)};

        std::cout << rightVelocity << ' ' << leftVelocity << '\n';

        rightVelocity = rightVelocity > m_maxSpeed ? m_maxSpeed : rightVelocity;
        leftVelocity = leftVelocity > m_maxSpeed ? m_maxSpeed : leftVelocity;

        set_velocity(rightVelocity, leftVelocity);
    }

    void set_velocity(double rightVelocity, double leftVelocity)
    {

        for(int index=0; index<4; index++)
        {   
            // if (m_rightVelocity >=0)
            // {
                digitalWrite(m_fwRight->m_dirPin, HIGH);
                digitalWrite(m_fwLeft->m_dirPin, HIGH);
                digitalWrite(m_bwRight->m_dirPin, HIGH);
                digitalWrite(m_bwLeft->m_dirPin, HIGH);

            // }
            // else if (m_rightVelocity <0)
            // {
            //     digitalWrite(m_fwLeft->m_dirPin, LOW);
            //     digitalWrite(m_fwRight->m_dirPin, LOW);
            //     digitalWrite(m_bwRight->m_dirPin, LOW);
            //     digitalWrite(m_bwLeft->m_dirPin, LOW);
            // }
            
            softPwmWrite(m_fwRight->m_pwmPin, abs(rightVelocity));
            softPwmWrite(m_fwLeft->m_pwmPin, abs(leftVelocity));
            softPwmWrite(m_bwRight->m_pwmPin, abs(rightVelocity));
            softPwmWrite(m_bwLeft->m_pwmPin, abs(leftVelocity));
        }
    }

    void stopMotors()
    {
        m_rightVelocity = abs(m_rightVelocity);

        // Decreasing the motor velocity gradually
        while(m_rightVelocity >= 0)
        {

            softPwmWrite(m_fwRight->m_pwmPin, m_rightVelocity);
            softPwmWrite(m_fwLeft->m_pwmPin, m_rightVelocity);
            softPwmWrite(m_bwRight->m_pwmPin, m_rightVelocity);
            softPwmWrite(m_bwLeft->m_pwmPin, m_rightVelocity);

            m_rightVelocity--;
        }

        digitalWrite(m_fwRight->m_pwmPin, LOW);
        digitalWrite(m_fwLeft->m_pwmPin, LOW);
        digitalWrite(m_bwRight->m_pwmPin, LOW);
        digitalWrite(m_bwLeft->m_pwmPin, LOW);

        delay(3);
    }    

    double find_distance_between_points(double point1[3], double point2[3])
    {
        // Finding the distance between 2 points on a 2D plane using 
        // the formula d = √((x_2-x_1)² + (y_2-y_1)²). 
        // Here since the robot's x and z axes are parallel to the
        // ground we use those coordinates to calculate the distance.
        return sqrt((pow((point1[0]-point2[0]), 2)+pow((point1[2]-point2[2]), 2)));
    }

    void speed_callback(const std_msgs::Float64::ConstPtr &msg)
    {
        m_speed = msg->data;
    }

    void steer_callback(const std_msgs::Float64::ConstPtr &msg)
    {
        m_steer = msg->data;
    }
};

double lattitude = 0.0;
double longitude = 0.0;

void lattitude_callback(const std_msgs::Float64::ConstPtr &val)
{
    lattitude = val->data;
}

void longitude_callback(const std_msgs::Float64::ConstPtr &val)
{
    longitude = val->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    if(wiringPiSetup()<0)
    {
        std::cout<<"setup wiring pi failed"<<'\n';
        return 1;
    }

    // Defining the 4 motors using the pin numbers the respective
    // dir and PWM pins are connected to
    motorControl fwRight{5, 4};
    motorControl fwLeft{3, 2};
    motorControl bwRight{29, 28};
    motorControl bwLeft{25, 24};

    Navigation navigation{&n, &fwRight, &fwLeft, &bwRight, &bwLeft};
    ros::Rate loop_rate(10);

    time_t loopStartTime;
    time_t loopCurrTime;
    time_t timeDiff{0};

    ros::Publisher abs_pose_pub = n.advertise<nav_msgs::Odometry>("/absolute_pose",1000);
    ros::Publisher way_pts_pub = n.advertise<nav_msgs::Path>("/waypoints_input",1000);
    ros::Publisher ext_speed_pub = n.advertise<std_msgs::Float64>("/external_speed",1000);

    std_msgs::Float64 ext_speed;
    ext_speed.data = 1000.0;

    // Prepping waypoint data to be published for waypoint tracking controller package 
    nav_msgs::Path way_pts;     // Need to hard code this for testing purpose

    // ros::Time way_pts_time;
    way_pts.header.frame_id = "map";
    
    // Hardcoded waypoints = (-44.7, 0, 30.5), (-42.4, 0, 42.29), (-25.8, 0, 44), (-5.4, 0, 49.1)
    geometry_msgs::PoseStamped posestamp;

    posestamp.pose.position.x = 12.950877;
    posestamp.pose.position.y = 80.132106;
    posestamp.pose.position.z = 0.0;
    posestamp.pose.orientation.x = 0.0;
    posestamp.pose.orientation.y = 0.0;
    posestamp.pose.orientation.z = 0.0;
    posestamp.pose.orientation.w = 1.0;

    way_pts.header.stamp = ros::Time::now();
    way_pts.poses.push_back(posestamp);

    posestamp.pose.position.x = 12.9509462;
    posestamp.pose.position.y = 80.1318963;
    posestamp.pose.position.z = 0.0;
    posestamp.pose.orientation.x = 0.0;
    posestamp.pose.orientation.y = 0.0;
    posestamp.pose.orientation.z = 0.0;
    posestamp.pose.orientation.w = 1.0;

    way_pts.header.stamp = ros::Time::now();
    way_pts.poses.push_back(posestamp);

    nav_msgs::Odometry abs_pose;
    abs_pose.header.frame_id = "map";

    ros::Subscriber lat_sub = n.subscribe("/lattitude",1000,lattitude_callback);
    ros::Subscriber long_sub = n.subscribe("/longitude",1000,longitude_callback);


    while (ros::ok())
    {
        // if (setpoint.m_setpoint != -1)     //if the setpoint has been set, then true
        // {
        //     navigation.navigate_to_point(&setpoint);
        // }
        // else
        // {
        //     navigation.stopMotors();
        //     setpoint.reachedSetpointStatus(0);
        // }

        abs_pose.pose.pose.position.x = lattitude;  // X axis and Z axis are parallel to the ground in Webots
        abs_pose.pose.pose.position.y = longitude;
        abs_pose.pose.pose.orientation.x = 0;
        abs_pose.pose.pose.orientation.y = 0;
        abs_pose.pose.pose.orientation.z = 0;
        abs_pose.pose.pose.orientation.w = 1;

        abs_pose_pub.publish(abs_pose);         // Causes prblm in TF

        ext_speed_pub.publish(ext_speed);
        way_pts_pub.publish(way_pts);

        navigation.navigate_to_point();
        
        ros::spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
