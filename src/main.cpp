#include <cmath>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <ctime>
#include <csignal>
#include <softPwm.h>

class motorControl
{
public:
    int m_dirPin;
    int m_pwmPin;
    int m_speed;

    motorControl(int dirPin, int pwmPin)
    {
        m_dirPin = dirPin;
        m_pwmPin = pwmPin;

        // Set pin mode
        pinMode(m_dirPin, OUTPUT);
        pinMode(m_pwmPin, OUTPUT);

        // Create soft pwm
        softPwmCreate(m_pwmPin, 0, 100);
    }
};

class SetPoint
{
public:
    ros::Publisher m_reachedSetpointPub{};
    ros::Subscriber m_setpointSub{};
    int m_setpoint{-1};
    
    SetPoint(ros::NodeHandle* n)
    {
        m_reachedSetpointPub = n->advertise<std_msgs::Bool>("isReachedSetPoint", 1000);
        m_setpointSub = n->subscribe("setpoint", 1, &SetPoint::setpoint_callback, this);
    }

    void setpoint_callback(const std_msgs::Int8::ConstPtr &val)
    {
        m_setpoint = val->data;
    }

    void reachedSetpointStatus(int boolVal)
    {
        std_msgs::Bool value;
        value.data = boolVal;
        m_reachedSetpointPub.publish(value);
    }
};

class Navigation
{
public:
    const int m_maxSpeed {15};
    int m_rightVelocity{0};
    int m_leftVelocity{0};

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
    }

    void navigate_to_point(SetPoint* setpoint)
    {
        if (setpoint->m_setpoint == 0)  // Location A
        {
            m_rightVelocity = 15;
            m_leftVelocity = 15;

            set_velocity();
        }
        else if (setpoint->m_setpoint == 1)     // Location B
        {
            m_rightVelocity = -15;
            m_leftVelocity = -15;

            set_velocity();
        }
    }

    void set_velocity()
    {

        for(int index=0; index<4; index++)
        {   
            if (m_rightVelocity >=0)
            {
                digitalWrite(m_fwRight->m_dirPin, HIGH);
                digitalWrite(m_fwLeft->m_dirPin, HIGH);
                digitalWrite(m_bwRight->m_dirPin, HIGH);
                digitalWrite(m_bwLeft->m_dirPin, HIGH);

            }
            else if (m_rightVelocity <0)
            {
                digitalWrite(m_fwLeft->m_dirPin, LOW);
                digitalWrite(m_fwRight->m_dirPin, LOW);
                digitalWrite(m_bwRight->m_dirPin, LOW);
                digitalWrite(m_bwLeft->m_dirPin, LOW);
            }
            
            softPwmWrite(m_fwRight->m_pwmPin, abs(m_rightVelocity));
            softPwmWrite(m_fwLeft->m_pwmPin, abs(m_rightVelocity));
            softPwmWrite(m_bwRight->m_pwmPin, abs(m_rightVelocity));
            softPwmWrite(m_bwLeft->m_pwmPin, abs(m_rightVelocity));
        }
    }

    void stopMotors()
    {
        m_rightVelocity = abs(m_rightVelocity);

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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    if(wiringPiSetup()<0)
    {
        std::cout<<"setup wiring pi failed"<<'\n';
        return 1;
    }

    motorControl fwRight{5, 4};
    motorControl fwLeft{3, 2};
    motorControl bwRight{29, 28};
    motorControl bwLeft{25, 24};

    Navigation navigation{&n, &fwRight, &fwLeft, &bwRight, &bwLeft};
    SetPoint setpoint(&n);

    ros::Rate loop_rate(10);

    time_t loopStartTime;
    time_t loopCurrTime;
    time_t timeDiff{0};

    while (ros::ok())
    {
        if (setpoint.m_setpoint != -1)     //if the setpoint has been set, then true
        {
            navigation.navigate_to_point(&setpoint);
        }
        else
        {
            navigation.stopMotors();
            setpoint.reachedSetpointStatus(0);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
