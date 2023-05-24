// ****************************************************************************************************************************************
// Title            : delivery.cpp
// Description      : The script is responsible for resolving the sender and receiver information and determines the location setpoint and
//                    publishes it. It also publishes appropriate status signals.           
// Author           : Sowbhagya Lakshmi H T
// Last revised on  : 20/05/2023
// ****************************************************************************************************************************************

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include <wiringPi.h>
#include <ctime>
#include <csignal>
#include <softPwm.h>

// For sleep function
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

enum class ProgressStatusOptions
{
    done,
    in_progress,
    none,
};

enum class AvailabilityStatusOptions
{
    yes,
    no,
};

class Sender
{
public:
    std::string m_location{};
    ros::Subscriber m_locationSub{};

    Sender(ros::NodeHandle* n)
    {
        m_locationSub = n->subscribe("senderLocation", queue_size=1000, &Sender::location_callback, this);
    }

    void location_callback(const std_msgs::String::ConstPtr &name)
    {
        this->m_location = name->data.c_str();
    }
};

class Receiver
{
public:
    std::string m_location{};
    ros::Subscriber m_locationSub{};

    Receiver(ros::NodeHandle* n)
    {
        m_locationSub = n->subscribe("receiverLocation", queue_size=1000, &Receiver::location_callback, this);
    }

    void location_callback(const std_msgs::String::ConstPtr &name)
    {
        m_location = name->data.c_str();
    }
};

class SetPoint
{
public:
    bool m_isReachedSetPoint{0};  
    ros::Publisher m_setpointPub{};
    ros::Subscriber m_reachedSetpointSub{};
    int m_setpoint{-1};

    SetPoint(ros::NodeHandle* n)
    {
        m_setpointPub = n->advertise<std_msgs::Int8>("setpoint", queue_size=1000);
        m_reachedSetpointSub = n->subscribe("isReachedSetPoint", queue_size=10000, &SetPoint::reached_setpoint_callback, this);
    }

    void publish_setpoint(int sp)
    {
        std_msgs::Int8 value;
        value.data = sp;
        m_setpointPub.publish(value);
    }

    void reached_setpoint_callback(const std_msgs::Bool::ConstPtr &value)
    {   
        m_isReachedSetPoint = value->data;
    }

    bool find_setpoint(std::string location)
    {
        if (location.compare("Location A") == 0)
        {
            publish_setpoint(0);
        }
        else if (location.compare("Location B") == 0)
        {
            publish_setpoint(1);
        }
        else
        {
            publish_setpoint(-1);
            return 0;
        }

        return 1;
    }
};

class BotAvailability
{
public:
    ros::Publisher m_availabilityPub{};
    std::vector<std::string> m_availabilityStatusOptions{"yes", "no"};

    BotAvailability(ros::NodeHandle* n)
    {
        m_availabilityPub = n->advertise<std_msgs::String>("availability", queue_size=1000);
    }

    void set_availability_status(AvailabilityStatusOptions status)
    {
        std::stringstream strStreamAvailabilityStatus;
        strStreamAvailabilityStatus << m_availabilityStatusOptions[static_cast<int>(status)];

        std_msgs::String availabilityStatus;
        availabilityStatus.data = strStreamAvailabilityStatus.str();

        m_availabilityPub.publish(availabilityStatus);
    }
};

class ProgressStatus
{
public:
    ros::Publisher m_progressStatusPub{};
    std::vector<std::string> m_progressStatusOptions{"done", "in progress", "none"};

    ProgressStatus(ros::NodeHandle* n)
    {
        m_progressStatusPub = n->advertise<std_msgs::String>("progress", queue_size=1000);
    }

    void set_progress_status(ProgressStatusOptions status)
    {
        std::stringstream strStreamProgressStatus;
        strStreamProgressStatus << m_progressStatusOptions[static_cast<int>(status)];

        std_msgs::String progressStatus;
        progressStatus.data = strStreamProgressStatus.str();

        m_progressStatusPub.publish(progressStatus);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery");
    ros::NodeHandle n;

    BotAvailability botAvailability{&n};
    ProgressStatus progressStatus{&n};
    SetPoint setpoint{&n};
    Sender sender{&n};
    Receiver receiver{&n};

    ros::Rate loopRate(10); 

    time_t loopStartTime;
    time_t loopCurrTime;
    time_t timeDiff{0};

    std::cout << '\n';

    while (ros::ok())
    {

        time_t timeDiff{0};

        int senderCount{0};
        // Sender loop
        while (ros::ok())
        {
            bool isfoundSetpoint{setpoint.find_setpoint(sender.m_location)};            

            if (isfoundSetpoint)
            {   
                if (senderCount == 0)
                {
                    time(&loopStartTime);
                    senderCount++;
                }

                time(&loopCurrTime);
                timeDiff = loopCurrTime - loopStartTime;
                
		ROS_INFO("Found setpoint"); 
                botAvailability.set_availability_status(AvailabilityStatusOptions::no);
                progressStatus.set_progress_status(ProgressStatusOptions::in_progress);
            }

            sleep(2);   // Small wait in code to compensate for delay in data transmission
            ros::spinOnce();
            loopRate.sleep();

            if (timeDiff > 6)
            {
                ROS_INFO("Reached sender at %s", sender.m_location.c_str());
                break;
            }
        }

        // std::cout << "Receiver loop\n";
        time(&loopStartTime);

        // Receiver loop
        while (ros::ok())
        {
            int receiverCount{0};

            bool isfoundSetpoint{setpoint.find_setpoint(receiver.m_location)};

            sleep(1);
            ros::spinOnce();
            loopRate.sleep();

            time(&loopCurrTime);
            timeDiff = loopCurrTime - loopStartTime;

            if (timeDiff > 6)
            {
                progressStatus.set_progress_status(ProgressStatusOptions::done);
                botAvailability.set_availability_status(AvailabilityStatusOptions::yes);

                ROS_INFO("Reached receiver at %s", receiver.m_location.c_str());
                ROS_INFO("Completed delivery!");

                ros::spinOnce();
                loopRate.sleep();
                sleep(1);
                break;
            }
        }
        std::cout << '\n';
        sleep(4);
        ros::spinOnce();
        loopRate.sleep();
    }
}
