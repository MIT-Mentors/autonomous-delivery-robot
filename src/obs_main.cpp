#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <math.h>
#include <string>
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


int flag = 0; 

// flag = 0 -> Go Forward
// flag = 1 -> right
// flag = 2 -> left
// flag = 3 -> u turn

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

class Navigation
{
public:
    const int m_maxSpeed {30};
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

        digitalWrite(m_fwRight->m_dirPin, HIGH);
        digitalWrite(m_fwLeft->m_dirPin, HIGH);
        digitalWrite(m_bwRight->m_dirPin, HIGH);
        digitalWrite(m_bwLeft->m_dirPin, HIGH);
    }

    void navigate_to_point()
    {
        if (flag == 0)
        {
            m_rightVelocity = 15;
            m_leftVelocity = 15;
        }
        else if (flag == 1)     //right
        {
            m_rightVelocity = 5;
            m_leftVelocity = 40;
        }
        else if (flag == 2)     //left
        {
            m_rightVelocity = 40;
            m_leftVelocity = 5;
        }
        else if (flag == 3)     //U turn
        {
            m_rightVelocity = 0;
            m_leftVelocity = 0;
        }

        else
        {
            std::cout << "Unexpected\n";
        }

        // m_rightVelocity = 5;
        // m_leftVelocity = 30;

        set_velocity();
    }

    void set_velocity()
    {

        // for(int index=0; index<4; index++)
        // {   
            // if (m_rightVelocity >=0)
            // {
            //     std::cout << "Hello\n";
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
            
                // std::cout << "Hello\n";

            softPwmWrite(m_fwRight->m_pwmPin, abs(m_rightVelocity));
            softPwmWrite(m_fwLeft->m_pwmPin, abs(m_leftVelocity));
            softPwmWrite(m_bwRight->m_pwmPin, abs(m_rightVelocity));
            softPwmWrite(m_bwLeft->m_pwmPin, abs(m_leftVelocity));
        // }
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

    // double find_distance_between_points(double point1[3], double point2[3])
    // {
    //     // Finding the distance between 2 points on a 2D plane using 
    //     // the formula d = √((x_2-x_1)² + (y_2-y_1)²). 
    //     // Here since the robot's x and z axes are parallel to the
    //     // ground we use those coordinates to calculate the distance.
    //     return sqrt((pow((point1[0]-point2[0]), 2)+pow((point1[2]-point2[2]), 2)));
    // }
};

std::vector<float> laserScanData;

std::vector<float> slicing(std::vector<float>& arr, int X, int Y)
{
 
    // Starting and Ending iterators
    auto start = arr.begin() + X;
    auto end = arr.begin() + Y + 1;
 
    // To store the sliced vector
    std::vector<float> result(Y - X + 1);
 
    // Copy vector using copy function()
    copy(start, end, result.begin());
 
    // Return the final sliced vector
    return result;
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &data)
{

    laserScanData = data->ranges;

    // ROS_INFO("Callback called\n");

    // for (auto element : laserScanData)
    // {
    //     std::cout << element << ' ';    
    // }

    std::vector<float> left;
    std::vector<float> right;
    std::vector<float> front;

    left = slicing(laserScanData, 0, 159);
    front = slicing(laserScanData,160, 319);
    right = slicing(laserScanData, 320, 479);

    bool left_obs_flag = 0 ;
    bool front_obs_flag = 0 ;
    bool right_obs_flag = 0 ; // Checking for obstacle in left

    int left_count  = 0;
    int thresh1 = 2;
    for(int i=0 ; i<160 ; i++)
        if ( left[i] != 0 and left[i] < thresh1)
            left_count += 1;

    //Checking for obstacle in left

    int front_count  = 0;

    for (int j=0 ; j<160 ; j++ )
        if ( front[j] != 0 and front[j] < thresh1) 
            front_count += 1;
        
    //Checking for obstacle in left

    int right_count  = 0;

    for ( int k=0; k<160 ; k++)
        if (right[k] != 0 and right[k] < thresh1)
            right_count += 1;
    
    int pixel_thresh{10};
        
    if (left_count > pixel_thresh)
        left_obs_flag = 1;
    if (front_count > pixel_thresh)   
        front_obs_flag = 1;
    if (right_count > pixel_thresh)
        right_obs_flag = 1;

    //Obstacle avoidance algorithm

    if (front_obs_flag == 1)
    {
        if (right_obs_flag == 1 and left_obs_flag == 0)
            { 
            std::cout<<"Turn Left"<<std::endl;
            flag = 2;
            }
        else if (right_obs_flag == 0) 
            {
            std::cout<<"Turn Right"<<std::endl; //default
            flag = 1;
            }
        else
        {
            std::cout<<"U Turn"<<std::endl;
            flag = 3;
        }
        }
    else
    {
        std::cout<<"Go Forward"<<std::endl;
        flag = 0; //sow
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    ros::Rate loop_rate(10000);

    std::cout << "Entered script\n";

    ros::Subscriber laserScanSub{};
    laserScanSub = n.subscribe("scan", 10000, laser_scan_callback);

    delay(2000);


    if(wiringPiSetup()<0)
    {
        std::cout<<"setup wiring pi failed"<<'\n';
        return 1;
    }

    motorControl fwRight{5, 4}; //dir, pwm
    motorControl fwLeft{3, 2};
    motorControl bwRight{29, 28};
    motorControl bwLeft{25, 24};

    Navigation navigation{&n, &fwRight, &fwLeft, &bwRight, &bwLeft};


    int count{0};

    std::cout << "Starting...\n";   


    while(ros::ok() && count < 500000)
    {
        // ROS_INFO("** ");
        navigation.navigate_to_point();

        count++;

        ros::spinOnce();
        loop_rate.sleep();

        // std::cout << count << " ";
    }

    navigation.stopMotors();
    std::cout << "Stopping motors\n";

    return 0;
}
