# AUTONOMOUS DELIVERY ROBOT

## Summary of the project 
This project is aimed at building an autonomous robot to be used for delivering things for a closed environment like an office, a college campus, etc consisting of multiple buildings. The robot is inspired by food and package delivery robots developed by FedEx, Amazon, Starship, etc. prominently used in the US. 

The robot delivers documents from one location to another autonomously with the ease of a web application thus, reducing time and human effort. Some of the features include: web application, simultaneous deliveries, and charging stations. The robot is being built for the MIT campus, Anna University. Only the staffs belonging to this campus will have access to the robot through the web application and delivery orders can be placed after registering. The bot autonomously navigates from its current location to the sender’s location, collects the documents and goes to the receiver’s location. A security system is employed for safety reasons made of electromagnet and relay. Only users will be able to open the cabinets where the documents are stored. Autonomous navigation is implemented by a combination of local obstacle avoidance and global path planning.

The local obstacle avoidance algorithm using the depth information obtained from the Intel Realsense R200 camera. Using the laserscan package we get a single streak of data (480 data points) present at a particular height. Depending on this data, we can find the distance at which different obstacles are present. As for global path planning, we get the sender and receiver location throgh the web application from the user. We get the current location of the bot from GPS module. With the help of the Mapbox API, we get an optimized path from one location to another. The bot moves with the help of this optimized path and it's current location.

The bot can be extended for use in delivering items in an open environment. It can be trained for an external environment like food and package delivery applications and also indoor environments like medical applications for delivering medicines within the vicinity of a hospital.

## System Requirements
### Operating system
Tested on Ubuntu Server 20.04 

## Hardware requirements
1. [Raspberry Pi 4 model B](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf)
2. [MDD10A - Dual Channel 10A DC Motor Driver](https://robu.in/wp-content/uploads/2015/01/MDD10A-Users-Manual-Google-Docs.pdf)
3. [Intel RealSense R200 Camera](https://www.mouser.com/pdfdocs/intel_realsense_camera_r200.pdf)
4. [NEO-M8N GPS module](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf)
5. [DC DC XL6009E1 Step-up boost converter module](https://www.haoyuelectronics.com/Attachment/XL6009/XL6009-DC-DC-Converter-Datasheet.pdf)
6. [Johnson Geared Motor](https://robu.in/wp-content/uploads/2017/05/Johnson-Geared-Motor-Made-In-India-12-V-DC-300-RPM-ROBU.IN_-1.pdf)
7. [5 V Relay](https://html.alldatasheet.com/html-pdf/157071/DBLECTRO/JQC-3FC/384/1/JQC-3FC.html)
8. [MPU 9250](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
9. DC 24 V 300 mA 10 mm 6 N push-pull solenoid electromagnet
10. PTron power bank
11. Wheels
12. Lithium ion batteries
13. SD Card
14. USB c cable

## Software requirements
1. [Robot Operating System (ROS) - Noetic](http://wiki.ros.org/noetic/Installation)
2. [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
3. [Git](https://git-scm.com/downloads)
4. [Wiring Pi](http://wiringpi.com/download-and-install)


## Setting up the Raspberry Pi
The Raspberry Pi was setup and accessed in headless mode.

### Prepare SD card
* This was done using in a Windows OS.
* For SD cards with more than 32GB size, the file system is exFAT and not FAT32. But RPi does not recognize exFAT, hence change it to FAT32 to work. Used a [3rd party software](https://www.diskpart.com/download-home.html) to do this for our 64GB SD card.
* Used [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to flash Ubuntu Server 20.04 LTS to the SD card.
* Before flashing, do ctrl+shift+x to open image customization options. There we can set the hostname, enable SSH and configure wifi. Helps to avoid using monitors and do a headless setup.
* After flashing, insert SD card into the slot in Raspberry Pi and connect the power supply (PTron power bank via USB c cable) to the Raspberry Pi.
* Successful setup should ensure    
    - Constant red LED - Indicates sufficient power.
    - Blinking green LED - Indicates that the SD card is being accessed.

### Access Raspberry Pi remotely
Make sure the Raspberry Pi and the PC are connected to the same network.
* Open the terminal
* To install necessary tools for SSH
```
sudo apt install nmap
sudo apt-get install openssh-client
sudo apt-get install openssh-server
```
* To get the local IP address
```
hostname -I
```
* Replace the last number of the local IP address with 0/24 to the subnet range (Eg: if the local IP address is 192.168.43.68 then...)
```
nmap -sn 192.168.43.0/24
```
From this, we will get the IP address of Raspberry Pi.
* To establish an SSH connection using the IP address of Raspberry Pi, username and password we will be requires to access the Pi remotely
```
ssh <username>@<ip_address>
```
This would then prompt the user to enter the password, after which remote access will be established.

### Setting up Catkin Workspace
Refer [this ](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) tutorial.

### Setting up the main repository

Open the terminal and execute the following
```
cd ~/catkin_ws/src/
git clone https://github.com/MIT-Mentors/autonomous-delivery-robot
cd ~/catkin_ws
catkin_make
```

### Setting up the camera
Refer [this.](https://github.com/MIT-Mentors/Intel-RealSense-Camera-R200-setup)

### Setting up Firebase database
Run the following commands
```
sudo apt install python3-pip
sudo pip3 install requests
pip install git+https://github.com/ozgur/python-firebase
```

### Setting up wiring pi
Refer [this](http://wiringpi.com/download-and-install) for installation.
After installation, we need to add a few lines to ~/.bashrc file
```
echo "
sudo usermod -a -G gpio user_name
% change the owner and group respectively
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem" >> ~/.bashrc
``` 
Then execute
```
source ~/. bashrc
```

## Running the software
### For Obstacle avoidance
In a terminal run
```
roscore
```
In another terminal run
```
rosrun autonomous-delivery-robot obs_main
```
#### For delivery from dummy location A to dummy location B
```
roslaunch autonomous-delivery-robot delivery.launch
```

## Running the app
Refer [this]()

## Software module overview
| Module | Purpose |
|--------|---------|
| src/access_database.py |  To access the database to read/write data |
| src/database_url.txt | Contains the url of the database |
| src/delivery.cpp | Resolves the sender and receiver data and publishes the setpoint |
| src/main.cpp | Navigates the robot to the setpoint |
| src/obs_main.cpp | Does obstacle avoidance whilst basic locomotion |

## Software workflow
Firebase is used as a database where all the user data, availability of the robot, and data pertaining to a specific order are stored. The users are the only ones who will be able to access the robot. All the users should be registered with the application and should belong to the closed community for safety purposes. For example, if the robot is trained for a college campus, then only the staff can access the robot. All these staffs should be registered with the application. The data used as the username can be an email id specific to the college alone. The data is stored as key-value pairs in a NoSQL format. 

The data in Firebase can be stored as either a Real-time database or a Firestore database. We want a real-time updation as time plays a critical role in determining the order when multiple orders are placed, therefore, we have our data stored in the Real-time database. The database and web application are connected after configuration is done in the application. The data in the database can be read and written through the application. If a new user registers with the application, their data gets added to the database. If an existing user tries to log in, their corresponding username and password are checked and after verification, they’ll be logged in. Once logged in, the user can place orders if the robot is available. 

The information regarding the availability of the robot is also stored in the database. As soon as the order is placed the availability field is changed from ‘Yes’ to ‘No’. So, placing the order when the availability is ‘No’ is not possible. The following information is given by the receiver when placing an order: Sender’s location, Receiver’s location, and Receiver’s name in the application. The receiver’s name must be one of the registered users. Once the order has been placed, all the data specific to the order will be updated in the database under the ‘Current delivery’ field. The data stored in the database is accessed by the ROS nodes running in the Raspberry Pi through Rest APIs provided by the Firebase to access the information.

Based on the current location of the bot, sender's location, and receiver's location, an optimized path is found using the Mapbox API. The bot navigates with the help of the current location and the waypoints we get from the APPI. Once, the location is reached, the user can access the document holder through the app. Once the delivery is done, the availability is changed from ‘No’ to ‘Yes’, and the data under ‘Current delivery’ is copied to ‘Previous deliveries’ field and data is cleared from the former. So, the Raspberry Pi and application is integrated through the database.

## Known issues
1. Local obstacle avoidance algorithm: The accuracy and precision of the camera data under various conditions plays a major role. Refer [this.](https://messy-scallop-252.notion.site/CAMERA-ACCURACY-AND-PRECISION-0b4867ea712e4581ab8c16f60915d73a) The range of the depth information from Inter Realsense R200 is 0.5 m to 6 m. This pose as a problem to the accuracy of the local obstacle avoidance algorithm.
2. Authentication and deployment: The application has not used the in-built feature of Firebase of authentication. Hence, dynamic updation of the data for each user individually cannot be done. The application can then be hosted with Firebase itself. 

## Future scope
Simultaneous deliveries can be implemented. This helps in reducing the resources and waiting time in average. The deliveries can be done taking into consideration the locations and the time when the order was placed. Multiple charging stations can be installed to charge the batteries wirelessly thus, reducing the human interference. An algorithm can be built such that once the battery level goes below a particular value the bot autonomously navigates to the nearest charging station to charge itself. Optimization of local obstacle avoidance algorithm can be done with the help of the accuracy and precision measurements at various lighting conditions and the RGB data from the camera. 

## Project members
[Aarthi Meena](https://github.com/Aarthi160802)

[Sowbhagya Lakshmi](https://github.com/Sowbhagya-lakshmi)

[Yogeshwari](https://github.com/yogeshwari-vs)

**Mentored by :** [Pragash Durai](https://github.com/Dcruise546)


***

## LICENSE

MIT License