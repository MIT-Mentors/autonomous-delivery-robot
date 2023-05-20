# AUTONOMOUS DELIVERY ROBOT

## Summary of the project 

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


## Setting up the computer

### Setting up Raspberry pi
The Raspberry Pi was setup and accessed in headless mode.

#### Prepare SD card
* This was done using in a Windows OS.
* For SD cards with more than 32GB size, the file system is exFAT and not FAT32. But RPi does not recognize exFAT, hence change it to FAT32 to work. Used a [3rd party software](https://www.diskpart.com/download-home.html) to do this for our 64GB SD card.
* Used [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to flash Ubuntu Server 20.04 LTS to the SD card.
* Before flashing, do ctrl+shift+x to open image customization options. There we can set the hostname, enable SSH and configure wifi. Helps to avoid using monitors and do a headless setup.
* After flashing, insert SD card into the slot in Raspberry Pi and connect the power supply (PTron power bank via USB c cable) to the Raspberry Pi.
* Successful setup should ensure    
    - Constant red LED - Indicates sufficient power.
    - Blinking green LED - Indicates that the SD card is being accessed.

#### Access Raspberry Pi remotely
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
roslaunch delivery.launch
```

## Running the app
Refer [this]()

## Software module overview

## Software workflow

## Known issues

## Future scope

## Further study

## Project members
[Aarthi Meena](https://github.com/Aarthi160802)

[Sowbhagya Lakshmi](https://github.com/Sowbhagya-lakshmi)

[Yogeshwari](https://github.com/yogeshwari-vs)

**Mentored by :** [Pragash Durai](https://github.com/Dcruise546)


***

## LICENSE

MIT License