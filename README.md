# AUTONOMOUS DELIVERY ROBOT

## Summary of the project 
This project is aimed at building an autonomous robot to be used for delivering things for a closed environment like an office, a college campus, etc consisting of multiple buildings. The robot is inspired by food and package delivery robots developed by FedEx, Amazon, Starship, etc. prominently used in the US. 

The robot delivers documents from one location to another autonomously with the ease of a web application thus, reducing time and human effort. Some of the features include: web application, simultaneous deliveries, and charging stations. The robot is being built for the MIT campus, Anna University. Only the staffs belonging to this campus will have access to the robot through the web application and delivery orders can be placed after registering. The bot autonomously navigates from its current location to the sender’s location, collects the documents and goes to the receiver’s location. A security system is employed for safety reasons made of electromagnet and relay. Only users will be able to open the cabinets where the documents are stored. Autonomous navigation is implemented by a combination of local obstacle avoidance and global path planning.

The local obstacle avoidance algorithm using the depth information obtained from the Intel Realsense R200 camera. Using the laserscan package we get a single streak of data (480 data points) present at a particular height. Depending on this data, we can find the distance at which different obstacles are present. As for global path planning, we get the sender and receiver location throgh the web application from the user. We get the current location of the bot from GPS module. With the help of the Mapbox API, we get an optimized path from one location to another. The bot moves with the help of this optimized path and it's current location.

The bot can be extended for use in delivering items in an open environment. It can be trained for an external environment like food and package delivery applications and also indoor environments like medical applications for delivering medicines within the vicinity of a hospital.


## Hardware requirements
1. [Raspberry Pi 4 model B](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf)
2. [MDD10A - Dual Channel 10A DC Motor Driver](https://robu.in/wp-content/uploads/2015/01/MDD10A-Users-Manual-Google-Docs.pdf)
3. [Intel RealSense R200 Camera](https://www.mouser.com/pdfdocs/intel_realsense_camera_r200.pdf)
4. [NEO-M8N GPS module](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf)
5. [DC DC XL6009E1 Step-up boost converter module](https://www.haoyuelectronics.com/Attachment/XL6009/XL6009-DC-DC-Converter-Datasheet.pdf)
6. [Johnson Geared Motor](https://robu.in/wp-content/uploads/2017/05/Johnson-Geared-Motor-Made-In-India-12-V-DC-300-RPM-ROBU.IN_-1.pdf)
7. [5 V Relay]()

## Software requirements

## Setting up the computer

## Running the software

## Running the app

## Software module overview

## Software workflow
Firebase is used as a database where all the user data, availability of the robot, and data pertaining to a specific order are stored. The users are the only ones who will be able to access the robot. All the users should be registered with the application and should belong to the closed community for safety purposes. For example, if the robot is trained for a college campus, then only the staff can access the robot. All these staffs should be registered with the application. The data used as the username can be an email id specific to the college alone. The data is stored as key-value pairs in a NoSQL format. 

The data in Firebase can be stored as either a Real-time database or a Firestore database. We want a real-time updation as time plays a critical role in determining the order when multiple orders are placed, therefore, we have our data stored in the Real-time database. The database and web application are connected after configuration is done in the application. The data in the database can be read and written through the application. If a new user registers with the application, their data gets added to the database. If an existing user tries to log in, their corresponding username and password are checked and after verification, they’ll be logged in. Once logged in, the user can place orders if the robot is available. 

The information regarding the availability of the robot is also stored in the database. As soon as the order is placed the availability field is changed from ‘Yes’ to ‘No’. So, placing the order when the availability is ‘No’ is not possible. The following information is given by the receiver when placing an order: Sender’s location, Receiver’s location, and Receiver’s name in the application. The receiver’s name must be one of the registered users. Once the order has been placed, all the data specific to the order will be updated in the database under the ‘Current delivery’ field. The data stored in the database is accessed by the ROS nodes running in the Raspberry Pi through Rest APIs provided by the Firebase to access the information.

Based on the current location of the bot, sender's location, and receiver's location, an optimized path is found using the Mapbox API. The bot navigates with the help of the current location and the waypoints we get along the APPI. Once, the location is reached, the user can access the document holder through the app. Once the delivery is done, the availability is changed from ‘No’ to ‘Yes’, and the data under ‘Current delivery’ is copied to ‘Previous deliveries’ field and data is cleared from the former. So, the Raspberry Pi and application is integrated through the database.

## Known issues
Markup : 1. Local obstacle avoidance algorithm: The accuracy of the  

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