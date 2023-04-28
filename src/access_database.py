#!/usr/bin/env python3

import rospy
from firebase import firebase
from std_msgs.msg import String

# Read the url of database from txt file 
with open(r"/home/pi/catkin_ws/src/delivery_robot_phase_1/src/database_url.txt","r") as urlFile:
    url = urlFile.read()

appHandle = firebase.FirebaseApplication(dsn=url, authentication=None)

def publish_delivery_info(senderLocationPub, receiverLocationPub):
	'''
	Gets the delivery location data from the database and publishes it.
	'''
	senderLocation = appHandle.get('currentDelivery/Sender/Location', None)
	receiverLocation = appHandle.get('currentDelivery/Receiver/Location', None)
	
	senderLocationPub.publish(senderLocation)
	receiverLocationPub.publish(receiverLocation)

def set_availability_status(status):
	'''
	Sets the availability status of the robot.
	status = "yes"/"no"
	'''
	appHandle.put(url='', name='availability', data=status.data)

def set_progress_status(status):
	'''
	Sets the status of the current delivery.
	status = "in progress"/"done"
	'''
	progressStatus = status.data
	appHandle.put(url='currentDelivery', name='status', data=progressStatus)

	if progressStatus == "done":
		appHandle.put(url='currentDelivery/Receiver', name='Location', data="nil")
		appHandle.put(url='currentDelivery/Sender', name='Location', data="nil")
		
def main():
	rospy.init_node('access_database', anonymous=True)

	# Publishers
	senderLocationPub = rospy.Publisher('senderLocation', String, queue_size=10)
	receiverLocationPub = rospy.Publisher('receiverLocation', String, queue_size=10)

	# Subscribers
	availabilityStatusSub = rospy.Subscriber("availability", String,set_availability_status)
	progressStatusSub = rospy.Subscriber("progress", String, set_progress_status)

	rate = rospy.Rate(10) # 10Hz
		
	while not rospy.is_shutdown():

		publish_delivery_info(senderLocationPub, receiverLocationPub)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')
