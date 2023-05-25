# ****************************************************************************************************************************************
# Title            : gps.py
# Description      : # Author           : Sowbhagya Lakshmi H T
# Last revised on  : 20/05/2023
# ****************************************************************************************************************************************

#!/usr/bin/env python3

import rospy
# from firebase import firebase
from std_msgs.msg import Float64
from selenium import webdriver
from selenium.webdriver.firefox.options import Options
import time

class WebDriver:

	location_data = {}

	def __init__(self):
		
		self.options = Options()
		self.options.add_argument("--headless")
		self.driver = webdriver.Firefox(options=self.options)
		self.driver.implicitly_wait(5)
		
		#self.driver  = webdriver.Firefox()

	def extract_coord(self):
		current_url = self.driver.current_url
		
		val = current_url.split("@")
		#print(val)

		coord = val[1].split(",")

		lat = coord[0]
		lng = coord[1]

		return lat, lng

	def scrape(self, url):
		try:
			self.driver.get(url)
		except Exception as e:
			self.driver.quit()
			
		time.sleep(4)

		lat, lng = self.extract_coord()

		self.driver.refresh()

		return lat, lng

def main():
	rospy.init_node('gps', anonymous=True)
	
	url = "https://maps.app.goo.gl/MShCmi9utfbrxLgg8"
	x = WebDriver()

	latitudePub = rospy.Publisher('latitude', Float64, queue_size=10)
	longitudePub = rospy.Publisher('longitude', Float64, queue_size=10)

	rate = rospy.Rate(10) # 10Hz

	while not rospy.is_shutdown():

		latitude, longitude = x.scrape(url)

		latitudePub.publish(float(latitude))
		longitudePub.publish(float(longitude))

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')

