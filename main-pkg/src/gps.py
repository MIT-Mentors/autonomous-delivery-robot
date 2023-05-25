#!/usr/bin/env python3

import rospy
from firebase import firebase
from std_msgs.msg import String
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

		print("lat: ", coord[0])
		print("long: ", coord[1])

	def scrape(self, url):
		try:
			self.driver.get(url)
		except Exception as e:
			self.driver.quit()
			
		time.sleep(4)

		lat, lng = self.extract_coord()

		self.driver.refresh()


def main():
	rospy.init_node('gps', anonymous=True)
	
	url = "https://maps.app.goo.gl/MShCmi9utfbrxLgg8"
	x = WebDriver()

	count = 0
	while(1):
		print(count)
		x.scrape(url)
		count+=1 	



if __name__ == '__main__':
	main()
