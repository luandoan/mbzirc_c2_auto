#!/usr/bin/env python

import rospy
import urllib2
import urllib2 as ul
import time
import actionlib
from actionlib_msgs.msg import *
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import math

class ptz_control():
    def __init__(self):
	rospy.init_node('ptz_control', anonymous=True)
	
	# Initialize params
	self.detect_counter = 0         # Detection counter
	self.rest_time = 0.1

	# Subscriber for topic
	rospy.Subscriber('/detection', numpy_msg(Floats), self.callback, queue_size=1)

    def callback(self, bearing):
	if (bearing.data[0] != 0):
	    if self.detect_counter > 10:
		rospy.logdebug("Object identified")
		# define zoom_level based on distance
		if (np.floor(bearing.data[1]/30) >= 5):
		    zoom_init = 5
	   	elif (np.floor(bearing.data[1]/30) = 4):
		    zoom_init = 4
		elif (np.floor(bearing.data[1]/30) = 3):
		    zoom_init = 3
		elif (np.floor(bearing.data[1]/30) = 2):
		    zoom_init = 2
		elif (np.floor(bearing.data[1]/30) = 1):
		    zoom_init = 1
		else:
		    zoom_init = 0

		# define pan_angle
		if (bearing.data[0] > 0):
		    goRight()
		elif(bearing.data[0] < 0):
		    goLeft()

		# Zoom to get close to object
		z = 0
		while (z < zoom_init):
		    zoomIn()
		    z = z + 1

		
 

	    else:
		self.detect_counter = self.detect_counter + 1






    def goRight():
    	ip_right = 'http://192.168.0.99/?command=ptz_req&req=start&param=directionleft&channel=1&stream=0'
    	URL_right = ip_right
    	ip_stop = 'http://192.168.0.99/?command=ptz_req&req=stop&param=directionleft&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_right)
    	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)
	
    def goLeft():
    	ip_left = 'http://192.168.0.99/?command=ptz_req&req=start&param=directionright&channel=1&stream=0'
    	URL_left = ip_left
    	ip_stop = 'http://192.168.0.99/?command=ptz_req&req=stop&param=directionright&channel=1&stream=0'
   	URL_stop = ip_stop    
    	response = ul.urlopen(URL_left)
    	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def lookUp():
    	ip_up = 'http://192.168.0.99/?command=ptz_req&req=start&param=directiondown&channel=1&stream=0'
    	URL_up = ip_up
	ip_stop = 'http://192.168.0.99/?command=ptz_req&req=stop&param=directiondown&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_up)
	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def lookDown():
    	ip_down = 'http://192.168.0.99/?command=ptz_req&req=start&param=directionup&channel=1&stream=0'
    	URL_down = ip_down
	ip_stop = 'http://192.168.0.99/?command=ptz_req&req=stop&param=directionup&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_down)
	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def zoomTile():
    	ip_zoomIn = 'http://192.168.0.99/?command=ptz_req&req=start&param=zoomtile&channel=1&stream=0'
    	URL_zoomIn = ip_zoomIn
    	ip_stop = 'http://192.168.0.99/?command=ptz_req&req=stop&param=zoomtile&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_zoomIn)
    	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def zoomWide():
    	ip_zoomOut = 'http://192.168.0.99/?command=ptz_req&req=start&param=zoomwide&channel=1&stream=0'
    	URL_zoomOut = ip_zoomOut
    	ip_stop = 'http://192.168.0.99/?command=ptz_req&req=stop&param=zoomwide&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_zoomOut)
    	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def stopPTZ():
    	ip = 'http://192.168.0.99/?command=ptz_req&req=stop&param=directiondown&channel=1&stream=0'
    	URL = ip
    	response = ul.urlopen(URL)



if __name__== '__main__':
    try:
	ptz_control()
    	rospy.spin()
    except KeyboardInterrupt:
	rospy.loginfo("Shutting down PTZ node")



