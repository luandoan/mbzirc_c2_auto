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
import rosparam


class ptz_control():
    def __init__(self):
	rospy.init_node('ptz_control', anonymous=True)
	
	# Initialize params
	self.detect_counter = 0         # Detection counter
	self.rest_time = 0.1
	self.n_pan = 0
	# Subscriber for topic
	rospy.Subscriber('/floats', numpy_msg(Floats), self.callback, queue_size=1)

	# Zoom all the way out
	#self.zoomOut()
	#rospy.sleep(1.0)
	#self.zoomIn()

    def callback(self, bearing):
	# Zoom all the way out
	#self.zoomOut()
	#rospy.sleep(1.0)

	if (bearing.data[1] != 0):
  	    print bearing.data[0]
	    print bearing.data[1]
	    
	    #zl = rospy.get_param('current_zlevel')

	    # define zoom_level based on distance
	    if (np.floor(bearing.data[1]/10) >= 5):
	        zoom_level = 5
	    elif (np.floor(bearing.data[1]/10) == 4):
		zoom_level = 4
	    elif (np.floor(bearing.data[1]/10) == 3):
		zoom_level = 3
	    elif (np.floor(bearing.data[1]/10) == 2):
		zoom_level = 2
	    elif (np.floor(bearing.data[1]/10) == 1):
		zoom_level = 1
	    else:
		zoom_level = 0
	    print "Zooming level: ", zoom_level

	    #zl = rospy.get_param('current_zlevel')
	    

		
	    # Zoom in to the certain level
	    self.zoom(zoom_level)	    

	    rospy.set_param('current_zlevel', zoom_level)
	    # define pan_angle
	    if (bearing.data[0] > 0):
		self.goRight()
		self.n_pan = self.n_pan + 1
		print "Pan to right"
	    elif(bearing.data[0] < 0):
		self.goLeft()
		print "Pan to left"
	    print "Number of pan: ", self.n_pan
		
	    #zw = 5 - zoom_level
	    #while (z < zw):
		#self.zoomWide()
		#print "Zooming out 1 step"
		#z = z + 1
		

		
 

	   # else:
		#self.detect_counter = self.detect_counter + 1

    def zoom(self,level):
	self.zoomOut()
	z = 0
	while(z < level):
	    self.zoomTile()
	    z = z + 1

    def zoom_level(self,level):
	if (zl == level):
	    self.zoomStop()
	elif (zl - level == 1):
	    self.zoomWide()
	else:
	    z = 0
	    while(z < level):
	    	self.zoomTile()
	    	z = z + 1


    def goRight(self):
    	ip_right = 'http://10.10.10.13/?command=ptz_req&req=start&param=directionleft&channel=1&stream=0'
    	URL_right = ip_right
    	ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=directionleft&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_right)
    	#rospy.Duration(10.0)
    	#response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)
	
    def goLeft(self):
    	ip_left = 'http://10.10.10.13/?command=ptz_req&req=start&param=directionright&channel=1&stream=0'
    	URL_left = ip_left
    	ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=directionright&channel=1&stream=0'
   	URL_stop = ip_stop    
    	response = ul.urlopen(URL_left)
    	#rospy.Duration(0.0)
    	#response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def lookUp():
    	ip_up = 'http://10.10.10.13/?command=ptz_req&req=start&param=directiondown&channel=1&stream=0'
    	URL_up = ip_up
	ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=directiondown&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_up)
	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def lookDown():
    	ip_down = 'http://10.10.10.13/?command=ptz_req&req=start&param=directionup&channel=1&stream=0'
    	URL_down = ip_down
	ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=directionup&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_down)
	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def zoomTile(self):
    	ip_zoomIn = 'http://10.10.10.13/?command=ptz_req&req=start&param=zoomtile&channel=1&stream=0'
    	URL_zoomIn = ip_zoomIn
    	ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=zoomtile&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_zoomIn)
    	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def zoomIn(self):
    	ip_zoomIn = 'http://10.10.10.13/?command=ptz_req&req=start&param=zoomtile&channel=1&stream=0'
    	URL_zoomIn = ip_zoomIn
	response = ul.urlopen(URL_zoomIn)

    def zoomWide(self):
    	ip_zoomOut = 'http://10.10.10.13/?command=ptz_req&req=start&param=zoomwide&channel=1&stream=0'
    	URL_zoomOut = ip_zoomOut
    	ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=zoomwide&channel=1&stream=0'
    	URL_stop = ip_stop
    	response = ul.urlopen(URL_zoomOut)
    	rospy.Duration(0.0)
    	response = ul.urlopen(URL_stop)
    	#rospy.loginfo(fullURL)

    def zoomOut(self):
    	ip_zoomOut = 'http://10.10.10.13/?command=ptz_req&req=start&param=zoomwide&channel=1&stream=0'
    	URL_zoomOut = ip_zoomOut
	response = ul.urlopen(URL_zoomOut)

    def stopPTZ():
    	ip = 'http://10.10.10.13/?command=ptz_req&req=stop&param=directiondown&channel=1&stream=0'
    	URL = ip
    	response = ul.urlopen(URL)



if __name__== '__main__':
    try:
	ptz_control()
    	rospy.spin()
    except KeyboardInterrupt:
	rospy.loginfo("Shutting down PTZ node")



