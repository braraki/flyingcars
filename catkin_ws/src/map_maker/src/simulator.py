#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from map_maker.srv import *
from map_maker.msg import *

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

import math
import matplotlib.pyplot as plt
import time
import random

import csv
import networkx as nx
from enum import Enum
import numpy as np

info_dict = {}
step_dist = .1
delay = .1

def analyse(p):
	spots = []
	for index in range(len(p)-1):
		ID1 = p[index]
		ID2 = p[index + 1]
		(x1, y1, z1) = info_dict[ID1]
		(x2, y2, z2) = info_dict[ID2]
		dist = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**.5
		num_divisions = int(dist/step_dist) + 1
		for d in range(num_divisions):
			x = (x1) + (d/float(num_divisions))*(x2 - x1)
			y = (y1) + (d/float(num_divisions))*(y2 - y1)
			z = (z1) + (d/float(num_divisions))*(z2 - z1)
			spots.append((x, y, z))
	spots.append((x2, y2, z2))
	return(spots)

def analysis_pub(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.path)
	p = data.path
	locs = analyse(p)
	#publish location one at a time with a frequency of 1/delay
	pub = rospy.Publisher('SimPos_topic', SimPos, queue_size = 10)
	rate = rospy.Rate(1/float(delay))
	for loc in locs:
		(x, y, z) = loc
		x = int(1000*x)
		y = int(1000*y)
		z = int(1000*z)
		if not rospy.is_shutdown():
			#print((x,y,z))
			pub.publish(x, y, z)
			rate.sleep()



def listener():
	rospy.init_node("sim_node")	
	rospy.Subscriber('path_topic', HiPath, analysis_pub)
	rospy.spin()

def map_maker_client():
	rospy.wait_for_service('send_map')
	try:
		print('calling')
		global info_dict
		func = rospy.ServiceProxy('send_map', MapTalk)
		resp = func()
		print('recieved')
		x_list = resp.x_list
		y_list = resp.y_list
		z_list = resp.z_list
		num_IDs = resp.num_IDs
		adjacency_array = resp.adjacency_array
		A = np.array(adjacency_array)
		A.shape = (num_IDs, num_IDs)
		for ID in range(num_IDs):
			x = (x_list[ID])/1000.0
			y = (y_list[ID])/1000.0
			z = (z_list[ID])/1000.0
			#print(category_list[ID])
			info_dict[ID] = (x, y, z)
		#print(info_dict)
		listener()
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	print('test')
	map_maker_client()
