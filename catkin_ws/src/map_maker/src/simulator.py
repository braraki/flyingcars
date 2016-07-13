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

import thread
#arguments
step_dist = float(rospy.get_param('/simulator/step_dist'))
delay = float(rospy.get_param('/simulator/delay'))

def analyse(p, info_dict):
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
			spots.append((int(1000*x), int(1000*y), int(1000*z)))
	spots.append((int(1000*x2), int(1000*y2), int(1000*z2)))
	return(spots)

class system:
	def __init__(self, info_dict, adjacency_array, path, ID):
		self.info_dict = info_dict
		self.adjacency_array = adjacency_array
		self.path = path
		self.ID = ID
		self.spots = analyse(self.path, info_dict)
		self.index = 0

	def update(self):
		loc = self.spots[self.index]
		if self.index < len(self.spots)-1:
			self.index += 1
		return(loc)

	def new_path(self, path):
		if path != self.path:
			self.path = path
			self.spots = analyse(self.path, self.info_dict)
			self.index = 0



class full_system:
	def __init__(self, info_dict, adjacency_array):
		self.info_dict = info_dict
		self.adjacency_array = adjacency_array
		self.pub = rospy.Publisher('~SimPos_topic', SimPos, queue_size = 10)
		self.go = False
		self.system_list = []
		self.x_list = []
		self.y_list = []
		self.z_list = []
		self.cf_num = None

	def runner(self):
		rospy.Subscriber('~path_topic', HiPath, self.act)
		print('in runner')
		rospy.spin()

	def act(self, data):
		print('act')
		if not self.go:
			self.collect_info(data)
		sys = self.system_list[data.ID]
		sys.new_path(data.path)
		#self.sub_run()

	def sub_run(self):
		rate = rospy.Rate(1/float(delay))
		while self.go:
			for index in range(len(self.system_list)):
				sys = self.system_list[index]
				loc = sys.update()
				(self.x_list[index], self.y_list[index], self.z_list[index]) = loc
			if not rospy.is_shutdown():
				self.pub.publish(self.x_list, self.y_list, self.z_list)
				#print('published: ' + str((self.x_list[0], self.y_list[0], self.z_list[0])))
				rate.sleep()

	def collect_info(self, data):
		if self.cf_num == None:
			self.cf_num = data.num_IDs
			self.system_list = [None]*self.cf_num
			self.x_list = [None]*self.cf_num
			self.y_list = [None]*self.cf_num
			self.z_list = [None]*self.cf_num
		sys = system(self.info_dict, self.adjacency_array, data.path, data.ID)
		self.system_list[data.ID] = sys
		if None not in self.system_list:
			self.go = True
			#self.sub_run()
			thread.start_new_thread ( self.sub_run , ())
			print('thread initialized')




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
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])/1000.0
			y = (y_list[ID])/1000.0
			z = (z_list[ID])/1000.0
			#print(category_list[ID])
			info_dict[ID] = (x, y, z)
		#print(info_dict)
		fs = full_system(info_dict, A)
		fs.runner()
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	print('test')
	rospy.init_node("sim_node")	
	map_maker_client()
