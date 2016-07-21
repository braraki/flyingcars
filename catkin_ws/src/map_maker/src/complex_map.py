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

import networkx as nx
from enum import Enum
import numpy as np


#parameter

ideal_way_point_d = float(rospy.get_param('/complex_map/ideal_way_point_d'))

class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}

#a_list represents the distance at which the number of waypoint nodes should change.
#The values represent the max distance for the waypoint of that entries index
a_list = []

#expands the a_list
def update_a_list(dist):
	print('update')
	global a_list
	n = len(a_list) - 1
	unfound = True
	while unfound:
		n += 1
		denom = float(1/float(n+1)+1/float(n+2))
		a_n = (2*ideal_way_point_d)/denom
		a_list.append(a_n)
		if a_n > dist:
			unfound = False

#finds the proper number of waypoints based on the distance
def get_num_waypoints(dist):
	#print('get num')
	for i in range(len(a_list)):
		if a_list[i] >= dist:
			return(i)
	update_a_list(dist)
	return(get_num_waypoints(dist))

#returns info_dict and adjacency_array with waypoints added
def get_new_info(info_dict, adjacency_array):
	ID_num = len(info_dict)

	new_info_dict = info_dict.copy()
	G = nx.DiGraph()

	e_list = []
	for (ID1, row) in enumerate(adjacency_array):
		pass_1 = False
		G.add_node(ID1)
		info1 = info_dict[ID1]
		c1 = info1[1]
		if c1 != Category.land and c1 != Category.park:
			pass_1 = True
		for (ID2, value) in enumerate(row):
			if value == 1:
				pass_2 = False
				info2 = info_dict[ID2]
				c2 = info2[1]
				if c2 != Category.land and c2 != Category.park:
					pass_2 = True
				if pass_1 or pass_2:
					e_list.append((ID1, ID2))
				else:
					#print('in')
					(x1, y1, z1) = info1[0]
					(x2, y2, z2) = info2[0]
					dist = ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)**.5
					nw = get_num_waypoints(dist)
					last_ID = ID1
					for wp_num in range(nw):
						wp_x = x1 + ((wp_num + 1)/float(nw + 1))*(x2 - x1)
						wp_y = y1 + ((wp_num + 1)/float(nw + 1))*(y2 - y1)
						wp_z = z1 + ((wp_num + 1)/float(nw + 1))*(z2 - z1)
						wp_ID = ID_num
						ID_num += 1
						new_info_dict[wp_ID] = ((wp_x, wp_y, wp_z), Category.waypoint)
						e_list.append((last_ID, wp_ID))
						last_ID = wp_ID
						#print('made way point')
					e_list.append((last_ID, ID2))

	for e in e_list:
		G.add_edge(e[0], e[1])
	A = nx.to_numpy_matrix(G)

	return(new_info_dict, A)

#sends info out
class sender:
	def __init__(self, info_dict, adjacency_matrix):
		A2 = adjacency_matrix.flatten()
		A3 = A2.tolist()
		A4 = A3[0]
		self.A5 = []
		for fl in A4:
			self.A5.append(int(fl))
		#print(A5)

		coordinate_list = [None]*len(info_dict)
		self.x_list = [None]*len(info_dict)
		self.y_list = [None]*len(info_dict)
		self.z_list = [None]*len(info_dict)
		self.category_list = [None]*len(info_dict)

		for ID in info_dict:
			coor = info_dict[ID][0]
			self.x_list[ID] = coor[0]
			self.y_list[ID] = coor[1]
			self.z_list[ID] = coor[2]
			cat = info_dict[ID][1]
			self.category_list[ID] = int(cat)

		self.num_nodes = len(info_dict)

	def response(self, req):
		return MapTalkResponse(self.category_list, self.x_list, self.y_list, self.z_list, self.num_nodes, self.A5)

	def info_sender(self):
		rospy.init_node('complex_map_maker_server')
		s = rospy.Service('send_complex_map', MapTalk ,self.response)
		print('ready to send info back')
		rospy.spin()

def map_maker_client():
	rospy.wait_for_service('send_map')
	try:
		print('calling')
		info_dict = {}
		func = rospy.ServiceProxy('send_map', MapTalk)
		resp = func()
		print('recieved')
		category_list = resp.category_list
		x_list = resp.x_list
		y_list = resp.y_list
		z_list = resp.z_list
		num_IDs = resp.num_IDs
		adjacency_array = resp.adjacency_array
		A = np.array(adjacency_array)
		A.shape = (num_IDs, num_IDs)
		for ID in range(num_IDs):
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
		analysis = get_new_info(info_dict, A)
		s = sender(analysis[0], analysis[1])
		s.info_sender()
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	print('test')
	map_maker_client()