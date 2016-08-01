#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from map_maker.srv import *
from map_maker.msg import *
from planner.srv import *

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

import thread

#arguments

used_park_IDs = []

cf_num = int(rospy.get_param('/si_planner/cf_num'))
z_coefficient = float(rospy.get_param('/si_planner/z_coefficient'))
land_vel = .25#0.025 #m/s
air_vel = .5#0.05

si_dict = {}

count = 0

class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}

def convert_graph(old_graph, old_adj, time_horizon):
	og = old_graph
	oa = old_adj
	th = time_horizon

	# for i in range(num_IDs):
	# 	for j in range(time_horizon+1):
	# 		new_graph[(i,j)] = info_dict[ID]

	# new adj array
	# the format is this: each node ID is represented by
	# ID, ID+num_IDs, ID+2*num_IDs, .... ID+time_horizon*num_IDs
	# aka ID+(timestep)*num_IDs
	# note that you can retrieve the original ID of one of the
	# new states: old_ID = new_ID % num_IDs
	na = np.zeros((num_IDs*(th+1),num_IDs*(th+1))


	for ID in range(num_IDs):
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
		for timestep in range(th):
			na[ID+timestep*num_IDs, ID+(timestep+1)*num_IDs] = 1

			# need to find successors of ID (sID) and connect
			# ID+timestep*num_IDs to sID+(timestep+1)*num_IDs
			old_row = oa[ID]
			for (ID2, value) in enumerate(old_row):
				if value == 1:
					# ID2 is a successor
					na[ID+timestep+num_IDs, ID2+(timestep+1)*num_IDs] = 1

def opt_planner():
	global si_dict
	rospy.wait_for_service('send_complex_map')
	try:
		print('calling')
		func = rospy.ServiceProxy('send_complex_map', MapTalk)
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
		adj_array = A
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	print('test')
	rospy.init_node('opt_planner', anonymous = True)
	opt_planner()
