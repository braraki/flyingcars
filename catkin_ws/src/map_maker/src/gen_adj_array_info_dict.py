#!/usr/bin/env python

import rospy
from map_maker.srv import *

import time

import networkx as nx
from enum import Enum
import numpy as np



class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}



#map_topic should be 'send_map' or 'send_complex_map'
def map_maker_client(map_topic='send_complex_map'):
	rospy.wait_for_service(map_topic)
	try:
		print('calling')
		func = rospy.ServiceProxy(map_topic, MapTalk)
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
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
		return((info_dict, A))
	except rospy.ServiceException, e:
		print("service call failed")

'''
if __name__ == "__main__":
	map_maker_client()
'''