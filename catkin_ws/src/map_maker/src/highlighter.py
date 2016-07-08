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





class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud}
##search

##this will search through the dictionary returned from a landscape in the 
##get_true_connection_dict function

class SearchNode:
	def __init__(self, state, parent, cost=0):
		self.state = state
		self.parent = parent
		self.cost = cost

	def path(self):
		if self.parent == None:
			return [self.state]
		else:
			return self.parent.path() + [self.state]
	
class PriorityQueue:
	def __init__(self):
		self.data = []
	def push(self, item, cost):
		self.data.append((cost, item))
	def pop(self):
		self.data.sort()
		return self.data.pop(0)[1]
	def is_empty(self):
		return len(self.data) == 0
 
def a_star(successors, start_state, goal_test, heuristic=lambda x: 0):
	if goal_test(start_state):
		return [start_state]
	start_node = SearchNode(start_state, None, 0)
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	expanded = set()
	while not agenda.is_empty():
		parent = agenda.pop()
		if parent.state not in expanded:
			expanded.add(parent.state)
			if goal_test(parent.state):
				return parent.path()
			for child_state, cost in successors(parent.state):
				child = SearchNode(child_state, parent, parent.cost+cost)
				if child_state in expanded:
					continue
				agenda.push(child, child.cost+heuristic(child_state))
	return None

z_coefficient = 3
def find_path(ID1, ID2, adjacency_array, info_dict):
	(x2, y2, z2) = info_dict[ID2][0]
	def successors(id):
		(x1, y1, z1) = info_dict[id][0]
		sucs = []
		row = adjacency_array[id]
		for (ID2, value) in enumerate(row):
			if value == 1:
				(fx, fy, fz) = info_dict[ID2][0]
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + z_coefficient*(z1-fz)**2)**.5
				sucs.append((ID2, dist_traveled))
		return(sucs)
	def goal_test(id):
		return  ID2== id
	def dist(id):
		(x1, y1, z1) = info_dict[id][0]
		dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
		return(dist)
	return(a_star(successors, ID1, goal_test, dist))

end_pos = None
pub = None
adj_array = None
info_dict = {}

def generate_random_path(adjacency_array, info_dict):
	global end_pos
	park_IDs = []
	for id in info_dict:
		info = info_dict[id]
		c = info[1]
		if c == Category.park:
			park_IDs.append(id)
	if end_pos == None:
		ID1 = random.choice(park_IDs)
	else:
		for id in info_dict:
			if info_dict[id][0] == end_pos:
				ID1 = id
				break
	unfound = True
	while unfound:
		ID2 = random.choice(park_IDs)
		if ID2 != ID1:
			unfound = False
	p = find_path(ID1, ID2, adjacency_array, info_dict)
	return(p)



def get_postion(data):
	global end_pos
	global pub
	global adj_array
	global info_dict
	x = data.x/1000.0
	y = data.y/1000.0
	z = data.z/1000.0
	crazyflie_position = (x, y, z)
	if is_finished(crazyflie_position, end_pos):
		p = generate_random_path(adj_array, info_dict)
		if p != None:
			print(p)
			pub.publish(p)
			end_ID = p[len(p)-1]
			end_pos = info_dict[end_ID][0]


def publish_paths(delay, adjacency_array, info_dict):
	global pub
	num = 0
	pub = rospy.Publisher('path_topic', HiPath, queue_size = 10)
	rospy.init_node('highlighter', anonymous=True)
	rate = rospy.Rate(1/float(delay)) # 10hz
	while not rospy.is_shutdown():
		print(num)
		p = generate_random_path(adjacency_array, info_dict)
		if p != None:
			print(p)
			pub.publish(p)
		rate.sleep()
		num += 1

def is_finished(cf_pos, end_pos):
	if end_pos == None or cf_pos == None:
		return(True)
	(x1, y1, z1) = cf_pos
	(x2, y2, z2) = end_pos
	dist = ((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
	#print(dist)
	return(dist < .02)

def publish_path():
	global end_pos
	global adj_array
	global info_dict
	global pub
	pub = rospy.Publisher('path_topic', HiPath, queue_size = 10)
	rospy.init_node('highlighter', anonymous=True)
	rospy.Subscriber('SimPos_topic', SimPos, get_postion)
	end_pos = None
	if not rospy.is_shutdown():
		p = generate_random_path(adj_array, info_dict)
		if p != None:
			print(p)
			pub.publish(p)
			end_ID = p[len(p)-1]
			end_pos = info_dict[end_ID][0]
	rospy.spin()

delay = 10

info_dict = {}

def map_maker_client():
	global adj_array
	global info_dict
	rospy.wait_for_service('send_map')
	try:
		print('calling')
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
		adj_array = A
		for ID in range(num_IDs):
			x = (x_list[ID])/1000.0
			y = (y_list[ID])/1000.0
			z = (z_list[ID])/1000.0
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
		#publish_paths(delay, A, info_dict)
		publish_path()
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	print('test')
	map_maker_client()
