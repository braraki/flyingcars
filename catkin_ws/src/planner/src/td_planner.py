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

cf_num = int(rospy.get_param('/td_planner/cf_num'))
z_coefficient = float(rospy.get_param('/td_planner/z_coefficient'))
land_vel = 0.4 #m/s
air_vel = 0.8
dt = 0.1
res_table = []
numberOfNodes = 0
numTimesteps = 1200

class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}
##search

##this will search through the dictionary returned from a landscape in the 
##get_true_connection_dict function

class SearchNode:
	def __init__(self, state, parent, time, start,finish,cost=0):
		self.state = state
		self.parent = parent
		self.cost = cost
		self.start = start
		self.finish = finish
		self.time = time

	def path(self):
		if self.parent == None:
			return [self.state]
		else:
			return self.parent.path() + [self.state]

	def verbose_path(self):
		if self.parent == None:
			return [(self.state, self.start, self.finish)]
		else:
			return self.parent.verbose_path() + [(self.state,self.start,self.finish)]
	
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
 
def a_star(successors, goal_test, numNodes, start_node, heuristic=lambda x: 0):
	start_state = start_node.state

	if goal_test(start_state):
		return [start_state]

	#start_node = SearchNode(start_state, None, start_time, start_index, start_index, 0)
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	expanded = set()
	while not agenda.is_empty():
		parent = agenda.pop()
		if parent.state not in expanded or parent.state == parent.parent.state:
			expanded.add(parent.state)
			if goal_test(parent.state):
				return parent
			for child_state, cost, travel_time,start,finish in successors(parent.state,parent.time):
				child = SearchNode(child_state, parent, parent.time+travel_time,start,finish, parent.cost+cost)
				if child_state in expanded and child_state != parent.state:
					continue
				agenda.push(child, child.cost+heuristic(child_state))
	return None

def a_star_bad(successors, start_state, goal_test, numNodes, heuristic=lambda x: 0):
	if goal_test(start_state):
		return [start_state]

	start_node = SearchNode(start_state, None, 0, 0, 0, 0)
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	visited = [0]*numNodes
	while not agenda.is_empty():
		parent = agenda.pop()
		visited[parent.state] = visited[parent.state] + 1
		#print parent.state, visited[parent.state], parent.cost
		if goal_test(parent.state):
			return parent
		for child_state, cost, travel_time,start,finish in successors(parent.state,parent.time):
			child = SearchNode(child_state, parent, parent.time+travel_time,start,finish, (1+parent.cost+cost)*(visited[child_state]+1))
			agenda.push(child, child.cost+heuristic(child_state))
			print child.state, visited[child.state], child.cost
	return None

def successors(info_dict,adj_array):
	def __inner(state, t):

		(x1, y1, z1) = info_dict[state][0]
		sucs = []
		#the nth row in an adj_array corresponds to which nodes
		#the nth node connects to 
		row = adj_array[state]
		for (ID2, value) in enumerate(row):
			if value == 1:
				(fx, fy, fz) = info_dict[ID2][0]
				category = info_dict[ID2][1]
				if category == 1 or category == 2:
					vel = land_vel
				else:
					vel = air_vel
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + z_coefficient*(z1-fz)**2)**.5
				travel_time = dist_traveled/vel
				cost = travel_time + dist_traveled
				#assign a cost for waiting
				if dist_traveled == 0:
					travel_time = 0.11
					cost = travel_time * 10
				#convert the travel time into reservation table indices
				(start, finish) = timeToIndices(t, t+travel_time)
				reservations = res_table[ID2]
				res_list = reservations[start:finish]
				if start == finish:
					res_list = [reservations[start]]

				if 1 not in res_list:
					sucs.append((ID2, cost, travel_time,start,finish))
				#print ID2, res_list
		return(sucs)
	return __inner

def fillResTable(node):
	global res_table

	state = node.state
	start = node.start
	finish = node.finish

	for i in range(start,finish+1):
		if i >= 0:
			res_table[state][i] = 1
			if node.parent != None and node.parent.parent != None:
				res_table[node.parent.state][i] = 1

	if node.parent != None:
		fillResTable(node.parent)

def timeToIndices(start, finish):
	global res_table
	global numTimesteps

	startIndex = int(math.ceil(start/dt))
	finishIndex = int(math.ceil(finish/dt))

	if startIndex >= numTimesteps/2 or finishIndex >= numTimesteps/2:
		for i in range(numberOfNodes):
			res_table[i].extend([0]*1200)
		numTimesteps = numTimesteps + 1200
		print "ADDED MORE TIME STEPS:"
		print len(res_table[0])

	#print start, finish
	#print startIndex+1, finishIndex
	return startIndex+1,finishIndex

def a_star_old(successors, start_state, goal_test, numNodes, heuristic=lambda x: 0):
	if goal_test(start_state):
		return [start_state]

	start_node = SearchNode(start_state, None, 0, 0, 0, 0)
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	expanded = set()
	while not agenda.is_empty():
		parent = agenda.pop()
		if parent.state not in expanded:
			expanded.add(parent.state)
			if goal_test(parent.state):
				return parent
			for child_state, cost, travel_time,start,finish in successors(parent.state,parent.time):
				child = SearchNode(child_state, parent, parent.time+travel_time,start,finish, parent.cost+cost)
				if child_state in expanded:
					continue
				agenda.push(child, child.cost+heuristic(child_state))
	return None

class system:
	def __init__(self, adj_array, info_dict, cf_ID, pub):
		self.adj_array = adj_array
		self.info_dict = info_dict
		self.cf_ID = cf_ID
		self.pub = pub
		self.end_pos = None
		self.cf_pos = None
		self.park_IDs = []
		self.endNodes = {}
		for id in self.info_dict:
			info = self.info_dict[id]
			c = info[1]
			if c == Category.park:
				self.park_IDs.append(id)
		self.p = None

	def generate_random_path(self):
		if self.end_pos == None:
			ID1 = random.choice(self.park_IDs)
		else:
			for id in self.info_dict:
				if self.info_dict[id][0] == self.end_pos:
					ID1 = id
					break
		unfound = True
		while unfound:
			ID2 = random.choice(self.park_IDs)
			if ID2 != ID1:
				unfound = False
		self.end_pos = self.info_dict[ID2][0]
		p = self.find_path(ID1, ID2)
		return(p)

	def find_path(self, ID1, ID2):
		(x2, y2, z2) = self.info_dict[ID2][0]

		def goal_test(id):
			return  ID2== id

		def successors_old(id):
			(x1, y1, z1) = self.info_dict[id][0]
			sucs = []
			row = self.adj_array[id]
			for (ID2, value) in enumerate(row):
				if value == 1:
					(fx, fy, fz) = self.info_dict[ID2][0]
					dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + z_coefficient*(z1-fz)**2)**.5
					sucs.append((ID2, dist_traveled))
			return(sucs)
		def dist(id):
			(x1, y1, z1) = self.info_dict[id][0]
			dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
			return(dist)

		if ID1 in self.endNodes:
			last_node = self.endNodes[ID1]
			start_node = SearchNode(ID1, None, last_node.time, last_node.finish+1, last_node.finish+1, 0)
		else:
			start_node =SearchNode(ID1, None, 0, 0, 0, 0)

		end_node = a_star(successors(self.info_dict,self.adj_array), goal_test, len(info_dict),start_node, dist)
		fillResTable(end_node)
		self.endNodes[end_node.state] = end_node

		print end_node.verbose_path()
		return end_node.path()

	def update_cf_pos(self, pos):
		#print('position updated: '+str(pos))
		self.cf_pos = pos
		if self.is_finished():
			self.publish_new_path()


	def is_finished(self):
		if self.cf_pos == self.end_pos:
			return(True)
		if self.end_pos != None and self.cf_pos != None:
			(x1, y1, z1) = self.cf_pos
			(x2, y2, z2) = self.end_pos
			dist = ((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
			#print(dist)
			return(dist < .02)
		return(False)

	def publish_new_path(self):
		self.p = self.generate_random_path()
		if self.p != None and self.p != []:
			self.pub.publish(cf_num, self.cf_ID, self.p)
			print('published')
			print self.p

	def publish_old_path(self):
		if self.p != None and self.p != []:
			self.pub.publish(cf_num, self.cf_ID, self.p)

class full_system:
	def __init__(self, adj_array, info_dict):
		self.adj_array = adj_array
		self.info_dict = info_dict
		self.system_list = []
		self.pub = rospy.Publisher('~path_topic', HiPath, queue_size = 10)
		self.runner()

	def runner(self):
		for ID in range(cf_num):
			sys = system(self.adj_array, self.info_dict, ID, self.pub)
			self.system_list.append(sys)
		rospy.Subscriber('~SimPos_topic', SimPos, self.pos_update)
		for sys in self.system_list:
			sys.publish_new_path()
		thread.start_new_thread ( self.double_check , ())
		rospy.spin()

	def double_check(self):
		while True:
			for sys in self.system_list:
				sys.publish_old_path()
			time.sleep(.1)

	#response to simulator, updates position accordingly
	def pos_update(self, data):
		x_list = data.x
		y_list = data.y
		z_list = data.z
		for id in range(len(x_list)):
			sys = self.system_list[id]
			x = x_list[id]/1000.0
			y = y_list[id]/1000.0
			z = z_list[id]/1000.0
			sys.update_cf_pos((x, y, z))

def td_planner():
	global res_table
	global numberOfNodes
	rospy.wait_for_service('send_map')
	try:
		print('calling')
		global info_dict
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
		info_dict = {}
		numberOfNodes = num_IDs
		for ID in range(num_IDs):
			x = (x_list[ID])/1000.0
			y = (y_list[ID])/1000.0
			z = (z_list[ID])/1000.0
			c = static_category_dict[category_list[ID]]
			res_table.append([0]*numTimesteps)
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z),c)
		#print(info_dict)
		fs = full_system(A, info_dict)
		fs.runner()
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	rospy.init_node("td_planner",anonymous=False)
	td_planner()
