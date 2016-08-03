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

from map_maker import gen_adj_array_info_dict


#parameter

ideal_way_point_d = float(rospy.get_param('/complex_map/ideal_way_point_d'))

land_vel = float(rospy.get_param('/complex_map/land_vel'))
air_vel = float(rospy.get_param('/complex_map/air_vel'))
time_step = float(rospy.get_param('/complex_map/time_step'))
optimal = bool(rospy.get_param('/complex_map/optimal'))

if optimal:
	air_way_point_d = air_vel*(time_step)
	land_way_point_d = land_vel*(time_step)
else:
	air_way_point_d = ideal_way_point_d
	land_way_point_d = ideal_way_point_d

'''
class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}
'''

#a_list represents the distance at which the number of waypoint nodes should change.
#The values represent the max distance for the waypoint of that entries index
'''
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
'''
'''
class tile:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.flyable = False
		self.length = 0
		self.width = 0

	def assign_length(self, l):
		self.length = l

	def assign_width(self, w):
		self.width = w

	def assign_flyable(self, f):
		self.flyable = f

	def is_contained(self, new_x, new_y):
		if self.x <= new_x <= self.x+self.length:
			if self.y <= new_y <= self.y+self.width:
				#print("TRUEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
				return(True)
		return(False)

#mark nodes must come before cloud nodes

def remake_cloud(info_dict, A):
	new_info_dict = {}
	e_list = []
	min_x = 0
	max_x = 0
	min_y = 0
	max_y = 0
	max_ID = 0

	cloud_heights = []

	tile_list = []
	tile_x = []
	tile_y = []
	cloud_interface_list = []

	for (ID1, row) in enumerate(A):
		((x1, y1, z1), c1) = info_dict[ID1]
		if x1 < min_x:
			min_x = x1
		elif x1 > max_x:
			max_x = x1
		if y1 < min_y:
			min_y = y1
		elif y1 > max_y:
			max_y = y1
		if c1 != Category.cloud and c1 != Category.interface:
			new_info_dict[ID1] = info_dict[ID1]
			if ID1 > max_ID:
				max_ID = ID1
			for (ID2, value) in enumerate(row):
				if value == 1:
					((x2, y2, z2), c2) = info_dict[ID2]
					if c2 != Category.cloud and c2 != Category.interface:
						e_list.append((ID1, ID2))
			if c1 == Category.mark:
				if x1 not in tile_x:
					tile_x.append(x1)
				if y1 not in tile_y:
					tile_y.append(y1)
				t = tile(x1, y1)
				tile_list.append(t)
		else:
			cloud_interface_list.append(((x1, y1, z1), c1))
			if c1 == Category.cloud and z1 not in cloud_heights:
				cloud_heights.append(z1)
	#determining tile size
	tile_x = sorted(tile_x)
	tile_y = sorted(tile_y)
	if len(tile_x) == 1:
		tile_length = 1000
	else:
		tile_length = tile_x[1] - tile_x[0]
	if len(tile_y) == 1:
		tile_width == 1000
	else:
		tile_width = tile_y[1] - tile_y[0]
	#determining flyable
	for t in tile_list:
		t.assign_length(tile_length)
		t.assign_width(tile_width)
		for ((x1, y1, z1), c1) in cloud_interface_list:
			if t.is_contained(x1, y1):
				t.assign_flyable(True)
	#making the clouds
	if len(cloud_heights) > 0:
		(cloud_info_dict, cloud_e_list) = generate_cloud(cloud_heights, tile_list, min_x, max_x, min_y, max_y, max_ID)
		for ID1 in new_info_dict:
			((x1, y1, z1), c1) = new_info_dict[ID1]
			if c1 != Category.mark:
				for t in tile_list:
					if t.is_contained(x1, y1):
						if t.flyable:
							min_dist = None
							for ID2 in cloud_info_dict:
								(x2, y2, z2) = cloud_info_dict[ID2][0]
								dist = ((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)**.5
								if min_dist == None:
									min_dist = dist
									chosen = ID2
								elif min_dist > dist:
									min_dist = dist
									chosen = ID2
							e_list.append((ID1, chosen))
							e_list.append((chosen, ID1))
	else:
		cloud_info_dict = {}
		cloud_e_list = []
	final_e_list = e_list + cloud_e_list
	new_info_dict.update( cloud_info_dict)
	#print('new info dict')
	#print(new_info_dict)
	#print(" ")
	#print('final e list')
	#print(final_e_list)
	G = nx.DiGraph()
	for ID in new_info_dict:
		G.add_node(ID)
	for e in final_e_list:
		G.add_edge(e[0], e[1])
	A = nx.to_numpy_matrix(G)
	A2 = A.flatten()
	A3 = A2.tolist()
	A4 = A3[0]
	A5 = []
	for fl in A4:
		A5.append(int(fl))
	A6 = np.array(A5)
	num_IDs = len(new_info_dict)
	A6.shape = (num_IDs, num_IDs)

	return((new_info_dict, A6))

def generate_cloud(cloud_heights, tile_list, min_x, max_x, min_y, max_y, max_ID):

	current_ID = max_ID
	cloud_info_dict = {}
	multi_connect = []

	cloud_layer_dist = 0
	cloud_heights = sorted(cloud_heights)
	cloud_height = cloud_heights[0]
	num_cloud_layers = len(cloud_heights)
	#print(cloud_heights)
	#print('num cloud layers: '+str(num_cloud_layers))
	if num_cloud_layers > 1:
		cloud_layer_dist = cloud_heights[1] - cloud_heights[0]

	#getting air_cloud_dimensions
	grid_dist = (2.0/float(1 + (2.0)**.5))*air_way_point_d
	num_min_x = int(min_x / grid_dist)
	num_max_x = int(max_x / grid_dist)
	num_min_y = int(min_y / grid_dist)
	num_max_y = int(max_y / grid_dist)
	#print((num_min_x, num_max_x, num_min_y, num_max_y))

	current_layer = 0

	while current_layer < num_cloud_layers:
		z = cloud_height + current_layer * cloud_layer_dist
		x_multiple = num_min_x
		while x_multiple <= num_max_x:
			x = x_multiple*grid_dist
			y_multiple = num_min_y
			while y_multiple <= num_max_y:
				#print('current_layer: '+str(current_layer))
				#print('x: '+str(x_multiple))
				#print('y: '+str(y_multiple))
				#print(" ")
				y = y_multiple*grid_dist
				for t in tile_list:
					if t.is_contained(x, y):
						if t.flyable:
							current_ID += 1
							cloud_info_dict[current_ID] = ((x, y, z), Category.cloud)
							if (x_multiple + y_multiple)%2 == 0:
								multi_connect.append(current_ID)
						break
				y_multiple += 1
			x_multiple += 1
		current_layer += 1
	#print('cloud info dict')
	#print(cloud_info_dict)
	#print(" ")
	cloud_e_list = connect_cloud(cloud_info_dict, multi_connect, grid_dist, cloud_layer_dist)
	return(cloud_info_dict, cloud_e_list)


def connect_cloud(cloud_info_dict, multi_connect, grid_dist, cloud_layer_dist):
	cloud_e_list = []
	for ID1 in cloud_info_dict:
		(x1, y1, z1) = cloud_info_dict[ID1][0]
		if ID1 in multi_connect:
			min_dist = ((2.0)**.5 * grid_dist)*1.1
		else:
			min_dist = grid_dist * 1.1		
		for ID2 in cloud_info_dict:
			(x2, y2, z2) = cloud_info_dict[ID2][0]
			#same layer
			if z2 == z1:
				distance = ((x1 - x2)**2 + (y1 - y2)**2)**.5
				if distance < min_dist:
					cloud_e_list.append((ID1, ID2))
			#different layer
			elif abs(z2 - z1) <= cloud_layer_dist*1.1:
				if x1 == x2 and y1 == y2:
					cloud_e_list.append((ID1, ID2))
	return(cloud_e_list)



'''


def get_num_waypoints2(ID1, ID2, info_dict):
	(x1, y1, z1) = info_dict[ID1][0]
	c1 = info_dict[ID1][1]
	(x2, y2, z2) = info_dict[ID2][0]
	c2 = info_dict[ID2][1]
	dist = ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)**.5
	if c1 != Category.cloud and c2 == Category.cloud and c1 != Category.interface and c2 == Category.interface:
		waypoint_d = land_way_point_d
	else:
		waypoint_d = air_way_point_d
	raw_num = dist/float(waypoint_d)
	low = math.floor(raw_num)
	hi = math.ceil(raw_num)
	low_d = dist/float(low+1)
	hi_d = dist/float(hi+1)
	if abs(hi_d - waypoint_d) < abs(waypoint_d - low_d):
		return(int(hi))
	else:
		return(int(low))

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
		(x1, y1, z1) = info1[0]
		if c1 != Category.land and c1 != Category.park:
			pass_1 = True
		for (ID2, value) in enumerate(row):
			if value == 1:
				pass_2 = False
				info2 = info_dict[ID2]
				c2 = info2[1]
				(x2, y2, z2) = info2[0]
				if c2 != Category.land and c2 != Category.park:
					pass_2 = True
				if not optimal and (pass_1 or pass_2):
					e_list.append((ID1, ID2))
				elif pass_1 and pass_2 and z1 == z2:
					e_list.append((ID1, ID2))
				else:
					#print('in')
					(x1, y1, z1) = info1[0]
					(x2, y2, z2) = info2[0]
					dist = ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)**.5
					#nw = get_num_waypoints(dist)
					nw = get_num_waypoints2(ID1, ID2, info_dict)
					last_ID = ID1
					if pass_1 or pass_2:
						wp_c = Category.air_waypoint
					else:
						wp_c = Category.waypoint
					for wp_num in range(nw):
						wp_x = x1 + ((wp_num + 1)/float(nw + 1))*(x2 - x1)
						wp_y = y1 + ((wp_num + 1)/float(nw + 1))*(y2 - y1)
						wp_z = z1 + ((wp_num + 1)/float(nw + 1))*(z2 - z1)
						wp_ID = ID_num
						ID_num += 1
						new_info_dict[wp_ID] = ((wp_x, wp_y, wp_z), wp_c)
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
		s = rospy.Service('send_complex_map', MapTalk ,self.response)
		print('ready to send info back')
		rospy.spin()
'''
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
'''

if __name__ == "__main__":
	print('test')
	rospy.init_node('complex_map_maker_server')
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_map')
	Category = gen_adj_array_info_dict.Category
	#print(info_dict)
	#if optimal:
		#(info_dict, A) = remake_cloud(info_dict, A)
	analysis = get_new_info(info_dict, A)
	s = sender(analysis[0], analysis[1])
	#s = sender(info_dict, A)
	s.info_sender()