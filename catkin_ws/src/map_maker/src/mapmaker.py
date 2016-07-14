#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from map_maker.srv import *
'''
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
'''
import math
import matplotlib.pyplot as plt
import time
import random
import networkx as nx
from enum import Enum
import numpy as np

#imported parameters
map_road_ratio = float(rospy.get_param('/mapmaker/map_road_ratio'))
map_tile_size = float(rospy.get_param('/mapmaker/map_tile_size'))

map_interface_height = float(rospy.get_param('/mapmaker/map_interface_height'))
map_cloud_height = float(rospy.get_param('/mapmaker/map_cloud_height'))
map_num_cloud_layers = int(rospy.get_param('/mapmaker/map_num_cloud_layers'))
map_cloud_layer_dist = float(rospy.get_param('/mapmaker/map_cloud_layer_dist'))
map_cloud_density = int(rospy.get_param('/mapmaker/map_cloud_density'))


#imported map parameters
map_num_long = int(rospy.get_param('/mapmaker/map_num_long'))
map_num_wide = int(rospy.get_param('/mapmaker/map_num_wide'))
non_fly_list = rospy.get_param('/mapmaker/non_fly_list')
map_pre_dict = rospy.get_param('/mapmaker/map_pre_dict')

#converts strings to tuples
for k in map_pre_dict.keys():
	map_pre_dict[eval(k)] = map_pre_dict[k]
	del map_pre_dict[k]
for index in range(len(non_fly_list)):
	non_fly_list[index] = eval(non_fly_list[index])

print(non_fly_list)
print(map_pre_dict)

#parameters
auto_road_ratio = .5
drone_ground_speed = 1
drone_air_speed = 1

#used to get edge ID, do not change
edge_num = 0



class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

class return_node:
	def __init__(self, x, y, z, ID, category):
		self.x = x
		self.y = y
		self.z = z
		self.ID = ID
		self.category = category

	def turn_to_graph(self, graph):
		graph.add_node(self.ID)

class edge:
	def __init__(self, node1_ID, node2_ID, node1, node2, ID):
		self.node1_ID = node1_ID
		self.node2_ID = node2_ID
		self.node1 = node1
		self.node2 = node2
		self.ID = ID

	def turn_to_graph(self, graph):
		graph.add_edge(self.node1_ID, self.node2_ID)

class node:
	def __init__(self, x, y, z = 0, angle = None, ID = None, category = None, successors = []):
		self.x = x
		self.y = y
		self.z = z
		self.angle = angle
		self.ID = ID
		self.category = category
		self.successors = successors[:]
		self.add_successor(self)

	def add_successor(self, suc):
		if suc not in self.successors:
			self.successors.append(suc)

	def add_category(self, c):
		self.category = c

	def generate_return_and_edge(self):
		r_node = return_node(self.x, self.y, self.z, self.ID, self.category)
		edge_list = []
		for s in self.successors:
			global edge_num
			e = edge(self.ID, s.ID, self, s, edge_num)
			edge_list.append(e)
			edge_num += 1
		return((r_node, edge_list))

class tile:

	def __init__(self, length, width, exitnodelist = [], flyable = True, road_ratio = auto_road_ratio, x = None, y = None, elevation = 0, node_list = []):
		self.length = length
		self.width = width
		if 'C' in exitnodelist and len(exitnodelist)>1:
			exitnodelist.remove('C')
		self.exitnodelist = []
		for i in exitnodelist:
			if i not in self.exitnodelist:
				self.exitnodelist.append(i)
		self.flyable = flyable
		self.road_ratio = road_ratio
		self.elevation = elevation
		self.x = x
		self.y = y
		self.node_list = node_list[:]

	#sets the tiles coordinates
	def set_co(self, x, y):
		self.x = x
		self.y = y

	#creates the node for a tile
	def create_nodes(self):
		delta_x = self.length*self.road_ratio*.25
		delta_y = self.width*self.road_ratio*.25
		#helipad
		if self.exitnodelist == ['C']:
			n = node(self.x, self.y, self.elevation, get_rud_angle(0,0))
			n.add_category(Category.park)
			self.node_list.append(n)
		#dead-end
		elif len(self.exitnodelist) == 1:
			dx = delta_x
			dy = delta_y
			x_coeff = -1
			y_coeff = -1
			if 'N' in self.exitnodelist:
				dy *= -1
				y_coeff = 1
			elif 'S' in self.exitnodelist:
				y_coeff = 1
			elif 'E' in self.exitnodelist:
				dx *= -1
				x_coeff = 1
			elif 'W' in self.exitnodelist:
				x_coeff = 1
			n1 = node(self.x+dx, self.y+dy, self.elevation, get_rud_angle(dx, dy))
			n2 = node(self.x+dx*x_coeff, self.y+dy*y_coeff, self.elevation, get_rud_angle(dx*x_coeff, dy*y_coeff))
			n1.add_category(Category.land)
			n2.add_category(Category.land)
			self.node_list.append(n1)
			self.node_list.append(n2)

		#non-split
		elif len(self.exitnodelist) == 2:
			dx = delta_x
			dy = delta_y
			#verticle
			if 'N' in self.exitnodelist and 'S' in self.exitnodelist:
				dy = 0
			#horizontal
			elif 'E' in self.exitnodelist and 'W' in self.exitnodelist:
				dx = 0
			#postive slope turn
			elif ('E' in self.exitnodelist and 'S' in self.exitnodelist) or ('W' in self.exitnodelist and 'N' in self.exitnodelist):
				dx *= 1
				dy *= -1
			n1 = node(self.x+dx, self.y+dy, self.elevation, get_rud_angle(dx, dy))
			n2 = node(self.x-dx, self.y-dy, self.elevation, get_rud_angle(-dx, -dy))
			n1.add_category(Category.land)
			n2.add_category(Category.land)
			self.node_list.append(n1)
			self.node_list.append(n2)
		#intersection
		elif len(self.exitnodelist) > 2:
			for dx in [-1*delta_x, 1*delta_x]:
				for dy in [-1*delta_y, 1*delta_y]:
					n = node(self.x+dx, self.y+dy, self.elevation, get_rud_angle(dx, dy))
					n.add_category(Category.land)
					self.node_list.append(n)
		
	#makes connections to nodes within the same tile, connects 90 degree off things anti-clockwise
	def connect_own(self):
		for node1 in self.node_list:
			for node2 in self.node_list:
				if node1.angle != None and node2.angle != None:
					if node1.category == Category.land and node2.category == Category.land:
						if (node2.angle - node1.angle)%360 == 90:
							node1.add_successor(node2)
						
						if len(self.exitnodelist) == 2:
							if 'N' in self.exitnodelist and 'S' in self.exitnodelist:
								node1.add_successor(node2)
								node2.add_successor(node1)
							elif 'E' in self.exitnodelist and 'W' in self.exitnodelist:
								node1.add_successor(node2)
								node2.add_successor(node1)
						

	#adds parking nodes to a tile
	def add_and_connect_parking(self):
		dx = 0
		dy = 0
		#normal road
		if len(self.exitnodelist) == 2:
			straight = False
			if 'N' in self.exitnodelist and 'S' in self.exitnodelist:
				dx = self.road_ratio*self.length*.5
				straight = True
			elif 'E' in self.exitnodelist and 'W' in self.exitnodelist:
				dy = self.road_ratio*self.length*.5
				straight = True
			elif 'N' in self.exitnodelist and 'E' in self.exitnodelist:
				dx = -1*self.road_ratio*self.length*.25
				dy =  -1*self.road_ratio*self.width*.25
			elif 'W' in self.exitnodelist and 'N' in self.exitnodelist:
				dx = 1*self.road_ratio*self.length*.25
				dy =  -1*self.road_ratio*self.width*.25
			elif 'S' in self.exitnodelist and 'W' in self.exitnodelist:
				dx = 1*self.road_ratio*self.length*.25
				dy =  1*self.road_ratio*self.width*.25
			elif 'E' in self.exitnodelist and 'S' in self.exitnodelist:
				dx = -1*self.road_ratio*self.length*.25
				dy =  1*self.road_ratio*self.width*.25
			if straight:
				n1 = node(self.x + dx, self.y + dy, self.elevation, get_rud_angle(dx, dy))
				n1.add_category(Category.park)
				n2 = node(self.x - dx, self.y - dy, self.elevation, get_rud_angle(-1*dx, -1*dy))
				n2.add_category(Category.park)
			else:
				n1 = node(self.x + 2*dx, self.y + dy, self.elevation, get_rud_angle(dx, dy))
				n1.add_category(Category.park)
				n2 = node(self.x + dx, self.y + 2*dy, self.elevation, get_rud_angle(dx, dy))
				n2.add_category(Category.park)
			for n3 in self.node_list:
				if n3.angle == n1.angle:
					n3.add_successor(n1)
					n1.add_successor(n3)
				if n3.angle == n2.angle:
					n3.add_successor(n2)
					n2.add_successor(n3)
			self.node_list.append(n1)
			self.node_list.append(n2)

		#3 way intersection and dead-end
		elif len(self.exitnodelist) == 3 or len(self.exitnodelist) == 1 and 'C' not in self.exitnodelist:
			if 'N' not in self.exitnodelist and 'S' in self.exitnodelist:
				dy = self.road_ratio*self.length*.5
			elif 'S' not in self.exitnodelist and 'N' in self.exitnodelist:
				dy = -1*self.road_ratio*self.length*.5
			elif 'E' not in self.exitnodelist and 'W' in self.exitnodelist:
				dx = self.road_ratio*self.width*.5
			elif 'W' not in self.exitnodelist and 'E' in self.exitnodelist:
				dx = -1*self.road_ratio*self.width*.5
			angle = get_rud_angle(dx, dy)
			n1 = node(self.x + dx, self.y + dy, self.elevation, angle)
			n1.add_category(Category.park)
			self.node_list.append(n1)
			for n in self.node_list:
				if (n.angle - angle)%360 == 45:
					n1.add_successor(n)
				elif (angle - n.angle)%360 == 45:
					n.add_successor(n1)

	#add mark nodes
	def add_mark_nodes(self):
		n = node(self.x - self.length*.5, self.y - self.width*.5, self.elevation)
		n.add_category(Category.mark)
		self.node_list.append(n)

	#finds a node that matches certain constraints, which side of the tile it should be on
	#and which axis of alignment is most important (necessary to do corners and deadends
	#as the correct node may not be in the predicted quadrant, rather further back on the dominant axis)
	def find_node(self, x_sign, y_sign, dom_axis):
		max_x = 0
		max_y = 0
		chosen_n = None
		for n in self.node_list:
			if n.category == Category.land:
				if (n.x - self.x)*x_sign >= max_x:
					if (n.y - self.y)*y_sign >= max_y:
						max_x = (n.x - self.x)*x_sign
						max_y = (n.y - self.y)*y_sign
						chosen_n = n
		#catching corners
		if chosen_n == None:
			if dom_axis == 'x':
				for n in self.node_list:
					if n.category == Category.land:
						if (n.x - self.x)*x_sign >= max_x:
							max_x = (n.x - self.x)*x_sign
							chosen_n = n
			if dom_axis == 'y':
				for n in self.node_list:
					if n.category == Category.land:
						if (n.y - self.y)*y_sign >= max_y:
							max_y = (n.y - self.y)*y_sign
							chosen_n = n
		return(chosen_n)

	#displays the board, not the nodes and edges; single lines for the road running to the center of cells
	def display(self):
		length = self.length
		width = self.width
		base_x = self.x - .5*length
		base_y = self.y - .5*width
		xs = [base_x, base_x + length, base_x+length, base_x, base_x]
		ys = [base_y, base_y, base_y+width, base_y+width, base_y]
		plt.plot(xs, ys, 'k')
		if not self.flyable:
			plt.fill(xs, ys, 'r')
		for letter in self.exitnodelist:
			if letter == 'C':
				x = base_x + .5*length
				y = base_y +.5*width
			if letter == 'N':
				x = base_x + .5*length
				y = base_y + width
			if letter == 'S':
				x = base_x + .5*length
				y = base_y
			if letter == 'E':
				x = base_x + length
				y = base_y + .5*width
			if letter == 'W':
				x = base_x
				y = base_y + .5*width
			plt.plot([x, base_x+.5*length],[y, base_y +.5*width], 'g')
			plt.plot([x, base_x+.5*length],[y, base_y +.5*width], 'go')

#think about remove interface and cloud arguments, never will be used
class landscape:
	#initiates a landscape
	def __init__(self, num_long, num_wide, tile_dict = {}, interface = None, cloud = None):
		self.num_long = num_long
		self.num_wide = num_wide
		self.tile_dict = tile_dict
		for x in range(num_long):
			for y in range(num_wide):
				if (x,y) not in tile_dict:
					self.tile_dict[(x, y)] = None
				else:
					t = tile_dict[(x, y)]
					t.set_co(x*t.length, y*t.width)
					t.create_nodes()
					t.connect_own()
					t.add_and_connect_parking()
					t.add_mark_nodes()
		self.interface = interface
		self.cloud = cloud

	#fills a tile
	def fill_tile(self, tile, coordinate):
		self.tile_dict[coordinate] = tile
		x = coordinate[0]*tile.length
		y = coordinate[1]*tile.width
		tile.set_co(x, y)
		tile.create_nodes()
		tile.connect_own()

	#displays every tile
	def display(self):
		for co in self.tile_dict.keys():
			t = self.tile_dict[co]
			if t != None:
				t.display()
		plt.show(block = False)
			
	#checks if road runs into neighboring tiles, assumes map is valid
	def ground_tile_successors(self, tile_coordinates):
		t = self.tile_dict[tile_coordinates]
		x = tile_coordinates[0]
		y = tile_coordinates[1]
		successors = []
		if 'N' in t.exitnodelist and y<self.num_wide-1:
			successors.append((x, y+1))
		if 'S' in t.exitnodelist and y>0:
			successors.append((x, y-1))
		if 'E' in t.exitnodelist and x<self.num_long-1:
			successors.append((x+1, y))
		if 'W' in t.exitnodelist and x>0:
			successors.append((x-1, y))
		return(successors)

	#does not assume valid map, verifies the ground tile successors and constructs a dict
	def get_true_connection_dict(self):
		successor_d = {}
		for co in self.tile_dict:
			suc = self.ground_tile_successors(co)
			successor_d[co] = suc
		checked_d = {}
		for co in successor_d:
			successors = successor_d[co]
			true_successors = []
			for s in successors:
				if co in successor_d[s]:
					true_successors.append(s)
			checked_d[co] = true_successors
		return(checked_d)

	#assumes two tiles connect validly, finds and adds the connection between their nodes
	def cross_tile_connect_node(self, tile1, tile2):
		if tile2.y > tile1.y and tile2.x == tile1.x:
			x1_sign = 1
			y1_sign = 1
			x2_sign = 1
			y2_sign = -1
			dom_axis = 'x'
		elif tile2.y < tile1.y and tile2.x == tile1.x:
			x1_sign = -1
			y1_sign = -1
			x2_sign = -1
			y2_sign = 1
			dom_axis = 'x'
		elif tile2.x > tile1.x and tile2.y == tile1.y:
			x1_sign = 1
			y1_sign = -1
			x2_sign = -1
			y2_sign = -1
			dom_axis = 'y'
		elif tile2.x < tile1.x and tile2.y == tile1.y:
			x1_sign = -1
			y1_sign = 1
			x2_sign = 1
			y2_sign = 1
			dom_axis = 'y'
		else:
			return
		n1 = tile1.find_node(x1_sign, y1_sign, dom_axis)
		n2 = tile2.find_node(x2_sign, y2_sign, dom_axis)
		n1.add_successor(n2)

	#connects the nodes throughout tiles on the landscape
	def fully_connect(self):
		tiles = self.tile_dict.values()
		checked_d = self.get_true_connection_dict()
		for co1 in checked_d:
			co_list = checked_d[co1]
			tile1 = self.tile_dict[co1]
			for co in co_list:
				t = self.tile_dict[co]
				self.cross_tile_connect_node(tile1, t)

	#assigns IDs to the nodes, done without any particular order/pattern (there may be a pattern, but its not intended)
	def assign_ID(self):
		num = 0
		for t in self.tile_dict.values():
			for n in t.node_list:
				n.ID = num
				num += 1
		if self.interface != None and self.cloud != None:
			for ns in self.interface.node_dict.values():
				for n in ns:
					n.ID = num
					num += 1
			for n in self.cloud.node_dict.values():
				n.ID = num
				num += 1

	#generates lists of the simple node and edge class that represent the map
	def get_nodes_and_edges(self):
		self.fully_connect()
		if self.interface != None and self.cloud != None:
			self.interface.generate_nodes()
			self.interface.connect_to_land()
			self.cloud.generate_nodes()
			self.cloud.connect_own_2()
			self.connect_interface_and_cloud()
		self.assign_ID()
		return_node_list = []
		edge_list = []
		for t in self.tile_dict.values():
			for n in t.node_list:
				info = n.generate_return_and_edge()
				return_node_list.append(info[0])
				edge_list += info[1]
		if self.interface != None and self.cloud != None:
			for ns in self.interface.node_dict.values():
				for n in ns:
					info = n.generate_return_and_edge()
					return_node_list.append(info[0])
					edge_list += info[1]
			for n in self.cloud.node_dict.values():
				info = n.generate_return_and_edge()
				return_node_list.append(info[0])
				edge_list += info[1]
		return((return_node_list, edge_list))

	#builds interface and cloud
	def generate_interface_and_cloud(self, interface_height, cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density):
		self.generate_interface(interface_height)
		self.generate_cloud(cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density)

	#builds interface
	def generate_interface(self, interface_height):
		i = interface(interface_height, self)
		self.interface = i

	#builds cloud
	def generate_cloud(self, cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density):
		c = cloud(cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density, self)
		self.cloud = c

	#connects interface to cloud
	def connect_interface_and_cloud(self):
		min_z = self.cloud.height
		for t in self.interface.node_dict:
			i_nodes = self.interface.node_dict[t]
			for i_node in i_nodes:
				c_node_list = self.cloud.tile_node_dict[t]
				for c_node in c_node_list:
					if c_node.z == min_z:
						i_node.add_successor(c_node)
						c_node.add_successor(i_node)


# a single layer to connect the ground to the cloud
class interface:
	def __init__(self, height, landscape):
		self.height = height
		self.landscape = landscape
		self.node_dict = {}

	#generates the interfaces nodes
	def generate_nodes(self):
		for t in self.landscape.tile_dict.values():
			if t.flyable:
				if len(t.exitnodelist) > 0:
					temp_list =[]
					for n in t.node_list:
						if n.category == Category.park or n.category == Category.land:
							x = n.x
							y = n.y
							n2 = node(x, y, self.height)
							n2.add_category(Category.interface)
							temp_list.append(n2)
					self.node_dict[t] = temp_list


	#connects the interface to the ground
	def connect_to_land(self):
		for t in self.landscape.tile_dict.values():
			if t in self.node_dict:
				ns = self.node_dict[t]
				for n1 in ns:
					for n2 in t.node_list:
						if n2.category != Category.mark:
							if n1.x == n2.x and n1.y == n2.y:
								n1.add_successor(n2)
								n2.add_successor(n1)

#hovering multilayer, multinode per tile node-cloud(density represents how many nodes across a cell)
class cloud:
	def __init__(self, height, num_layers, layer_dist, density, landscape):
		self.height = height
		self.num_layers = num_layers
		self.layer_dist = layer_dist
		self.density = density
		self.node_dict = {}
		self.tile_node_dict = {}
		self.landscape = landscape

	#builds nodes
	def generate_nodes(self):
		for t in self.landscape.tile_dict.values():
			if t.flyable:
				base_y = t.y - .5*t.width + 1/float(2*self.density)*t.width
				y_dist = t.width/float(self.density)
				base_x = t.x - .5*t.length + 1/float(2*self.density)*t.length
				x_dist = t.length/float(self.density)
				for z in range(self.num_layers):
					z = z*self.layer_dist + self.height
					for dy in range(self.density):
						y = base_y + dy*y_dist
						for dx in range(self.density):
							x = base_x + dx*x_dist
							n = node(x, y, z)
							n.add_category(Category.cloud)
							if t not in self.tile_node_dict:
								self.tile_node_dict[t] = [n]
							else:
								self.tile_node_dict[t] += [n]
							self.node_dict[(x,y,z)] = n

	#links adjacent (including diagonal) nodes in a two-way edge
	def connect_own(self):
		t = self.landscape.tile_dict.values()[0]
		y_dist = t.width/float(self.density)
		x_dist = t.length/float(self.density)
		z_dist = self.layer_dist
		for n in self.node_dict.values():
			for dx in [-1*x_dist, 0 , x_dist]:
				for dy in [-1*y_dist, 0, y_dist]:
					for dz in [-1*z_dist, 0, z_dist]:
						if dx != 0 or dy != 0 or dz != 0:
							if (dx+n.x, dy+n.y, dz+n.z) in self.node_dict:
								n2 = self.node_dict[(dx+n.x, dy+n.y, dz+n.z)]
								n.add_successor(n2)

	#same purpose as connect own, bug free but less efficient
	def connect_own_2(self):
		for t in self.tile_node_dict:
			nodes = self.tile_node_dict[t]
			for n1 in nodes:
				for n2 in nodes:
					if n1 != n2:
						x_dist = abs(n1.x - n2.x)
						y_dist = abs(n1.y - n2.y)
						z_dist = abs(n1.z - n2.z)
						if x_dist < (t.length/float(self.density))*1.1:
							if y_dist < (t.width/float(self.density))*1.1:
								if z_dist < self.layer_dist*1.1:
									n1.add_successor(n2)
									n2.add_successor(n1)
			for t2 in self.tile_node_dict:
				if t2 != t:
					if (t2.x - t.x)<t.length*1.1:
						if (t2.y - t.y)<t.width*1.1:
							for n1 in nodes:
								for n2 in self.tile_node_dict[t2]:
									x_dist = abs(n1.x - n2.x)
									y_dist = abs(n1.y - n2.y)
									z_dist = abs(n1.z - n2.z)
									if x_dist < (t.length/float(self.density))*1.1:
										if y_dist < (t.width/float(self.density))*1.1:
											if z_dist < self.layer_dist*1.1:
												n1.add_successor(n2)
												n2.add_successor(n1)
												x_dist = abs(n1.x - n2.x)
												y_dist = abs(n1.y - n2.y)
												z_dist = abs(n1.z - n2.z)
												if x_dist < (t.length/float(self.density))*1.1:
													if y_dist < (t.width/float(self.density))*1.1:
														if z_dist < self.layer_dist*1.1:
															n1.add_successor(n2)
															n2.add_successor(n1)



'''
#step by step building of a map (probably not going to be used to much)
#not up to date
def builder():
	tile_length = input('input tile length: ')
	tile_width = input('input tile width: ')
	num_long = input('input num long: ')
	num_wide = input('input num wide: ')
	l = landscape(num_long, num_wide)
	for x in range(num_long):
		for y in range(num_wide):
			proper = False
			while not proper:
				print('current tile is '+str((x, y)))
				directions = raw_input('input directions: ')
				exitnodelist = []
				if 'c' in directions or 'C' in directions:
					exitnodelist.append('C')
				if 'n' in directions or 'N' in directions:
					exitnodelist.append('N')
				if 's' in directions or 'S' in directions:
					exitnodelist.append('S')
				if 'e' in directions or 'E' in directions:
					exitnodelist.append('E')
				if 'w' in directions or 'W' in directions:
					exitnodelist.append('W')
				flyable = raw_input('input flyable: ')
				if flyable == 'True' or flyable == 'False':
					flyable = bool(flyable)
					proper = True
				else:
					print('Something went wrong, try again on the same tile')
			print(" ")
			t = tile(tile_length, tile_width, exitnodelist, flyable)
			l.fill_tile(t, (x, y))
			l.display()
	info = l.get_nodes_and_edges()
	return_nodes = info[0]
	edges = info[1]
	node_plot(return_nodes, edges)
	return(info)
'''
#estimates an angle, used in connecting the nodes together
#one would have no reason to call this function, but it is necessary for the code to function
def get_rud_angle(dx, dy):
	angle_dict = {(1,0):0,(1,1):45,(0,1):90, (-1,1):135, (-1,0):180, (-1,-1):225, (0, -1):270, (1,-1):315, (0,0):None}
	if dx != 0:
		dx = int(abs(dx)/dx)
	if dy != 0:
		dy = int(abs(dy)/dy)
	return(angle_dict[(dx, dy)])

#using return nodes and edges, displays the information
def node_plot(node_list, edge_list):
	ID_dict = {}
	x_dot_list = []
	y_dot_list = []
	for n in node_list:
		ID_dict[n.ID] = (n.x, n.y, n.z)
		if n.z == 0:
			x_dot_list.append(n.x)
			y_dot_list.append(n.y)
	plt.plot(x_dot_list, y_dot_list, 'bo')
	for e in edge_list:
		ax = plt.axes()
		n1 = ID_dict[e.node1_ID]
		n2 = ID_dict[e.node2_ID]
		if n1[2] == 0 and n2[2] == 0:
			n1x = n1[0]
			n1y = n1[1]
			n2x = n2[0]
			n2y = n2[1]
			ax.arrow(n1x, n1y, n2x - n1x, n2y - n1y, head_width = .15, head_length = .35)
	plt.show()

#generates a random valid map, does not necessarily have the map fully connected (islands can exist)
def generate_random_world(num_long, num_wide, length, width, density):
	l = landscape(num_long, num_wide)
	current_density = 0
	letters = ['N','S','E','W','C']
	tile_dict = {}
	coop_list = []
	for x in range(num_long):
		for y in range(num_wide):
			tile_dict[(x,y)] = []
			coop_list.append((x,y))
	iterations = 0
	while current_density < density:
		co_1 = random.choice(coop_list)
		if len(tile_dict[co_1])>2:
			co_1 = random.choice(coop_list)
		letter = random.choice(letters)
		current_list = tile_dict[co_1]
		if letter == 'N' and co_1[1]<(num_wide-1) and 'N' not in current_list:
			current_list.append('N')
			tile_dict[co_1] = current_list
			next_list = tile_dict[(co_1[0], co_1[1]+1)]
			next_list.append('S')
			tile_dict[(co_1[0], co_1[1]+1)] = next_list
		elif letter == 'S' and co_1[1]>0 and 'S' not in current_list:
			current_list.append('S')
			tile_dict[co_1] = current_list
			next_list = tile_dict[(co_1[0], co_1[1]-1)]
			next_list.append('N')
			tile_dict[(co_1[0], co_1[1]-1)] = next_list
		elif letter == 'E' and co_1[0]<(num_long-1) and 'E' not in current_list:
			current_list.append('E')
			tile_dict[co_1] = current_list
			next_list = tile_dict[(co_1[0]+1, co_1[1])]
			next_list.append('W')
			tile_dict[(co_1[0]+1, co_1[1])] = next_list
		elif letter == 'W' and co_1[0]>0 and 'W' not in current_list:
			current_list.append('W')
			tile_dict[co_1] = current_list
			next_list = tile_dict[(co_1[0]-1, co_1[1])]
			next_list.append('E')
			tile_dict[(co_1[0]-1, co_1[1])] = next_list
		elif letter == 'C' and current_list == []:
			current_list.append('C')
			tile_dict[co_1] = current_list
		paths = 0
		for co in coop_list:
			paths += len(tile_dict[co])
		current_density = paths/float(num_long*num_wide*4)
		iterations += 1
		#print(iterations)
		#print(current_density)
	for tile in tile_dict:
		v = tile_dict[tile]
		if len(v)>1 and 'C' in v:
			v.remove('C')
		tile_dict[tile] = v
	return(tile_dict)

map_num_long = 12
map_num_wide = 12
map_pre_dict = generate_random_world(map_num_long, map_num_wide, map_tile_size, map_tile_size, .4)





#the next scripts constructs a landscape and then a node/edge map without calling builder
#one can fairly easily use a script to build a landscape and as our landscape will be mostly
#set, this (or something similar), is probably what we will use to construct the landscape

map_dict = {}
for co in map_pre_dict.keys():
	exitnodelist = map_pre_dict[co]
	flyable = True
	elevation = 0
	if 'C' in exitnodelist:
		elevation = 1
	if co in non_fly_list:
		flyable = False
	t = tile(map_tile_size, map_tile_size, exitnodelist, flyable, map_road_ratio, co[0]*map_tile_size, co[1]*map_tile_size, elevation)
	map_dict[co] = t

final_map = landscape(map_num_long, map_num_wide, map_dict)

final_map.generate_interface_and_cloud(map_interface_height, map_cloud_height, map_num_cloud_layers, map_cloud_layer_dist, map_cloud_density)


info = final_map.get_nodes_and_edges()
return_nodes = info[0]
edges = info[1]

#building csv files, no longer necessary, but I like it
ID_dict = {}
category_dict = {}

num_parking = 0
num_mark = 0
num_land = 0
num_interface = 0
num_cloud = 0
for n in info[0]:
	x = n.x
	y = n.y
	z = n.z
	ID = n.ID
	ID_dict[ID] = (x,y,z)
	category_dict[ID] = n.category
	#node_writer.writerow([str(ID), str(x), str(y), str(z)])
	if n.category == Category.park:
		num_parking += 1
	elif n.category == Category.mark:
		num_mark += 1
	elif n.category == Category.land:
		num_land += 1
	elif n.category == Category.interface:
		num_interface += 1
	elif n.category == Category.cloud:
		num_cloud += 1

print('num parking: '+str(num_parking))
print('num mark: '+str(num_mark))
print('num land: '+str(num_land))
print('num interface: '+str(num_interface))
print('num cloud: '+str(num_cloud))

#networkx
G = nx.DiGraph()
for n in return_nodes:
	n.turn_to_graph(G)
for e in edges:
	e.turn_to_graph(G)
A = nx.adjacency_matrix(G)
A = nx.to_numpy_matrix(G)
A2 = A.flatten()
A3 = A2.tolist()
A4 = A3[0]
A5 = []
for fl in A4:
	A5.append(int(fl))
#print(A5)

coordinate_list = [None]*len(ID_dict)
w_list = [None]*len(ID_dict)
x_list = [None]*len(ID_dict)
y_list = [None]*len(ID_dict)
z_list = [None]*len(ID_dict)
category_list = [None]*len(ID_dict)
test_list = [1]*len(ID_dict)
for ID in ID_dict:
	coor = ID_dict[ID]
	coordinate_list[ID] = coor
	w_list[ID] = int(coor[0])
	x_list[ID] = int(coor[0]*1000)
	y_list[ID] = int(coor[1]*1000)
	z_list[ID] = int(coor[2]*1000)
	cat = category_dict[ID]
	category_list[ID] = int(cat)

num_nodes = len(ID_dict)

print(len(category_list))

#print(coordinate_list)


def response(req):
	return MapTalkResponse(category_list, x_list, y_list, z_list, num_nodes, A5)

def info_sender():
	rospy.init_node('map_maker_server')
	s = rospy.Service('send_map', MapTalk,response)
	print('ready to send info back')
	rospy.spin()



if __name__ == "__main__":
	info_sender()