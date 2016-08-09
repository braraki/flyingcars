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

import tf

import thread

from map_maker import gen_adj_array_info_dict

#arguments
#thickness of the tile (generally)
tilethickness = float(rospy.get_param('/simple_marker/tilethickness'))
#thickness of the road
roadthickness = float(rospy.get_param('/simple_marker/roadthickness'))
#represents how close to the edge of the tile the parking spot will be
#at a roadratio around .75, a parking frac of 1.0 is needed
#at a roadratio of .5, a parking frac of .5 is most asthetically pleasing (in my opinion)
parking_frac = float(rospy.get_param('/simple_marker/parking_frac'))
#print(robot_description)
air_node_display = bool(rospy.get_param('/simple_marker/air_node_display'))
waypoint_node_display = bool(rospy.get_param('/simple_marker/waypoint_node_display'))
buildings = str(rospy.get_param('/simple_marker/buildings'))
display_frequency = int(rospy.get_param('/simple_marker/display_frequency'))
if buildings == 'True':
	buildings = True
else:
	buildings = False

house_ID = 0
reps = 0

def processFeedback(feedback):
	p = feedback.pose.position
	print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

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

class visual_node:
	def __init__(self, ID, x, y, z, category = None, successors = [], precursors = []):
		self.ID = ID
		self.x = x
		self.y = y 
		self.z = z
		if self.z == 0:
			self.color = (0, 255, 0)
		else:
			self.color = (0, 0, 255)
		self.successors = successors[:]
		self.precursors = precursors[:]
		if category != None:
			self.categorize(category)
		self.tile = None

	def add_successor(self, node):
		if node not in self.successors:
			self.successors.append(node)

	def add_precursor(self, node):
		if node not in self.precursors:
			self.precursors.append(node)

	def categorize(self, category):
		#print(category)
		self.category = category
		if category == Category.land or category == Category.park:
			self.color = (0, 255, 0)
		elif category == Category.interface:
			self.color = (0, 0, 255)
		elif category == Category.cloud:
			self.color = (255, 255, 255)
		elif category == Category.waypoint or category == Category.air_waypoint:
			self.color = (255, 0, 255)
		else:
			self.color = (0, 0, 0)

	def construct(self, int_marker):
		n_marker = Marker()
		n_marker.type = Marker.CUBE
		n_marker.scale.x = .1*.5
		n_marker.scale.y = .1*.5
		n_marker.scale.z = .1*.5
		(n_marker.color.r, n_marker.color.g, n_marker.color.b) = self.color	
		n_marker.color.a = .5
		n_marker.pose.position.x = self.x
		n_marker.pose.position.y = self.y
		n_marker.pose.position.z = self.z
			
		n_control = InteractiveMarkerControl()
		n_control.always_visible = True
		n_control.markers.append( n_marker )
		int_marker.controls.append(n_control)

		return(int_marker)

	def assign_tile(self, t):
		self.tile = t

class visual_edge:
	def __init__(self, ID, node1ID, node2ID, node1 = None, node2 = None):
		self.ID = ID
		self.node1ID = node1ID
		self.node2ID = node2ID
		self.node1 = node1
		self.node2 = node2

	def construct(self, int_marker):
		x1 = self.node1.x
		y1 = self.node1.y
		z1 = self.node1.z
		x2 = self.node2.x
		y2 = self.node2.y
		z2 = self.node2.z

		color = (80, 80, 80)
		a_marker = Marker()
		a_marker.type = Marker.ARROW
		a_marker.scale.x = .05*.5
		a_marker.scale.y = .1*.5
		a_marker.scale.z = .1*.5
		(a_marker.color.r, a_marker.color.g, a_marker.color.b) = color
		a_marker.color.a = .5
		start = Point()
		end = Point()

		start.x = self.node1.x
		start.y = self.node1.y
		start.z = self.node1.z
		end.x = self.node2.x
		end.y = self.node2.y
		end.z = self.node2.z

		a_marker.points.append(start)
		a_marker.points.append(end)
			
		a_control = InteractiveMarkerControl()
		a_control.always_visible = True
		a_control.markers.append( a_marker )
		int_marker.controls.append(a_control)

		return(int_marker)

class node_scape:
	def __init__(self, info_dict, adjacency_matrix):
		self.info_dict = info_dict
		self.adjacency_matrix = adjacency_matrix
		self.node_list = []
		self.node_ID_dict = {}
		self.edge_list = []
		for ID in info_dict:
			info = info_dict[ID]
			coor = info[0]
			cat = info[1]
			n = visual_node(ID, coor[0], coor[1], coor[2], cat)
			self.node_list.append(n)
			self.node_ID_dict[ID] = n
		edge_num = 0
		for (ID1, row) in enumerate(adjacency_matrix):
			for (ID2, value) in enumerate(row):
				if value == 1:
					n1 = self.node_ID_dict[ID1]
					n2 = self.node_ID_dict[ID2]
					e = visual_edge(edge_num, ID1, ID2, n1, n2)
					self.edge_list.append(e)
					edge_num += 1
		self.successor_precursors()
		self.tile_dict = {}

	#turns edges into successors and precursors for nodes
	def successor_precursors(self):
		print('edge work')
		for e in self.edge_list:
			n1 = e.node1
			n2 = e.node2
			n1.add_successor(n2)
			n2.add_precursor(n1)
		print('edge work over')

	def construct(self):
		#rospy.Subscriber('commands', String, self.interpret)

		server = InteractiveMarkerServer("simple_marker")
		
		#rospy.Subscriber('commands', String, self.interpret)
		
		
		# create an interactive marker for our server
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "base_link"
		int_marker.name = "my_marker"

		for n in self.node_list:
			if True:#n.category == 'land':
				int_marker = n.construct(int_marker)
			# 'commit' changes and send to all clients
		
		for e in self.edge_list:
			node1 = e.node1
			node2 = e.node2
			if node1.category != Category.cloud or node2.category != Category.cloud:
				int_marker = e.construct(int_marker)
		

		server.insert(int_marker, processFeedback)

		server.applyChanges()
		rospy.spin()

class building_scape:
	def __init__(self, node_scape, mark_x, mark_y):
		self.mark_x = mark_x
		self.mark_y = mark_y
		self.node_scape = node_scape
		self.tile_dict = {}
		self.crazyflie_list = []
		self.cf_num = None
		self.server = InteractiveMarkerServer("simple_marker")
		rospy.Subscriber('~time_path_topic', HiPathTime, self.respond)
		rospy.Subscriber('~SimPos_topic', SimPos, self.pos_respond)
		rospy.Subscriber('~Start_SimPos_topic', SimPos, self.pos_respond)

	#does all of the tile work (roadratio, flyable, etc.)
	def build_tiles(self):
		'''
		markxs = []
		markys = []
		z_dict = {}
		base_road_ratio = None
		for n in self.node_scape.node_list:
			if n.category == Category.mark:
				if n.x not in markxs:
					markxs.append(n.x)
				if n.y not in markys:
					markys.append(n.y)
				z_dict[(n.x, n.y)] = n.z
		markxs = sorted(markxs)
		markys = sorted(markys)
		'''
		base_road_ratio = None
		markxs = list(set(self.mark_x))
		markys = list(set(self.mark_y))
		markxs = sorted(markxs)
		markys = sorted(markys)
		length = markxs[1] - markxs[0]
		width = markys[1] - markys[0]
		for x in markxs:
			c_x = x + length*.5
			for y in markys:
				c_y = y + length*.5
				z = 0#z_dict[(x, y)]
				t = tile(c_x, c_y, z, length, width)
				self.tile_dict[(c_x, c_y)] = t
				for n in self.node_scape.node_list:
					if t.test_node(n):
						t.assign_node(n)
		for t in self.tile_dict.values():
			t.build_exitnodelist()
			t.build_flyable(self.node_scape.node_list)
			if base_road_ratio == None:
				base_road_ratio = t.return_road_ratio()
		if base_road_ratio == None:
			base_road_ratio = .5 #just as a default
		for t in self.tile_dict.values():
			t.assign_road_ratio(base_road_ratio)

		#getting angle for houses
		opts = {(length, 0):0, (-1*length, 0):math.pi, (0, width):math.pi*.5, (0, -1*width):math.pi*1.5}
		for spot in self.tile_dict:
			(x, y) = spot
			t = self.tile_dict[spot]
			if t.land_nodes+t.park_nodes+t.exitnodelist == []:
				k = opts.keys()
				random.shuffle(k)
				for o in k:
					if (x+o[0], y+o[1]) in self.tile_dict:
						n = self.tile_dict[(x+o[0], y+o[1])]
						if n.exitnodelist != []:
							t.assign_theta(opts[o])
							break

	#response to path info, builds and updates path
	def respond(self, data):
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.path)
		if self.cf_num == None:
			self.cf_num = data.num_IDs
			self.crazyflie_list = [None]*self.cf_num
		p = data.path
		if self.crazyflie_list[data.ID] == None:
			cf = crazyflie(data.ID, p, self.server, self.node_scape)
			self.crazyflie_list[data.ID] = cf
			cf.construct_path()
		else:
			cf = self.crazyflie_list[data.ID]
			cf.update_path(p)

	#response to position information, builds and updates crazyflie
	#using reps to smooth out motion
	def pos_respond(self, data):
		global reps
		if reps%display_frequency == 0:
			if len(self.crazyflie_list) != len(data.x):
				self.cf_num = len(data.x)
				self.crazyflie_list = [None]*self.cf_num
				for cf_ID in range(len(data.x)):
					cf = crazyflie(cf_ID, [], self.server, self.node_scape)
					self.crazyflie_list[cf_ID] = cf
			for index in range(len(data.x)):
				x = data.x[index]
				y = data.y[index]
				z = data.z[index]
				print((x,y,z))
				if self.crazyflie_list[index] != None:
					cf = self.crazyflie_list[index]
					cf.update_flie((x,y,z))
					cf.construct_flie(False)
			self.server.applyChanges()
		reps += 1
	'''
	def fluid_construct(self):
		rospy.Subscriber('~time_path_topic', HiPathTime, self.respond)
		rospy.Subscriber('~SimPos_topic', SimPos, self.pos_respond)
		rospy.Subscriber('~Start_SimPos_topic', SimPos, self.pos_respond)
		#self.server = InteractiveMarkerServer("simple_marker")
		#rospy.spin()
	'''

	def construct(self):
		#self.node_scape.construct()
		
		#rospy.Subscriber('~time_path_topic', HiPathTime, self.respond)
		#rospy.Subscriber('~SimPos_topic', SimPos, self.pos_respond)
		
		#thread.start_new_thread ( self.fluid_construct , ())

		# create an interactive marker for our server
		n_int_marker = InteractiveMarker()
		n_int_marker.header.frame_id = "base_link"
		n_int_marker.name = "my_node_marker"

		for n in self.node_scape.node_list:
			if air_node_display:
				n_int_marker = n.construct(n_int_marker)
			elif n.category != Category.cloud and n.category != Category.interface:
				n_int_marker = n.construct(n_int_marker)

		e_int_marker = InteractiveMarker()
		e_int_marker.header.frame_id = "base_link"
		e_int_marker.name = "my_edge_marker"

		for e in self.node_scape.edge_list:
			node1 = e.node1
			node2 = e.node2
			if node1.category != Category.cloud and node2.category != Category.cloud:
				if node1.category != Category.interface and node2.category != Category.interface:
					if node1.category != Category.air_waypoint and node2.category != Category.air_waypoint:
						e_int_marker = e.construct(e_int_marker)

		t_int_marker = InteractiveMarker()
		t_int_marker.header.frame_id = "base_link"
		t_int_marker.name = "my_tile_marker"

		tiles = self.tile_dict.values()
		random.shuffle(tiles)
		for t in tiles:
			t_int_marker = t.construct(t_int_marker)
		thread.start_new_thread(self.construct_2, (n_int_marker, e_int_marker, t_int_marker))
		#self.construct_2(n_int_marker, e_int_marker, t_int_marker)
		time.sleep(1000000000)
		
	def construct_2(self, im1, im2, im3):
		self.server.insert(im1, processFeedback)
		self.server.insert(im2, processFeedback)
		self.server.insert(im3, processFeedback)
		self.server.applyChanges()
		rospy.spin()

class crazyflie:
	def __init__(self, ID, path, server, node_scape):
		self.ID = ID
		self.position = None
		self.server = server
		self.node_scape = node_scape

		self.path = []
		for id in path:
			if id in self.node_scape.node_ID_dict:
				self.path.append(id)

		self.int_marker2 = InteractiveMarker()
		self.int_marker2.header.frame_id = "base_link"
		self.int_marker2.name = "my_marker2_cf"+str(self.ID)

		self.int_marker3 = InteractiveMarker()
		self.int_marker3.header.frame_id = "base_link"
		self.int_marker3.name = "my_marker3_cf"+str(self.ID)

		self.broadcaster = tf.TransformBroadcaster()

	def construct_path(self):
		self.int_marker2 = InteractiveMarker()
		self.int_marker2.header.frame_id = "base_link"
		self.int_marker2.name = "my_marker2_cf"+str(self.ID)
		for index in range(len(self.path)-1):
			ID1 = self.path[index]
			ID2 = self.path[index + 1]
			n1 = self.node_scape.node_ID_dict[ID1]
			n2 = self.node_scape.node_ID_dict[ID2]

			color = (255, 0, 0)
			a_marker = Marker()
			a_marker.type = Marker.ARROW
			a_marker.scale.x = .05*.5
			a_marker.scale.y = .1*.5
			a_marker.scale.z = .1*.5
			(a_marker.color.r, a_marker.color.g, a_marker.color.b) = color
			a_marker.color.a = 1
			start = Point()
			end = Point()

			start.x = n1.x
			start.y = n1.y
			start.z = n1.z
			end.x = n2.x
			end.y = n2.y
			end.z = n2.z

			a_marker.points.append(start)
			a_marker.points.append(end)
				
			a_control = InteractiveMarkerControl()
			a_control.always_visible = True
			a_control.markers.append( a_marker )
			self.int_marker2.controls.append(a_control)

		self.server.insert(self.int_marker2, processFeedback)

		self.server.applyChanges()

	#updates path and removes waypoints
	def update_path(self, path):
		if path != self.path:
			self.path = []
			for id in path:
				if id in self.node_scape.node_ID_dict:
					self.path.append(id)
			self.construct_path()

	def construct_flie(self, apply_changes = True):
		self.broadcaster.sendTransform((self.position[0], self.position[1], self.position[2]+.025),
			tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "crazy_flie"+str(self.ID)+"/base_link", "base_link")
		
		if True:
			self.int_marker3 = InteractiveMarker()
			self.int_marker3.header.frame_id = "base_link"
			self.int_marker3.name = "my_marker3_cf"+str(self.ID)
			
			cf_marker = Marker()
			cf_marker.type = Marker.CYLINDER
			cf_marker.scale.x = .15
			cf_marker.scale.y = .15
			cf_marker.scale.z = .03

			cf_marker.color.r = 1
			cf_marker.color.g = 1
			cf_marker.color.b = 0
			cf_marker.color.a = 1

			cf_marker.pose.position.x = self.position[0]
			cf_marker.pose.position.y = self.position[1]
			cf_marker.pose.position.z = self.position[2]
			
			cf_control = InteractiveMarkerControl()
			cf_control.always_visible = True
			cf_control.markers.append( cf_marker )
			self.int_marker3.controls.append(cf_control)

			self.server.insert(self.int_marker3, processFeedback)
			if apply_changes:
				self.server.applyChanges()
		
	def update_flie(self, pos):
		self.position = pos

class tile:
	def __init__(self, x, y, z, length, width):
		self.x = x
		self.y = y
		self.z = z
		self.length = length
		self.width = width
		self.land_nodes = []
		self.park_nodes = []
		self.exitnodelist = []
		self.road_ratio = None
		#flyable default is true
		self.flyable = True

		self.theta = None
		self.house = None

	def assign_theta(self, theta):
		self.theta = theta

	#finds if node is contained in tile
	def test_node(self, n):
		if n.category == Category.land or n.category == Category.park:
			if n.x >= self.x - .5*self.length and n.x <= self.x + .5*self.length:
				if n.y >= self.y - .5*self.width and n.y <= self.y + .5*self.length:
					return(True)
		return(False)

	#tests and assigns node if proper
	def assign_node(self, n):
		if n not in self.land_nodes + self.park_nodes:
			if n.z > self.z:
				self.z = n.z
			if n.category == Category.land:
				self.land_nodes.append(n)
				n.assign_tile(self)
			elif n.category == Category.park:
				self.park_nodes.append(n)
				n.assign_tile(self)

	#determines road_ratio
	def return_road_ratio(self):
		if 'C' in self.exitnodelist or self.exitnodelist == []:
			return(None)
		running_r = 0
		for n in self.land_nodes:
			x_dist = abs(n.x - self.x)
			y_dist = abs(n.y - self.y)
			x_r = (x_dist*4)/self.length
			y_r = (y_dist*4)/self.width
			if x_r >= running_r:
				running_r = x_r
			if y_r >= running_r:
				running_r = y_r
		if running_r == 0:
			return(None)
		print(running_r)
		return(running_r)

	def assign_road_ratio(self, ratio):
		self.road_ratio = ratio

	#reconstructs the exitnodelist
	def build_exitnodelist(self):
		for ln in self.land_nodes:
			if 'C' not in self.exitnodelist:
				self.exitnodelist.append('C')
			for suc in ln.successors:
				if suc.category == Category.land:
					t = suc.tile
					if t.x > self.x and 'E' not in self.exitnodelist:
						self.exitnodelist.append('E')
					elif t.x < self.x and 'W' not in self.exitnodelist:
						self.exitnodelist.append('W')
					elif t.y > self.y and 'N' not in self.exitnodelist:
						self.exitnodelist.append('N')
					elif t.y < self.y and 'S' not in self.exitnodelist:
						self.exitnodelist.append('S')
		if len(self.exitnodelist) > 1:
			self.exitnodelist.remove('C')

	#determines if flyable
	def build_flyable(self, full_node_list):
		f = False
		all_nodes = self.land_nodes + self.park_nodes
		if len(all_nodes)>0:
			for n in self.land_nodes + self.park_nodes:
				suc = n.successors
				for n2 in suc:
					if n2.category == Category.interface or n2.category == Category.cloud:
						f = True
						break
		else:
			for n in full_node_list:
				if n.category == Category.cloud or n.category == Category.interface:
					if n.x >= self.x - .5*self.length and n.x <= self.x + .5*self.length:
						if n.y >= self.y - .5*self.width and n.y <= self.y + .5*self.length:
							f = True
		self.flyable = f

	def construct(self, int_marker):
		if buildings:
			if self.exitnodelist+self.park_nodes+self.land_nodes == []:
				if self.house == None:
					if self.theta == None:
						theta = random.choice([0, math.pi*.5, math.pi, math.pi*1.5])
					else:
						theta = self.theta
					h = house(self.x, self.y, self.z, theta)
					self.house = h
					thread.start_new_thread ( self.house.construct , ())
		#base
		base_marker = Marker()
		base_marker.type = Marker.CUBE
		base_marker.scale.x = self.length
		base_marker.scale.y = self.width
		base_marker.scale.z = tilethickness + self.z

		if self.flyable:
			base_marker.color.r = 0.0
			base_marker.color.g = 1.0
			base_marker.color.b = 0.0			
		else:
			base_marker.color.r = 1.0
			base_marker.color.g = 0.5
			base_marker.color.b = 0.0
		base_marker.color.a = 1.0

		base_marker.pose.position.x = self.x
		base_marker.pose.position.y = self.y
		base_marker.pose.position.z = -.5*tilethickness+(-1*roadthickness)+self.z*.5
		
		base_control = InteractiveMarkerControl()
		base_control.always_visible = True
		base_control.markers.append( base_marker )
		int_marker.controls.append(base_control)

		#circle
		if len(self.exitnodelist) > 0:
			cylinder_marker = Marker()
			cylinder_marker.type = Marker.CYLINDER
			cylinder_marker.scale.x = self.road_ratio*self.length
			cylinder_marker.scale.y = self.road_ratio*self.width
			cylinder_marker.scale.z = roadthickness

			cylinder_marker.color.r = 0.2
			cylinder_marker.color.g = 0.2
			cylinder_marker.color.b = 0.2
			cylinder_marker.color.a = 1.0

			cylinder_marker.pose.position.x = self.x
			cylinder_marker.pose.position.y = self.y
			cylinder_marker.pose.position.z = -.5*roadthickness+self.z

			cylinder_control = InteractiveMarkerControl()
			cylinder_control.always_visible = True
			cylinder_control.markers.append( cylinder_marker )
			int_marker.controls.append(cylinder_control)

		#parking
		for n in self.park_nodes:
			cylinder_marker = Marker()
			cylinder_marker.type = Marker.CYLINDER
			cylinder_marker.scale.x = (1-self.road_ratio)*self.length*parking_frac
			cylinder_marker.scale.y = (1-self.road_ratio)*self.width*parking_frac
			cylinder_marker.scale.z = roadthickness

			cylinder_marker.color.r = 0.2
			cylinder_marker.color.g = 0.2
			cylinder_marker.color.b = 0.2
			cylinder_marker.color.a = 1.0			
			if n.x - self.x == 0:
				x_center = 0
			elif (abs(n.x - self.x)-.25*self.length*self.road_ratio) < .01:
				x_center = n.x - self.x
			else:
				x_center = abs(n.x - self.x)/float(n.x - self.x)*self.road_ratio*self.length*.5
			if n.y - self.y == 0:
				y_center = 0
			elif (abs(n.y - self.y)-.25*self.width*self.road_ratio) < .01:
				y_center = n.y - self.y
			else:
				y_center = abs(n.y - self.y)/float(n.y - self.y)*self.road_ratio*self.width*.5
			cylinder_marker.pose.position.x = self.x + x_center
			cylinder_marker.pose.position.y = self.y + y_center
			cylinder_marker.pose.position.z = -.5*roadthickness+self.z

			cylinder_control = InteractiveMarkerControl()
			cylinder_control.always_visible = True
			cylinder_control.markers.append( cylinder_marker )
			int_marker.controls.append(cylinder_control)


		#road
		for letter in self.exitnodelist:
			new_x = self.x
			new_y = self.y
			new_w = self.width
			new_l = self.length
			l_frac = .08
			w_frac = .08
			if letter != 'C':
				if letter == 'N':
					new_y += self.width*.25
					new_l = self.road_ratio*self.length
					new_w = self.width*.5
					w_frac = .5
				if letter == 'S':
					new_y -= self.width*.25
					new_l = self.road_ratio*self.length
					new_w = self.width*.5
					w_frac = .5
				if letter == 'E':
					new_x += self.length*.25
					new_w = self.road_ratio*self.width
					new_l = self.length*.5
					l_frac = .5
				if letter == 'W':
					new_x -= self.length*.25
					new_w = self.road_ratio*self.width
					new_l = self.length*.5
					l_frac = .5
				road_marker = Marker()
				road_marker.type = Marker.CUBE
				road_marker.scale.x = new_l
				road_marker.scale.y = new_w
				road_marker.scale.z = roadthickness
				road_marker.color.r = 0.2
				road_marker.color.g = 0.2
				road_marker.color.b = 0.2

				road_marker.color.a = 1.0
				road_marker.pose.position.x = new_x
				road_marker.pose.position.y = new_y
				road_marker.pose.position.z = -.5*roadthickness+self.z

				road_control = InteractiveMarkerControl()
				road_control.always_visible = True
				road_control.markers.append( road_marker )
				
				int_marker.controls.append(road_control)

				#yellow line
				if len(self.exitnodelist)<3:
					line_marker = Marker()
					line_marker.type = Marker.CUBE
					line_marker.scale.x = new_l*l_frac
					line_marker.scale.y = new_w*w_frac
					line_marker.scale.z = roadthickness*1.15
					line_marker.color.r = 1
					line_marker.color.g = 1
					line_marker.color.b = 0
					line_marker.color.a = 1
					line_marker.pose.position.x = new_x
					line_marker.pose.position.y = new_y
					line_marker.pose.position.z = -.5*roadthickness+self.z

					line_control = InteractiveMarkerControl()
					line_control.always_visible = True
					line_control.markers.append( line_marker )

					int_marker.controls.append(line_control)

		return(int_marker)


class house:
	def __init__(self, x, y, z, theta):
		global house_ID
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta
		self.broadcaster = tf.TransformBroadcaster()
		self.ID = house_ID
		print('ID: '+str(self.ID))
		house_ID += 1

	def construct(self):
		while True:
			self.broadcaster.sendTransform((self.x, self.y, self.z - roadthickness),
				tf.transformations.quaternion_from_euler(0, 0, self.theta), rospy.Time.now(), "house"+str(self.ID)+"/whole", "base_link")
			time.sleep(.1)
		#self.broadcaster.publishFixedTransforms()

#ns = node_scape('nodes.csv', 'edges.csv')
#ns.analyse()
#ns.construct()




'''
def map_maker_client():
	rospy.wait_for_service('send_map')
	rospy.wait_for_service('send_complex_map')
	try:
		print('calling')
		info_dict = {}
		func = rospy.ServiceProxy('send_map', MapTalk)
		resp = func()
		func_complex = rospy.ServiceProxy('send_complex_map', MapTalk)
		resp_complex = func_complex()
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
		if waypoint_node_display:
			for ID in range(num_IDs, resp_complex.num_IDs):
				c = static_category_dict[resp_complex.category_list[ID]]
				if c == Category.waypoint:
					x = (resp_complex.x_list[ID])
					y = (resp_complex.y_list[ID])
					z = (resp_complex.z_list[ID])
					info_dict[ID] = ((x, y, z), c)
		#print(info_dict)
		ns = node_scape(info_dict, A)
		#ns.construct()
		bs = building_scape(ns)
		bs.build_tiles()
		bs.construct()
	except rospy.ServiceException, e:
		print("service call failed")
'''

if __name__ == "__main__":
	print('test')
	rospy.init_node("simple_marker")
	if waypoint_node_display:
		info_dict = gen_adj_array_info_dict.map_maker_client('send_complex_map')[0]
		A = gen_adj_array_info_dict.map_maker_client('send_map')[1]
	else:
		(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_map')
	(mark_x, mark_y) = gen_adj_array_info_dict.get_marks()
	Category = gen_adj_array_info_dict.Category
	ns = node_scape(info_dict, A)
	#ns.construct()
	bs = building_scape(ns, mark_x, mark_y)
	bs.build_tiles()
	bs.construct()
