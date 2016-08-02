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

from gurobipy import *

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

def convert_graph(old_graph, old_adj, startend, time_horizon):
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
	na = np.zeros((num_IDs*(th+1),num_IDs*(th+1)))
	# new dict assigns a cost to each edge
	costs = {}
	arcs = tuplelist()

	# the first arcs are "loop back" arcs
	# that connect each goal node to the
	# corresponding start node
	# note that the 'end' node occurs
	# at the last time step
	for start, end in startend:
		arcs.append((end+th*num_IDs,start))


	for ID in range(num_IDs):
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
		for timestep in range(th):
			current_state = ID+timestep*num_IDs
			next_state = ID+(timestep+1)*num_IDs
			na[(current_state, next_state)] = 1
			arcs.append((current_state, next_state))
			# costs are structured so that each robot/edge combo has
			# a unique cost. This should allow us to adjust cost based
			# on robot-specific data such as battery voltage
			for cf in range(cf_num):
				cost_of_waiting = 0.1
				costs[(cf,current_state,next_state)] = cost_of_waiting

			# need to find successors of ID (sID) and connect
			# ID+timestep*num_IDs to sID+(timestep+1)*num_IDs
			old_row = oa[ID]
			for (ID2, value) in enumerate(old_row):
				if value == 1:
					# ID2 is a successor
					next_neighbor = ID2+(timestep+1)*num_IDs
					na[(current_state, next_neighbor)] = 1
					arcs.append((current_state, next_neighbor))
					for cf in range(cf_num):
						cost_of_travel = 0.2
						costs[(cf,current_state, next_neighbor)] = cost_of_travel

def make_model(arcs, costs, startend):

	m = Model('netflow')


	# add a flow variable for each robot-arc pair
	flow = {}
	for cf in range(cf_num):
		for k,a in enumerate(arcs):
			i,j = a
			# the only variables that I want to maximize are
			# the flow variables corresponding to robot i
			# going back to loopback arc i
			# there obj = 1.0 when it is a loopback arc
			# and the cf num corresponds to the arc num
			# otherwise the objective is 0
			loopback = k < len(startend) and cf == k
			flow[cf,i,j] = m.addVar(ub=1.0, obj= 1.0 if loopback else 0.0, vtype=GRB.BINARY,
									name='flow_%s_%s_%s' % (cf, i, j))

	m.update()

	# add capacity constraint on each arc
	# one robot max per arc
	for i, j in arcs:
		m.addConstr(quicksum(flow[cf,i,j] for cf in range(cf_num)) <= 1,
			name='cap_%s_%s' % (i,j))

	# add capacity constraint on each loopback arc
	# aka, only robot i can pass through loopback arc i
	# CRAZYFLIE NUM MUST CORRESPOND TO LOOPBACK ARC NUM
	for i,a in enumerate(startend):
		s, e = a #unpack the tuple
		for cf in range(cf_num):
			if cf != i:
				m.addConstr(flow[cf,s,e] == 0,
					'loopback_cap_%s_%s' % (s,e))

	# flow conservation constraints
	for cf in range(cf_num):
		for j in range(num_IDs):
			m.addConstr(quicksum(flow[cf,i,j] for i,j in arcs.select('*',j)) ==
				quicksum(flow[cf,j,k] for j,k in arcs.select(j,'*')),
					'node_%s_%s' % (h,j))


	# add head-on collision constraint
	for j in range(num_IDs):
		out = arcs.select(j,'*')
		for u_t0, v_t1 in out:
			v_t0 = v_t1 - num_IDs
			u_t1 = u_t0 + num_IDs
			if valid(v_t0) and valid(u_t1):
				v_out = arcs.select(v_t0,'*')
				out_nodes = [y for x, y in v_out]
				if u_t1 in out_nodes:
					m.addConstr(quicksum(flow[cf,u_t0,v_t1] for cf in range(cf_num)) +
						quicksum(flow[cf,v_t0,u_t1] for cf in range(cf_num)) <= 1,
						'head_on_%s_%s' % (u_t0, v_t0))

	# add meet collision constraint
	for v in range(num_IDs):
		v_out = arcs.select(v,'*')
		flow_list = []
		for i,o in v_out:
			for cf in range(cf_num):
				flow_list.append((cf,i,o))
		m.addConstr(quicksum(flow[cf,i,o] for cf,i,o in flow_list) <= 1)


def opt_planner():
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
