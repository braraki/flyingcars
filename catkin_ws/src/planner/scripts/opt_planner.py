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
from map_maker import gen_adj_array_info_dict

from gurobipy import *

#arguments

used_park_IDs = []

cf_num = int(rospy.get_param('/opt_planner/cf_num'))
z_coefficient = float(rospy.get_param('/opt_planner/z_coefficient'))
continuous = bool(rospy.get_param('/opt_planner/continuous'))
land_vel = float(rospy.get_param('/opt_planner/land_vel'))
air_vel = float(rospy.get_param('/opt_planner/air_vel'))
air_buffer_dist = float(rospy.get_param('/opt_planner/air_buffer_dist'))

count = 0
nontime_IDs = 0
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
		self.dist = {}
	def push(self, item, cost):
		self.data.append((cost, item))
		self.dist[item] = cost
	def pop(self):
		self.data.sort()
		min_node = self.data.pop(0)[1]
		self.dist.pop(min_node,None)
		return min_node
	def is_empty(self):
		return len(self.data) == 0
	def in_queue(self,node):
		return node in self.dist
	def decrease_priority(self,node,cost):
		for i,pair in enumerate(self.data):
			old_cost,item = pair
			if item == node:
				self.data[i] = (cost,item)

 
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

def dijkstra(successors, goal_state):
	distances = {}
	steps = {}
	distances[goal_state] = 0
	steps[goal_state] = 0
	agenda = PriorityQueue()
	agenda.push(goal_state, 0)
	
	while not agenda.is_empty():
		parent = agenda.pop()
		children = successors(parent)
		for child, cost in children:
			alt_cost = distances[parent] + cost
			alt_step = steps[parent] + 1
			if child not in distances:
				print child, alt_step
				distances[child] = alt_cost
				agenda.push(child, alt_cost)
				steps[child] = alt_step
			elif alt_cost < distances[child]:
				print child, alt_step
				distances[child] = alt_cost
				agenda.decrease_priority(child,alt_cost)
				steps[child] = alt_step
		if children == None:
			print "%d HAD NO CHILDREN" % (parent)
	return distances, steps

def true_distances(info_dict, adj_array, goal):
	predecessor_matrix = adj_array.transpose()
	def successors(ID1):
		(x1, y1, z1) = info_dict[ID1][0]
		sucs = []
		row = predecessor_matrix[ID1]
		print row
		for (ID2, value) in enumerate(row):
			if value == 1:
				(fx, fy, fz) = info_dict[ID2][0]
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
				sucs.append((ID2, dist_traveled))
		print "%s succeed %s" % (sucs, ID1)
		return sucs
	return dijkstra(successors, goal)

def convert_graph(old_graph, old_adj, startend, time_horizon):
	og = old_graph
	oa = old_adj
	th = time_horizon
	num_IDs = len(old_graph)

	true_dists = []
	for i in range(len(startend)):
		true_dists.append(true_distances(old_graph,oa,startend[i][1])[0])

	# new adj array
	# the format is this: each node ID is represented by
	# ID, ID+num_IDs, ID+2*num_IDs, .... ID+time_horizon*num_IDs
	# aka ID+(timestep)*num_IDs
	# note that you can retrieve the original ID of one of the
	# new states: old_ID = new_ID % num_IDs
	na = np.zeros((num_IDs*(th+1),num_IDs*(th+1)))
	# new dict assigns a cost to each edge
	costs = {}
	dists = {}
	arcs = tuplelist()

	# the first arcs are "loop back" arcs
	# that connect each goal node to the
	# corresponding start node
	# note that the 'end' node occurs
	# at the last time step
	for start, end in startend:
		arcs.append((end+th*num_IDs,start))
		print end+th*num_IDs, start


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
				dists[(cf,current_state,next_state)] = true_dists[cf][ID]

			# need to find successors of ID (sID) and connect
			# ID+timestep*num_IDs to sID+(timestep+1)*num_IDs
			old_row = oa[ID]
			for (ID2, value) in enumerate(old_row):
				if value == 1:
					# ID2 is a successor
					next_neighbor = ID2+(timestep+1)*num_IDs
					na[(current_state, next_neighbor)] = 1
					arcs.append((current_state, next_neighbor))

					(x1,y1,z1) = old_graph[ID][0]
					(x2,y2,z2) = old_graph[ID2][0]

					distance = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5

					for cf in range(cf_num):
						cost_of_travel = 0.2
						costs[(cf,current_state, next_neighbor)] = cost_of_travel
						dists[(cf,current_state, next_neighbor)] = true_dists[cf][ID2]
	

	print num_IDs
	print len(arcs)
	#print arcs
	return na, costs, arcs, dists

def make_model(arcs, costs, dists, startend, num_IDs, type):

	m, flow = generic_model(arcs,startend,num_IDs)

	if type == 'makespan':
		return makespan_model(m, flow, arcs, costs, startend, num_IDs)
	elif type == 'minmax_distance':
		return minmax_distance_model(m, flow, arcs, dists, startend, num_IDs)
	elif type == 'total_distance':
		return total_distance_model(m, flow, arcs, dists, startend, num_IDs)

def generic_model(arcs,startend,num_IDs):
	m = Model('netflow')

	# add a flow variable for each robot-arc pair
	flow = {}
	for cf in range(cf_num):
		for i,j in arcs:
			# the only variables that I want to maximize are
			# the flow variables corresponding to robot i
			# going back to loopback arc i
			# there obj = 1.0 when it is a loopback arc
			# and the cf num corresponds to the arc num
			# otherwise the objective is 0
			#loopback = k < len(startend) and cf == k
			#flow[cf,i,j] = m.addVar(ub=1.0, obj= 1.0 if loopback else 0.0, vtype=GRB.BINARY,
			#						name='flow_%s_%s_%s' % (cf, i, j))

			flow[cf,i,j] = m.addVar(ub=1.0, obj= 0.0, vtype=GRB.BINARY,
				name='flow_%s_%s_%s' % (cf, i, j))

	m.update()

	# add capacity constraint on each arc
	# one robot max per arc
	for i, j in arcs:
		m.addConstr(quicksum(flow[cf,i,j] for cf in range(cf_num)) <= 1.0,
			name='cap_%s_%s' % (i,j))

	# add capacity constraint on each loopback arc
	# aka, only robot i can pass through loopback arc i
	# CRAZYFLIE NUM MUST CORRESPOND TO LOOPBACK ARC NUM
	for i in range(len(startend)):
		s, e = arcs[i]
		for cf in range(cf_num):
			if cf != i:
				#print cf, s, e
				m.addConstr(flow[cf,s,e] == 0,
					'loopback_cap_%s_%s' % (s,e))

	# flow conservation constraints
	for cf in range(cf_num):
		for node in range(num_IDs):
			m.addConstr(quicksum(flow[cf,i,j] for i,j in arcs.select('*',node)) ==
				quicksum(flow[cf,j,k] for j,k in arcs.select(node,'*')),
					'node_%s_%s' % (cf,node))


	# add head-on collision constraint
	already_checked = []
	for node in range(num_IDs):
		out = arcs.select(node,'*')
		for u_t0, v_t1 in out:
			v_t0 = v_t1 - nontime_IDs
			u_t1 = u_t0 + nontime_IDs
			if valid(v_t0, num_IDs) and valid(u_t1, num_IDs) and u_t0 != v_t0:
				v_out = arcs.select(v_t0,'*')
				out_nodes = [y for x, y in v_out]
				if u_t1 in out_nodes:
					edge1 = (u_t0, v_t1)
					edge2 = (v_t0, u_t1)
					if edge1 not in already_checked:
						already_checked.append(edge1)
						already_checked.append(edge2)
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

	return m, flow

def makespan_model(m, flow, arcs, costs, startend, num_IDs):

	objExpr = LinExpr()
	for i in range(len(startend)):
		s,e = arcs[i]
		cf_ID = i
		objExpr.addTerms(1.0,flow[cf_ID,s,e])

	m.setObjective(objExpr,GRB.MAXIMIZE)

	return m, flow

def minmax_distance_model(m, flow, arcs, costs, startend, num_IDs):

	# this is the max distance, which we will want to minimize
	x_max = m.addVar(vtype=GRB.INTEGER,name="x_max")

	m.update()

	# flow through the loopback arc MUST be 1 for its given robot
	for cf in range(cf_num):
		i,j = arcs[cf]
		m.addConstr(flow[cf,i,j] == 1, 'loopback_filled_%s' %(cf))

	# we want to constrain x_max to be larger than the largest distance
	for cf in range(cf_num):
		# note that arcs[len(startend):] represents the NON loopback arcs
		m.addConstr(quicksum(costs[cf,i,j]*flow[cf,i,j] for i,j in arcs[len(startend):]) <= x_max,
					'max_dist_for_%s' % (cf))

	m.setObjective(x_max,GRB.MINIMIZE)

	return m, flow

def total_distance_model(m, flow, arcs, costs, startend, num_IDs):

	# flow through the loopback arc MUST be 1 for its given robot
	for cf in range(cf_num):
		i,j = arcs[cf]
		m.addConstr(flow[cf,i,j] == 1, 'loopback_filled_%s' %(cf))

	# the objective function is the sum of every nonloopback arc/robot variable times
	# its distance cost
	objExpr = LinExpr()
	for cf in range(cf_num):
		for i,j in arcs[len(startend):]:
			objExpr.addTerms(costs[cf,i,j],flow[cf,i,j])

	m.setObjective(objExpr,GRB.MINIMIZE)

	return m, flow

def valid(node, num_IDs):
	return node >= 0 and node < num_IDs

class full_system:
	def __init__(self, info_dict, adj_array):
		self.adj_array = adj_array
		self.info_dict = info_dict
		self.num_IDs = len(self.info_dict)
		self.system_list = []
		self.pubTime = rospy.Publisher('~time_path_topic',HiPathTime, queue_size=10)
		self.runner()

	def runner(self):
		global nontime_IDs

		nontime_IDs = self.num_IDs

		model_type = 'total_distance'
		startend = tuplelist()
		true_dists = []
		min_time_horizon = 10
		for cf_ID in range(cf_num):
			#sys = system(self.adj_array, self.info_dict, cf_ID, self.pubTime)
			#self.system_list.append(sys)
			(ID1, ID2) = self.request_situation(cf_ID)
			startend.append((ID1,ID2))
			true_dist_list = true_distances(self.info_dict,self.adj_array,startend[cf_ID][1])
			print startend[cf_ID]
			print true_dist_list
			true_dists.append(true_dist_list[1])
			if true_dists[cf_ID][ID1] > min_time_horizon:
				min_time_horizon = true_dists[cf_ID][ID1]
			print "min horizon %d for cf %d" % (min_time_horizon, cf_ID)

		na, costs, arcs, dists = convert_graph(self.info_dict,A,startend,min_time_horizon)

		m, flow = make_model(arcs, costs, dists, startend, self.num_IDs*(min_time_horizon+1),model_type)

		m.optimize()

		time_horizon = min_time_horizon

		while m.status != GRB.Status.OPTIMAL and time_horizon < 2*min_time_horizon*cf_num:
			time_horizon = time_horizon + 1
			na, costs, arcs, dists = convert_graph(self.info_dict,A, startend,time_horizon)
			m, flow = make_model(arcs, costs, dists, startend, self.num_IDs*(time_horizon+1),model_type)
			m.optimize()

		# Print solution
		if m.status == GRB.Status.OPTIMAL:
			print "TIME HORIZON: %d" % (time_horizon)
			solution = m.getAttr('x', flow)
			for h in range(cf_num):
				print('\nOptimal flows for %s:' % h)
				for i,j in arcs:
					if solution[h,i,j] > 0:
						print('%s -> %s: %g' % (i, j, solution[h,i,j]))

			paths,times = self.extract_paths(solution, arcs)
			for cf in range(cf_num):
				print cf, paths[cf]
				#print times[cf]
				self.pubTime.publish(cf_num,cf,paths[cf],times[cf])

	def extract_paths(self,solution,arcs):
		paths = {}
		times = {}
		for cf in range(cf_num):
			path = tuplelist()
			for i,j in arcs:
				if solution[cf,i,j] > 0:
					path.append((i,j))
			first_node = path.pop(0)[1]
			#print path
			first_timestep = 0
			ordered_path = self.order_path(path,first_node)
			time_adjusted_path = self.time_adjust(ordered_path)
			paths[cf] = time_adjusted_path
			current_time = time.time()
			times[cf] = [1*x for x in range(0,len(paths[cf]))]
		return paths,times

	def order_path(self,path,node):
		if path:
			edge = path.select(node,'*')[0]
			next_node = edge[1]
			path.remove(edge)
			return [node] + self.order_path(path,next_node)
		else:
			return [node]

	def time_adjust(self,path):
		new_path = []
		for timestep,node in enumerate(path):
			new_node = node - timestep*self.num_IDs
			new_path.append(new_node)
		return new_path

	def request_situation(self,cf_ID):
			global count
			print(count)
			count += 1
			#print('situation asking')
			rospy.wait_for_service('send_situation')
			try:
				#print('calling')
				func = rospy.ServiceProxy('send_situation', situation)
				resp = func(cf_ID)
				return((resp.start_ID, resp.end_ID))
			except rospy.ServiceException, e:
				t = 1
				#print("service call failed")

if __name__ == "__main__":
	rospy.init_node('opt_planner', anonymous = True)
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_complex_map')
	Category = gen_adj_array_info_dict.Category
	fs = full_system(info_dict, A)