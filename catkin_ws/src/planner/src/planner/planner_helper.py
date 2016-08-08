#!/usr/bin/env python

from map_maker import gen_adj_array_info_dict

Category = gen_adj_array_info_dict.Category

mark_x = []
mark_y = []

def get_cost(info_dict, ID1, ID2, air_vel, land_vel):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = land_vel
	if c1 != Category.land or c1 != Category.park or c1 != Category.waypoint or c2 != Category.land or c2 != Category.park or c2 != Category.waypoint:
		vel = air_vel
	time_passed = dist_traveled / float(vel)
	return(time_passed)