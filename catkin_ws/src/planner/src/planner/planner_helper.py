#!/usr/bin/env python

from map_maker import gen_adj_array_info_dict

Category = gen_adj_array_info_dict.Category

mark_x = []
mark_y = []

ENERGY_WEIGHT = 0.2
TIME_WEIGHT = 1 - ENERGY_WEIGHT

def get_cost(info_dict, ID1, ID2, air_vel, land_vel):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = land_vel
	hover_energy = 0
	if c1 != Category.land or c1 != Category.park or c1 != Category.waypoint or c2 != Category.land or c2 != Category.park or c2 != Category.waypoint:
		vel = air_vel
		hover_energy = 0.5
	time_passed = dist_traveled / float(vel)
	energy_expended = dist_traveled * float(vel) + hover_energy

	return(TIME_WEIGHT*time_passed + ENERGY_WEIGHT*energy_expended)

def optimal_cost(info_dict, ID1, ID2, air_vel, land_vel, timestep):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = dist_traveled/timestep
	hover_energy = 0
	if c1 != Category.land or c1 != Category.park or c1 != Category.waypoint or c2 != Category.land or c2 != Category.park or c2 != Category.waypoint:
		hover_energy = 0.5
	time_passed = dist_traveled / float(vel)
	energy_expended = dist_traveled * float(vel) + hover_energy

	return(TIME_WEIGHT*timestep + ENERGY_WEIGHT*energy_expended)