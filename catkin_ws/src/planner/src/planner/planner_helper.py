#!/usr/bin/env python

from map_maker import gen_adj_array_info_dict

Category = gen_adj_array_info_dict.Category

mark_x = []
mark_y = []

cf_mass = .038

def get_time(info_dict, ID1, ID2, air_vel, land_vel):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = land_vel
	if gen_adj_array_info_dict.is_air(c1) or gen_adj_array_info_dict.is_air(fc):
		vel = air_vel
	time_passed = dist_traveled / float(vel)
	return(time_passed)

def get_energy(info_dict, ID1, ID2, travel_time):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	multiplier = 1
	const_energy_expenditure = .25
	if gen_adj_array_info_dict.is_air(c1) or gen_adj_array_info_dict.is_air(fc):
		multiplier = 2
		const_energy_expenditure = .5
	TE = const_energy_expenditure*travel_time
	PE = max((fz-z1), 0)*cf_mass*9.8
	return(TE + PE)

def get_cost(energy, time):
	energy_coefficient = 1
	time_coefficient = 1
	cost = energy*energy_coefficient + time*time_coefficient
	return(cost)

def get_voltage(energy):
	#battery is 240mAh
	return(energy)