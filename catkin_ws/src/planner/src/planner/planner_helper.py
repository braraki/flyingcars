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

def get_energy(info_dict, ID1, ID2, travel_time, air_vel, land_vel):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = land_vel
	if gen_adj_array_info_dict.is_air(c1) or gen_adj_array_info_dict.is_air(fc):
		vel = air_vel
	expected_time = dist_traveled/float(vel)
	if expected_time < travel_time:
		wait_energy = get_wait_energy(info_dict, ID1, travel_time - expected_time)
		move_energy = get_move_energy(info_dict, ID1, ID2, expected_time, air_vel, land_vel)
	else:
		wait_energy = 0
		move_energy = get_move_energy(info_dict, ID1, ID2, travel_time, air_vel, land_vel)
	return(wait_energy + move_energy)

def get_wait_energy(info_dict, ID1, wait_time):
	((x1, y1, z1),c1) = info_dict[ID1]
	if gen_adj_array_info_dict.is_air(c1):
		wait_energy_expenditure = .5
	else:
		wait_energy_expenditure = .05
	return(wait_time * wait_energy_expenditure)

def get_move_energy(info_dict, ID1, ID2, move_time, air_vel, land_vel):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	if gen_adj_array_info_dict.is_air(c1) or gen_adj_array_info_dict.is_air(fc):
		const_energy_expenditure = .5
	else:
		const_energy_expenditure = .125
	TE = const_energy_expenditure*move_time
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