/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/
#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <vector>
#include "road.h"
#include "PathPlanner.h"
#include "Road_Vehicle.h"

using namespace std;

/*
from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np
*/

namespace cost_functions
{
	// COST FUNCTIONS
	double lane_change_cost(Trajectory trajectory, int dir);

	double time_diff_cost(Trajectory trajectory, double T);

	double diff_cost(vector<double> coeff, double duration, vector<double> goals, vector<double> sigma, double cost_weight);

	double s_diff_cost(Trajectory trajectory, vector<double> goal_s, vector<double> sigma_s);

	double d_diff_cost(Trajectory trajectory, vector<double> goal_d, vector<double> sigma_d);

	double collision_cost(Trajectory trajectory, vector<Road_Vehicle> road_vehicles);

	double buffer_cost(Trajectory trajectory, vector<Road_Vehicle> road_vehicles);

	double stays_on_road_cost(Trajectory trajectory, Road road, double EgoWidth);

	double exceeds_speed_limit_cost(Trajectory trajectory, Road road);

	double efficiency_cost(Trajectory trajectory, vector<double> goal_s);

	double total_accel_cost(Trajectory trajectory, double EXPECTED_ACC_IN_ONE_SEC);

	double max_accel_cost(Trajectory trajectory, double MAX_ACCEL);

	double max_jerk_cost(Trajectory trajectory, double MAX_JERK);

	double total_jerk_cost(Trajectory trajectory, double EXPECTED_JERK_IN_ONE_SEC);

	double closest_distance_to_any_vehicle(Trajectory trajectory, vector<Road_Vehicle> road_vehicles, double dt);

	double closest_distance_to_vehicle(Trajectory trajectory, Road_Vehicle road_vehicle, double dt);
}
#endif //COST_FUNCTIONS_H