/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/
#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <vector>
#include "road.h"

using namespace std;

/*
from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np
*/

namespace cost_functions
{
	// COST FUNCTIONS
	double time_diff_cost(vector<double> trajectory, double T);

	double s_diff_cost(vector<double> trajectory, vector<double> goal_s, vector<double> sigma_s);

	double d_diff_cost(vector<double> trajectory, vector<double> goal_d, vector<double> sigma_d);

	double collision_cost(vector<double> trajectory, vector<vector<double>> sensor_fusion);

	double buffer_cost(vector<double> trajectory, vector<vector<double>> sensor_fusion);

	double stays_on_road_cost(vector<double> trajectory, Road road, double EgoWidth);

	double exceeds_speed_limit_cost(vector<double> trajectory, Road road);

	double efficiency_cost(vector<double> trajectory, vector<double> goal_s);

	double total_accel_cost(vector<double> trajectory, double EXPECTED_ACC_IN_ONE_SEC);

	double max_accel_cost(vector<double> trajectory, double MAX_ACCEL);

	double max_jerk_cost(vector<double> trajectory, double MAX_JERK);

	double total_jerk_cost(vector<double> trajectory, double EXPECTED_JERK_IN_ONE_SEC);
}
#endif //COST_FUNCTIONS_H