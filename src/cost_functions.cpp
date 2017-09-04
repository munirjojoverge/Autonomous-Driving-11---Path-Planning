/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/
#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>      // std::accumulate
#include <algorithm>    // std::max, min
#include "cost_functions.h"
#include "utils.h"

using namespace std;
using namespace utils;

// priority levels for costs: Higher COST will give you a higher priority to AVOID

const double TIME_DIFF  = 10 ^ 1;
const double S_DIFF     = 10 ^ 3;
const double D_DIFF     = 10 ^ 4;
const double EFFICIENCY = 10 ^ 2;
const double MAX_JERK   = 10 ^ 8;
const double TOTAL_JERK = 10 ^ 7;
const double COLLISION  = 10 ^ 10;
const double DANGER     = 10 ^ 9;
const double MAX_ACCEL  = 10 ^ 6;
const double TOTAL_ACCEL = 10 ^ 5;

/*
from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np
*/
namespace cost_functions
{
	// COST FUNCTIONS
	double time_diff_cost(vector<double> trajectory, double T)
	{
		/*
		Penalizes trajectories that span a duration which is longer or
		shorter than the duration requested (T)f.
		*/

		// the last element on the trajectory should hold the last time value, i.e the total duration
		double t = trajectory[12];
		return TIME_DIFF * (logistic(fabs(t - T) / T));
	}

	double s_diff_cost(vector<double> trajectory, vector<double> goal_s, vector<double> sigma_s)
	{
		/*
		Penalizes trajectories whose s coordinate(and derivatives)
		differ from the goal.
		*/
		//cout << "23 - Traj Size: " << trajectory.size() << endl;
		double cost = 0.0;
		vector<double> s_coeff;
		//cout << "24 - parse Traj" << endl;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));
		double t = trajectory[12];

		//cout << "25 - s_diff_cost: t = " << t << endl;
		vector<double> evals = evaluate_f_and_N_derivatives(s_coeff, t, 2);
		//cout << "26 - Evaluating f and N derivatives Done. Size:" << evals.size() << endl;

		for (size_t i = 0; i < evals.size(); i++)
		{
			double diff = fabs(evals[i] - goal_s[i]);
			cost += logistic(diff / sigma_s[i]);
		}
		//cout << "27 - s_diff_coeff Cost Calculated " << endl;
		return S_DIFF * cost;
	}

	double d_diff_cost(vector<double> trajectory, vector<double> goal_d, vector<double> sigma_d)
	{
		/*
		Penalizes trajectories whose d coordinate(and derivatives)
		differ from the goal.
		*/
		double cost = 0.0;
		vector<double> d_coeff;
		std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));
		double t = trajectory[12];

		vector<double> evals = evaluate_f_and_N_derivatives(d_coeff, t, 2);
		for (size_t i = 0; i < evals.size(); i++)
		{
			double diff = fabs(evals[i] - goal_d[i]);
			cost += logistic(diff / sigma_d[i]);
		}
		return D_DIFF * cost;
	}

	double collision_cost(vector<double> trajectory, vector<vector<double>> sensor_fusion)
	{
		/*
		Binary cost function which penalizes collisions.
		*/
		double cost = 0.0;
		/*
		nearest = nearest_approach_to_any_vehicle(traj, predictions)
		if nearest < 2 * VEHICLE_RADIUS : return 1.0
		else : return 0.0;
		*/
		return COLLISION * cost;
	}

	double buffer_cost(vector<double> trajectory, vector<vector<double>> sensor_fusion)
	{
		/*
		Penalizes getting close to other vehicles.
		*/
		double cost = 0.0;
		/*
		nearest = nearest_approach_to_any_vehicle(traj, predictions)
		return logistic(2 * VEHICLE_RADIUS / nearest)
		*/
		return DANGER * cost;
	}

	double stays_on_road_cost(vector<double> trajectory, Road road, double EgoWidth)
	{		
		
		double RoadWidth = road.num_lanes * road.lane_width;

		vector<double> d_coeff;
		std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));
		double T = trajectory[12];
		double dt = T / 100.0;
		double t = 0.0;
		bool OnRoad = true;
		
		while (OnRoad && t<T)
		{
			double d = evaluate(d_coeff, t);
			OnRoad = ((d > EgoWidth) && (d < (RoadWidth - EgoWidth)));
			t += dt;
		}	
		return DANGER * 1; // ((OnRoad) ? 0 : 1);

	}

	double exceeds_speed_limit_cost(vector<double> trajectory, Road road)
	{
		vector<double> s_coeff;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));
		
		double T = trajectory[12];
		double dt = T / 100.0;
		double t = 0;
		vector<double> s_dot = differentiate(s_coeff);
		bool UnderSpeeLimit = true;
		while (UnderSpeeLimit && t<T)
		{
			double Speed = evaluate(s_dot, t);
			UnderSpeeLimit = (Speed <= road.speed_limit);
			t += dt;
		}
		return DANGER * ((UnderSpeeLimit) ? 0 : 1);
	}

	double efficiency_cost(vector<double> trajectory, vector<double> goal_s)
	{
		/*
		Rewards high average speeds.
		*/
		vector<double> s_coeff;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));
		double t = trajectory[12];

		double avg_v = evaluate(s_coeff, t) / t;

		return EFFICIENCY * (logistic(2 * (goal_s[1] - avg_v) / avg_v));
	}

	double total_accel_cost(vector<double> trajectory, double EXPECTED_ACC_IN_ONE_SEC)
	{
		vector<double> s_coeff;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

		// I should calculate also d accelerations...
		vector<double> d_coeff;
		std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

		double T = trajectory[12];

		vector<double> s_dot = differentiate(s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		double total_acc = 0.0;
		double dt = T / 100.0;
		for (size_t i = 1; i <= 100; i++)
		{
			double t = dt * i;
			total_acc += fabs(evaluate(s_d_dot, t)*dt);
		}
		double acc_per_second = total_acc / T;

		return TOTAL_ACCEL * (logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC));
	}

	double max_accel_cost(vector<double> trajectory, double MAX_ACCEL)
	{
		vector<double> s_coeff;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

		vector<double> d_coeff;
		std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

		double T = trajectory[12];

		vector<double> s_dot = differentiate(s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		double dt = T / 100.0;
		vector<double> all_accs;
		for (size_t i = 1; i <= 100; i++)
		{
			all_accs.push_back(evaluate(s_d_dot, (dt * i)));
		}
		double max_acc = *std::max_element(all_accs.begin(), all_accs.end());

		return MAX_ACCEL * ((fabs(max_acc) > MAX_ACCEL) ? 1 : 0);
	}

	double max_jerk_cost(vector<double> trajectory, double MAX_JERK)
	{
		vector<double> s_coeff;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

		vector<double> d_coeff;
		std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

		double T = trajectory[12];

		vector<double> s_dot = differentiate(s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		vector<double> jerk = differentiate(s_d_dot);

		double dt = T / 100.0;
		vector<double> all_jerks;
		for (size_t i = 1; i <= 100; i++)
		{
			all_jerks.push_back(evaluate(jerk, (dt * i)));
		}
		double max_jerk = *std::max_element(all_jerks.begin(), all_jerks.end());

		return MAX_JERK * ((fabs(max_jerk) > MAX_JERK) ? 1 : 0);
	}

	double total_jerk_cost(vector<double> trajectory, double EXPECTED_JERK_IN_ONE_SEC)
	{
		vector<double> s_coeff;
		std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

		vector<double> d_coeff;
		std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

		double T = trajectory[12];

		vector<double> s_dot = differentiate(s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		vector<double> jerk = differentiate(s_d_dot);

		double total_jerk = 0.0;
		double dt = T / 100.0;
		for (size_t i = 1; i <= 100; i++)
		{
			double t = dt * i;
			total_jerk += fabs(evaluate(jerk, t)*dt);
		}
		double jerk_per_second = total_jerk / T;

		return TOTAL_JERK * (logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC));
	}
}