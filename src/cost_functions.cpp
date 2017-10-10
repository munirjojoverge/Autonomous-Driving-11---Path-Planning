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
#include "Planner_Constants.h"

using namespace std;
using namespace utils;
using namespace constants;

// priority levels for costs: Higher COST will give you a higher priority to AVOID

const double TIME_DIFF  = 10 ^ 3;
const double S_DIFF     = 10 ^ 5;
const double D_DIFF     = 10 ^ 6;
const double EFFICIENCY = 10 ^ 9;
const double MAX_JERK   = 10 ^ 2;
const double TOTAL_JERK = 10 ^ 3;
const double COLLISION  = 10 ^ 15;
const double DANGER     = 10 ^ 10;
const double MAX_ACCEL  = 10 ^ 2;
const double TOTAL_ACCEL = 10 ^ 3;
const double RIGHT_LANE_CHANGE = 10^1;

/*
from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np
*/
namespace cost_functions
{
	// COST FUNCTIONS

	double lane_change_cost(Trajectory trajectory, int dir)
	{
		/*
		Binary cost function which penalizes RIGHT lane changes Vs LEFT lane changes.
		Gives priority to Left lane changes Vs Right Lane changes (if both abailable).
		*/
		//return (trajectory.maneuver == ST_LANE_CHANGE && dir == RIGHT) ? RIGHT_LANE_CHANGE : 0.0;
		return 0.0;
	}

	double time_diff_cost(Trajectory trajectory, double T)
	{
		/*
		Penalizes trajectories that span a duration which is longer or
		shorter than the duration requested (T)f.
		*/

		// the last element on the trajectory should hold the last time value, i.e the total duration
		double t = trajectory.duration;
		return TIME_DIFF * (logistic(fabs(t - T) / T));
	}

	double diff_cost(vector<double> coeff, double duration, vector<double> goals, vector<double> sigma, double cost_weight)
	{
		/*
		Penalizes trajectories whose s coordinate(and derivatives)
		differ from the goal.
		*/
		double cost = 0.0;
		vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);
		//////////cout << "26 - Evaluating f and N derivatives Done. Size:" << evals.size() << endl;

		for (size_t i = 0; i < evals.size(); i++)
		{
			double diff = fabs(evals[i] - goals[i]);
			cost += logistic(diff / sigma[i]);
		}
		////////cout << "diff_coeff Cost Calculated " << endl;
		return cost_weight * cost;
	}

	double s_diff_cost(Trajectory trajectory, vector<double> goal_s, vector<double> sigma_s)
	{
		/*
		Penalizes trajectories whose s coordinate(and derivatives)
		differ from the goal.
		*/
		//////////cout << "27 - s_diff_coeff Cost Calculated " << endl;
		return diff_cost(trajectory.s_coeff, trajectory.duration, goal_s, sigma_s, S_DIFF);
	}

	double d_diff_cost(Trajectory trajectory, vector<double> goal_d, vector<double> sigma_d)
	{
		/*
		Penalizes trajectories whose d coordinate(and derivatives)
		differ from the goal.
		*/
		return diff_cost(trajectory.d_coeff, trajectory.duration, goal_d, sigma_d, D_DIFF);
	}

	double collision_cost(Trajectory trajectory, vector<Road_Vehicle> road_vehicles)
	{
		/*
		Binary cost function which penalizes collisions.
		*/	
		double nearest = closest_distance_to_any_vehicle(trajectory, road_vehicles,dt);
		return (nearest < 1 * VEHICLE_SIZE[0]) ? COLLISION : 0.0;		
	}

	double buffer_cost(Trajectory trajectory, vector<Road_Vehicle> road_vehicles)
	{
		/*
		Penalizes getting close to other vehicles.
		*/		
		double nearest = closest_distance_to_any_vehicle(trajectory, road_vehicles,dt);		
		return DANGER * (logistic(2 * MIN_FOLLOW_DISTANCE / nearest));
	}

	double stays_on_road_cost(Trajectory trajectory, Road road, double EgoWidth)
	{		
		////////cout << "stays_on_road_cost" << endl;
		double RoadWidth = road.num_lanes * road.lane_width;
		
		double time_step = dt; // max(trajectory.duration / 50.0, dt);
		double t = 0.0;
		bool OnRoad = true;
		
		while (OnRoad && t < trajectory.duration)
		{
			double d = evaluate(trajectory.d_coeff, t);
			OnRoad = ((d > EgoWidth) && (d < (RoadWidth - EgoWidth)));
			t += time_step;
		}	
		return DANGER * ((OnRoad) ? 0 : 1);

	}

	double exceeds_speed_limit_cost(Trajectory trajectory, Road road)
	{
		
		////////cout << "exceeds_speed_limit_cost" << endl;
		double time_step = dt; // max(trajectory.duration / 50.0, dt);
		double t = 0.0;
		vector<double> s_dot = differentiate(trajectory.s_coeff);
		bool UnderSpeeLimit = true;
		while (UnderSpeeLimit && t < trajectory.duration)
		{
			double Speed = evaluate(s_dot, t);
			UnderSpeeLimit = (Speed <= road.speed_limit);
			t += time_step;
		}
		return DANGER * ((UnderSpeeLimit) ? 0 : 1);
	}

	double efficiency_cost(Trajectory trajectory, vector<double> goal_s)
	{
		/*
		Rewards high average speeds.
		*/
		////////cout << "efficiency_cost" << endl;
		double distance_traveled = evaluate(trajectory.s_coeff, trajectory.duration) - evaluate(trajectory.s_coeff, 0.0);
		double avg_v = distance_traveled / trajectory.duration;

		return EFFICIENCY * ((logistic(2 * (goal_s[1] - avg_v) / avg_v)) + 0.5);
	}

	double total_accel_cost(Trajectory trajectory, double EXPECTED_ACC_IN_ONE_SEC)
	{
		
		////////cout << "total_accel_cost" << endl;
		vector<double> s_dot = differentiate(trajectory.s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		double total_acc = 0.0;
		double time_step = dt; // max(trajectory.duration / 50.0, dt);

		for (double t = 0.0; t <= trajectory.duration; t += time_step)
		{			
			total_acc += fabs(evaluate(s_d_dot, t)*time_step);
		}
		double acc_per_second = total_acc / trajectory.duration;

		return TOTAL_ACCEL * (logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC));
	}

	double max_accel_cost(Trajectory trajectory, double MAX_ACCEL)
	{
		////////cout << "max_accel_cost" << endl;
		vector<double> s_dot = differentiate(trajectory.s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		double time_step = dt; // max(trajectory.duration / 50.0, dt);
		vector<double> all_accs;
		for (double t = 0.0; t <= trajectory.duration; t += time_step)
		{
			all_accs.push_back(evaluate(s_d_dot, t));
		}
		double max_acc = *std::max_element(all_accs.begin(), all_accs.end());

		return MAX_ACCEL * ((fabs(max_acc) > MAX_ACCEL) ? 1 : 0);
	}

	double max_jerk_cost(Trajectory trajectory, double MAX_JERK)
	{
		
		////////cout << "max_jerk_cost" << endl;
		vector<double> s_dot = differentiate(trajectory.s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		vector<double> jerk = differentiate(s_d_dot);

		double time_step = dt; // max(trajectory.duration / 50.0, dt);
		vector<double> all_jerks;
		for (double t = 0.0; t <= trajectory.duration; t += time_step)
		{
			all_jerks.push_back(evaluate(jerk, t));
		}
		double max_jerk = *std::max_element(all_jerks.begin(), all_jerks.end());

		return MAX_JERK * ((fabs(max_jerk) > MAX_JERK) ? 1 : 0);
	}

	double total_jerk_cost(Trajectory trajectory, double EXPECTED_JERK_IN_ONE_SEC)
	{
		////////cout << "total_jerk_cost" << endl;
		vector<double> s_dot = differentiate(trajectory.s_coeff);
		vector<double> s_d_dot = differentiate(s_dot);

		vector<double> jerk = differentiate(s_d_dot);

		double total_jerk = 0.0;
		double time_step = dt; // max(trajectory.duration / 50.0, dt);
		for (double t = 0.0; t <= trajectory.duration; t += time_step)
		{
			total_jerk += fabs(evaluate(jerk, t)*time_step);
		}
		double jerk_per_second = total_jerk / trajectory.duration;

		return TOTAL_JERK * (logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC));
	}

	double closest_distance_to_any_vehicle(Trajectory trajectory, vector<Road_Vehicle> road_vehicles, double time_step)
	{
		/*
		Calculates the closest distance to any vehicle during a trajectory.
		*/
		double closest = 999999;
		for (auto v : road_vehicles)
		{
			double dist = closest_distance_to_vehicle(trajectory, v, dt);
			if (dist < closest)
				closest = dist;
		}
		return closest;
	}

	double closest_distance_to_vehicle(Trajectory trajectory, Road_Vehicle road_vehicle, double time_step)
	{
		/*
		Calculates the closest distance to a the provided vehicle during a trajectory.
		*/
		double closest = 999999;
		for (double t = 0.0; t <= trajectory.duration; t += dt)
		{
			double cur_s = evaluate(trajectory.s_coeff, t);
			double cur_d = evaluate(trajectory.d_coeff, t);
			Frenet_Position v_pos = road_vehicle.predict_frenet_at(t); // Vechicle position at time t
			double dist = sqrt(pow((cur_s - v_pos.s), 2) + pow((cur_d - v_pos.d), 2));
			if (dist < closest)
				closest = dist;
		}
		return closest;
	}
}

