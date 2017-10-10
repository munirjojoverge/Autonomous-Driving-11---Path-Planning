/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>     /* div, div_t */
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Dense"
#include "PathPlanner.h"
#include "cost_functions.h"
#include "spline.h"
#include "utils.h"
#include "Planner_Constants.h"



using namespace std;
using namespace Eigen;
using namespace utils;
using namespace constants;
using namespace cost_functions;

double deg2rad(double x) { return x * pi / 180; }
double rad2deg(double x) { return x * 180 / pi; }

/*
This file encopasses the 3 main elements in a Path Planner:
1) Prediction:
1) Get and Arrange the Localization and Sensor Fusion Data: This is a snaptshot of where we are and where other vehicles are.
2) Use a hybrid Approach (Model of the car  + Naive Bayes Classifier) to predict where the other vehicles would be in X secs

2) The Behavoir Planner:
1) Finate Maneuver Machine (FSM) that will decide what to do with the info from Prediction
2) If lane change is requested we will generate 2 trayectories (change left and right) and evaluate the COST of each one
3) We will select 1 of the 3 choices (change left, right or stay in lane)
4)
3) Trajectory smoother: Once we have selected what the trajectory, we will smooth it before send it to the simulator
*/


PathPlanner::~PathPlanner() {}


int PathPlanner::get_lane(double d)
{
	int lane = int(d / this->road.lane_width);
		
	if (lane < 0) 
		lane = 0;
	else if (lane > this->road.num_lanes - 1) 
		lane = this->road.num_lanes-1;

	return lane;
}

double PathPlanner::lane2d(int lane) // Gets the d coordinates of the center of the lane provided
{
	// lane = 0 is the most right one. Lane value should be withiin limits. The caller should check for proper lane values
	return ((lane + 0.5) * this->road.lane_width);		
}

double PathPlanner::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

int PathPlanner::ClosestWaypoint(double x, double y)
{
	vector<double> maps_x = road.map_waypoints_x;
	vector<double> maps_y = road.map_waypoints_y;

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int PathPlanner::NextWaypoint(double x, double y, double theta)
{

	vector<double> maps_x = road.map_waypoints_x;
	vector<double> maps_y = road.map_waypoints_y;

	int closestWaypoint = ClosestWaypoint(x, y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> PathPlanner::getFrenet(double x, double y, double theta)
{
	vector<double> maps_x = road.map_waypoints_x;
	vector<double> maps_y = road.map_waypoints_y;

	int next_wp = NextWaypoint(x, y, theta);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s,frenet_d };

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::getXY(double s, double d)
{
	vector<double> maps_x = road.map_waypoints_x;
	vector<double> maps_y = road.map_waypoints_y;
	vector<double> maps_s = road.map_waypoints_s;

	vector<double> maps_dx = road.map_waypoints_dx;
	vector<double> maps_dy = road.map_waypoints_dy;

	tk::spline xs;
	xs.set_points(maps_s, maps_x);

	tk::spline ys;
	ys.set_points(maps_s, maps_y);

	tk::spline dxs;
	dxs.set_points(maps_s, maps_dx);

	tk::spline dys;
	dys.set_points(maps_s, maps_dy);

	double x = xs(s) + d *dxs(s);
	double y = ys(s) + d *dys(s);
	return { x,y };
}

double PathPlanner::calculate_cost(Trajectory trajectory, vector<Road_Vehicle> road_vehicles, vector<double> goal_s, vector<double> goal_d, double goal_duration)
{	
	//cout << "Calculating Cost" << endl;
	double cost = 0.0;

	int dir = get_lane(trajectory.goal_d[0]) - get_lane(trajectory.start_d[0]);
	cost += lane_change_cost(trajectory, dir);
	cout << "lane_change_cost: " << cost << endl;

	cost += time_diff_cost(trajectory, goal_duration);
	cout << "time_diff_cost: " << cost << endl;
	
	cost += s_diff_cost(trajectory, goal_s, SIGMA_S);
	cout << "s_diff_cost: " << cost << endl;
	
	cost += d_diff_cost(trajectory, goal_d, SIGMA_D);
	cout << "d_diff_cost: " << cost << endl;
	
	cost += collision_cost(trajectory, road_vehicles);
	cout << "collision_cost: " << cost << endl;
	
	cost += buffer_cost(trajectory, road_vehicles);
	cout << "buffer_cost: " << cost << endl;
	
	cost += stays_on_road_cost(trajectory, this->road, VEHICLE_SIZE[0]);
	cout << "stays_on_road_cost: " << cost << endl;
	
	cost += exceeds_speed_limit_cost(trajectory, this->road);
	cout << "exceeds_speed_limit_cost: " << cost << endl;
	
	cost += efficiency_cost(trajectory, goal_s);
	cout << "efficiency_cost: " << cost << endl;
	
	cost += total_accel_cost(trajectory, EXPECTED_ACC_IN_ONE_SEC);
	cout << "total_accel_cost: " << cost << endl;
	
	cost += max_accel_cost(trajectory, MAX_ACCEL);
	cout << "max_accel_cost: " << cost << endl;
	
	cost += total_jerk_cost(trajectory, EXPECTED_JERK_IN_ONE_SEC);
	cout << "total_jerk_cost: " << cost << endl;
	
	cost += max_jerk_cost(trajectory, MAX_JERK);
	cout << "max_jerk_cost: " << cost << endl;
		
	return cost;
}

void PathPlanner::addTrajectory(Maneuver maneuver, vector<double> s_coeff, vector<double> d_coeff, vector<double> start_s, vector<double> start_d, vector<double> goal_s, vector<double> goal_d, double T, vector<Trajectory> &trajectories)
{
	Trajectory OneTrajectory;
	OneTrajectory.maneuver = maneuver;
	std::copy(s_coeff.begin(), s_coeff.end(), std::back_inserter(OneTrajectory.s_coeff));
	std::copy(d_coeff.begin(), d_coeff.end(), std::back_inserter(OneTrajectory.d_coeff));
	std::copy(start_s.begin(), start_s.end(), std::back_inserter(OneTrajectory.start_s));
	std::copy(start_d.begin(), start_d.end(), std::back_inserter(OneTrajectory.start_d));
	std::copy(goal_s.begin(), goal_s.end(), std::back_inserter(OneTrajectory.goal_s));
	std::copy(goal_d.begin(), goal_d.end(), std::back_inserter(OneTrajectory.goal_d));	
	OneTrajectory.duration = T;	

	// push the trajectory
	trajectories.push_back(OneTrajectory);
}

vector<Trajectory> PathPlanner::PTG(Maneuver maneuver, vector<double> start_s, vector<double> start_d, vector<double> goal_s, vector<double> goal_d, double T)
{
	/*
	POLYNOMIAL TRAJECTORY GENERATION

	Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS(global).

	arguments :
	start_s - [s, s_dot, s_ddot]

	start_d - [d, d_dot, d_ddot]

	goal_s - [s, s_dot, s_ddot]

	goal_d - [d, d_dot, d_ddot]

	T - the desired time at which we will be at the goal(relative to now as t = 0)

	return:
	vector of vectors:(best_s, best_d, best_t, goal_s, goal_d) where:
	best_s are the 6 coefficients representing s(t)
	best_d gives coefficients for d(t) and best_t gives duration associated w /
	this trajectory.
	*/

	vector<Trajectory> trajectories = {};
	

	/*
	* Let's generate the GOAL TRaj and also 4 alternative trajectories on each side of the time dimension.
	*/	
	// ////////cout << "Goal Time: " << T << " Time range: " << t << endl;
	// ////////cout << "Num samples: " << N_PERTURB_SAMPLES << endl;
	vector<double> s_coeff = JMT(start_s, goal_s, T);
	vector<double> d_coeff = JMT(start_d, goal_d, T);
	addTrajectory(maneuver, s_coeff, d_coeff, start_s, start_d, goal_s, goal_d, T, trajectories);
	/*
	double t = T - (4 * dt);
	while (t <= T + (4 * dt))
	{		
		vector<double> pert_goal_s;
		vector<double> pert_goal_d;
		for (size_t j = 0; j < N_PERTURB_SAMPLES; j++)
		{
			//// ////////cout << "Perturbation: " << j+1 << endl;
			perturb_goal(goal_s, goal_d, pert_goal_s, pert_goal_d);
			vector<double> s_coeff = JMT(start_s, pert_goal_s, t);
			vector<double> d_coeff = JMT(start_d, pert_goal_d, t);
			addTrajectory(maneuver,s_coeff, d_coeff, start_s, start_d, pert_goal_s, pert_goal_d, t, trajectories);
		}
		t += dt;
	}
	*/
	////////cout << "Trajectories returned PTG: " << trajectories.size() << endl;

	return trajectories;
	
}

void PathPlanner::perturb_goal(vector<double> goal_s, vector<double> goal_d, vector<double> &pert_goal_s, vector<double> &pert_goal_d)
{
	/*
	Returns a "perturbed" version of the goal.
	*/
	// declare a random engine to be used across multiple and various method calls
	random_device rd;
	static default_random_engine randomGen(rd());

	// Since goal_s and goal_d have the same size we can use 1 single loop to generate their perturbed versions together
	for (size_t i = 0; i < goal_s.size(); i++)
	{
		normal_distribution<double> dist_s(goal_s[i], SIGMA_S[i]);
		pert_goal_s.push_back(dist_s(randomGen));

		normal_distribution<double> dist_d(goal_d[i], SIGMA_D[i]);
		pert_goal_d.push_back(dist_d(randomGen));
	}
}

vector<double> PathPlanner::JMT(vector< double> start, vector <double> end, double T)
{
	/*
	Calculate the Jerk Minimizing Trajectory that connects the initial state
	to the final state in time T.

	INPUTS

	start - the vehicles start location given as a length three array
	corresponding to initial values of [s, s_dot, s_double_dot]

	end   - the desired end state for vehicle. Like "start" this is a
	length three array.

	T     - The duration, in seconds, over which this maneuver should occur.

	OUTPUT
	an array of length 6, each value corresponding to a coefficent in the polynomial
	s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

	EXAMPLE

	> JMT( [0, 10, 0], [10, 10, 0], 1)
	[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	*/

	//// ////////cout << "Starting JMT" << endl;

	double a0 = start[0];
	double a1 = start[1];
	double a2 = 0.5*start[2];

	// Let's solve the rest assuming a 3 equations with 3 unkowns in the form of Ax = B
	double T2 = T*T;
	double T3 = T2*T;
	double T4 = T3*T;
	double T5 = T4*T;
	MatrixXd A(3, 3);

	A << T3, T4, T5,
		3 * T2, 4 * T3, 5 * T4,
		6 * T, 12 * T2, 20 * T3;

	VectorXd B(3);
	B << end[0] - (a0 + a1*T + a2*T2),
		end[1] - (a1 + 2 * a2*T),
		end[2] - 2 * a2;

	// If T = 0, then A's determinant is 0 and is NOT invertible and we can confortably pass back same status + 0, 0, 0..	
	VectorXd C(3);
	if (!T == 0)	
		C = A.inverse()*B;	
	else
		C << 0, 0, 0;
		
	return { a0, a1, a2, C[0], C[1], C[2] };
}

vector<Road_Vehicle> PathPlanner::filter_vehicles_by_lane(vector<Road_Vehicle> road_vehicles, int lane)
{
	vector<Road_Vehicle> filtered;

	for (auto vehicle : road_vehicles)
	{
		if (get_lane(vehicle.d) == lane) // Here we don't need to predict the "d" coordenate because we are assuming straight motion
		{
			filtered.push_back(vehicle);
		}
	}
	return filtered;
}

Road_Vehicle PathPlanner::get_closest_vehicle(vector<Road_Vehicle> filtered_by_lane, double closest_to, bool only_infront)
{
	// Data of the "nearest" verctors hold vechicle's [ id, x, y, vx, vy, s, d]	
	Road_Vehicle nearest;
	if (filtered_by_lane.size() > 0)
	{
		int min_s = 99999; // Abs distance
		bool consider_this_vehicle;
		for (auto vehicle : filtered_by_lane)
		{			
			double distance = vehicle.s - closest_to;
			if (only_infront && distance < 0)
			{
				consider_this_vehicle = false;
			}
			else
			{
				distance = fabs(distance);
				consider_this_vehicle = true;
			}

			if (consider_this_vehicle && distance < min_s)
			{
				min_s = distance;
				nearest = vehicle;
			}
		}
	}
	// the caller of this fuction should check if it's nearest_infront or _behind are empty.
	//That would mean no-one is in-front/back (at detectable distance)
	return nearest;
}

Road_Vehicle PathPlanner::get_closest_vehicle_in_lane(int lane, double closest_to, bool only_infront, vector<Road_Vehicle> road_vehicles)
{	
	// FIND NEREST VEHICLES IN EGO'S SAME LANE. Vector's format [ id, x, y, vx, vy, s, d]	
	vector<Road_Vehicle> vehicles_on_ego_lane = filter_vehicles_by_lane(road_vehicles, lane);
	Road_Vehicle Vehicle = get_closest_vehicle(vehicles_on_ego_lane, closest_to, only_infront);	
	return Vehicle;
}
