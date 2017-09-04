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
#include "PathPlanner.h"
#include "cost_functions.h"
#include "spline.h"
#include "utils.h"



using namespace std;
using namespace Eigen;
using namespace utils;
using namespace cost_functions;


/*
This file encopasses the 3 main elements in a Path Planner:
1) Prediction:
1) Get and Arrange the Localization and Sensor Fusion Data: This is a snaptshot of where we are and where other vehicles are.
2) Use a hybrid Approach (Model of the car  + Naive Bayes Classifier) to predict where the other vehicles would be in X secs

2) The Behavoir Planner:
1) Finate State Machine (FSM) that will decide what to do with the info from Prediction
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

int PathPlanner::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

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

int PathPlanner::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

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
vector<double> PathPlanner::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

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
vector<double> PathPlanner::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

	double perp_heading = heading - pi / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return { x,y };
}

vector<vector<double>> PathPlanner::filter_sensor_fusion_by_lane(vector<vector<double>> sensor_fusion, int lane)
{	
	vector<vector<double>> filtered;
	vector<vector<double>>::iterator it = sensor_fusion.begin();

	while (it != sensor_fusion.end())
	{		
		vector<double> vehicleData = *it; //  [ id, x, y, vx, vy, s, d]

		if (this->get_lane(vehicleData[6]) == lane)
		{
			filtered.push_back(vehicleData);
		}
		it++;
	}
	return filtered;
}

void PathPlanner::nearest(vector<vector<double>> filtered_by_lane, double Ego_s, vector<double> &nearest_infront, vector<double> &nearest_behind)
{
	// Data of the "nearest" verctors hold vechicle's [ id, x, y, vx, vy, s, d]	

	if (filtered_by_lane.size() > 0)
	{
		vector<vector<double>>::iterator it = filtered_by_lane.begin();
		int min_s_f = 1000; // Front
		int min_s_b = -1000; // Back
		while (it != filtered_by_lane.end())
		{
			vector<double> vehicleData = *it;
			double distance = vehicleData[5] - Ego_s;			
			if ((distance < 0.0) && (distance > min_s_b)) // Behind Ego Car
			{
				min_s_b = distance;
				nearest_behind = vehicleData;
			}
			if ((distance > 0.0) && (distance < min_s_f)) // In Front Ego Car
			{
				min_s_f = distance;
				nearest_infront = vehicleData;
			}
			it++;
		}
	}	
	// the caller of this fuction should check if it's nearest_infront or _behind are empty.
	//That would mean no-one is in-front/back (at detectable distance)
}

void PathPlanner::nearestAbs(vector<vector<double>> filtered_by_lane, double Ego_s, vector<double> &nearest)
{
	// Data of the "nearest" verctors hold vechicle's [ id, x, y, vx, vy, s, d]	

	if (filtered_by_lane.size() > 0)
	{
		vector<vector<double>>::iterator it = filtered_by_lane.begin();
		int min_s = 1000; // Abs distance
		while (it != filtered_by_lane.end())
		{
			vector<double> vehicleData = *it;
			double distance = fabs(vehicleData[5] - Ego_s);
			if (distance > min_s) 
			{
				min_s = distance;
				nearest = vehicleData;
			}			
			it++;
		}
	}
	// the caller of this fuction should check if it's nearest_infront or _behind are empty.
	//That would mean no-one is in-front/back (at detectable distance)
}

double PathPlanner::calculate_cost(vector<double> trajectory, vector<vector<double>> sensor_fusion, vector<double> goal_s, vector<double> goal_d, double T, double EgoWidth)
{	
	//// cout << "21 - Calculating Cost" << endl;
	double cost = 0.0;
	cost += time_diff_cost(trajectory, T);
	// cout << "22 - time_diff_cost done" << endl;
	cost += s_diff_cost(trajectory, goal_s, this->SIGMA_S);
	// cout << "23 - s_diff_cost done" << endl;
	cost += d_diff_cost(trajectory, goal_d, this->SIGMA_D);
	// cout << "24 - d_diff_cost done" << endl;
	cost += collision_cost(trajectory, sensor_fusion);
	// cout << "23 - collision_cost done" << endl;
	cost += buffer_cost(trajectory, sensor_fusion);
	// cout << "24 - buffer_cost done" << endl;
	cost += stays_on_road_cost(trajectory, this->road, EgoWidth);
	// cout << "24 - stays_on_road_cost done" << endl;
	cost += exceeds_speed_limit_cost(trajectory, this->road);
	// cout << "24 - exceeds_speed_limit_cost done" << endl;
	cost += efficiency_cost(trajectory, goal_s);
	// cout << "24 - efficiency_cost done" << endl;
	cost += total_accel_cost(trajectory, EXPECTED_ACC_IN_ONE_SEC);
	// cout << "24 - total_accel_cost done" << endl;
	cost += max_accel_cost(trajectory, MAX_ACCEL);
	// cout << "24 - max_accel_cost done" << endl;
	cost += total_jerk_cost(trajectory, EXPECTED_JERK_IN_ONE_SEC);
	// cout << "24 - total_jerk_cost done" << endl;
	cost += max_jerk_cost(trajectory, MAX_JERK);	
	// cout << "24 - max_jerk_cost done" << endl;
	//if (DEBUG)
	//// cout << "Cost:" << cost << endl;

	return cost;
}

void PathPlanner::addTrajectory(vector<double> s_coeff, vector<double> d_coeff, double T, vector<double> goal_s, vector<double> goal_d, vector<vector<double>> &trajectories)
{
	vector<double> OneTrajectory;
	auto it = OneTrajectory.end();
	OneTrajectory.insert(it, s_coeff.begin(), s_coeff.end());
	it = OneTrajectory.end();
	OneTrajectory.insert(it, d_coeff.begin(), d_coeff.end());
	it = OneTrajectory.end();
	OneTrajectory.insert(it, T);
	it = OneTrajectory.end();
	OneTrajectory.insert(it, goal_s.begin(), goal_s.end());
	it = OneTrajectory.end();
	OneTrajectory.insert(it, goal_d.begin(), goal_d.end());
	// push the trajectory
	trajectories.push_back(OneTrajectory);
}

vector<vector<double>> PathPlanner::PTG(vector<double> start_s, vector<double> start_d, vector<double> goal_s, vector<double> goal_d, double T)
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

	vector<vector<double>> trajectories = {};	
	

	/*
	* Let's generate the GOAL TRaj and also 4 alternative trajectories on each side of the time dimension.
	*/	
	double t = T - 4 * this->dt;
	// cout << "Goal Time: " << T << " Time range: " << t << endl;
	// cout << "Num samples: " << N_SAMPLES << endl;
	vector<double> s_coeff = JMT(start_s, goal_s, T);
	vector<double> d_coeff = JMT(start_d, goal_d, T);
	addTrajectory(s_coeff, d_coeff, t, goal_s, goal_d, trajectories);

	while (t <= T + 4 * this->dt)
	{		
		vector<double> pert_goal_s;
		vector<double> pert_goal_d;
		for (size_t j = 0; j < N_SAMPLES; j++)
		{
			//// cout << "Perturbation: " << j+1 << endl;
			perturb_goal(goal_s, goal_d, pert_goal_s, pert_goal_d);
			vector<double> s_coeff = JMT(start_s, pert_goal_s, t);
			vector<double> d_coeff = JMT(start_d, pert_goal_d, t);
			addTrajectory(s_coeff, d_coeff, t, pert_goal_s, pert_goal_d, trajectories);
		}
		t += this->dt;
	}
	// cout << "Trajectories returned PTG" << endl;

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

	//// cout << "Starting JMT" << endl;

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

	MatrixXd B = MatrixXd(3, 1);
	B << end[0] - (a0 + a1*T + a2*T2),
		 end[1] - (a1 + 2 * a2*T),
		 end[2] - 2 * a2;

	// Let's assume A is invertible..I'll have to check for the requirements on the real code.
	MatrixXd Ai = A.inverse();
	//// cout << "stop 19 E" << endl;

	MatrixXd C = Ai*B;

	vector <double> result = { a0, a1, a2 };
	for (int i = 0; i < C.size(); i++)
	{
		//// cout << "stop 19 F - For loop on JMT" << endl;
		result.push_back(C.data()[i]);
	}
	//// cout << "Finished JMT" << endl;
	return result;

}


/*
void PathPlanner::smoothTrajectory(vector<double>EgoData, vector<double> goal_s, vector<double> goal_d, vector<double> trajectory, vector<double> prev_path_x, vector<double> prev_path_y, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	// ********* TRAJECTORY SMOOTHER ************
	vector<double> pts_x;
	vector<double> pts_y;

	double pt1_x;
	double pt1_y;
	double pt2_x;
	double pt2_y;

	double ref_yaw = deg2rad(EgoData[2]);

	int prev_path_size = prev_path_x.size();
	if (prev_path_size < 2)
	{
		pt1_x = EgoData[0];
		pt1_y = EgoData[1];

		pt2_x = pt1_x - cos(ref_yaw);
		pt2_y = pt1_y - sin(ref_yaw);

	}
	else
	{
		pt1_x = prev_path_x[prev_path_size - 1];
		pt1_y = prev_path_y[prev_path_size - 1];

		pt2_x = prev_path_x[prev_path_size - 2];
		pt2_y = prev_path_y[prev_path_size - 2];
	}

	pts_x.push_back(pt2_x);
	pts_x.push_back(pt1_x);

	pts_y.push_back(pt2_y);
	pts_y.push_back(pt1_y);


	vector<double> s_coeff;
	std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

	vector<double> d_coeff;
	std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

	double t = trajectory[12];

	double s = evaluate(s_coeff, t);
	double d = evaluate(d_coeff, t);

	vector<double> next_wp = getXY(s, d, this->road.map_waypoints_s, this->road.map_waypoints_x, this->road.map_waypoints_y);
	pts_x.push_back(next_wp[0]);
	pts_y.push_back(next_wp[1]);
	

	tk::spline aSpline;

	//// cout << "pts_x size: " << pts_x.size() << endl;
	//// cout << "pts_y size: " << pts_y.size() << endl;

	aSpline.set_points(pts_x, pts_y);

	// cout << "prev_path_size: " << prev_path_size << endl;
	for (int i = 0; i < prev_path_size; i++)
	{
		next_x_vals.push_back(prev_path_x[i]);
		next_y_vals.push_back(prev_path_y[i]);
	}


	double MinTimeOfReaction;
	MinTimeOfReaction = max(EgoData[3] / this->MAX_ACCEL, this->MAX_ACCEL);
	double horizon = max(EgoData[3] *MinTimeOfReaction - 0.5*this->MAX_ACCEL*(MinTimeOfReaction*MinTimeOfReaction), 30.0);

	//// cout << "MinTimeOfReaction: " << MinTimeOfReaction << endl;
	//// cout << "horizon: " << horizon << endl;

	double target_x = horizon;
	double target_y = aSpline(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);
	double N = target_dist / (goal_s[1] * this->dt);

	//// cout << "Num of points needed in path: " << N << endl;
	//// cout << "target_dist: " << target_dist << endl;

	double inc = target_x / N;


	double x_val = 0;
	double y_val = 0;
	for (int i = 1; i < this->NUM_PATH_STEPS - prev_path_size; i++)
	{
		x_val += inc;
		y_val = aSpline(x_val);

		next_x_vals.push_back((x_val * cos(ref_yaw) - y_val*sin(ref_yaw)) + pt1_x);
		next_y_vals.push_back((x_val * sin(ref_yaw) + y_val*cos(ref_yaw)) + pt1_y);
	}
}

*/
