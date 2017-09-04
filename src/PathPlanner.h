/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <random>
#include <cmath>
#include "road.h"

using namespace std;

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

class PathPlanner {
public:
	PathPlanner(Road &_road) : road(_road) { Vs_desired = road.speed_limit; Vt = 0.75 * road.speed_limit; };
	virtual ~PathPlanner();	

	double calculate_cost(vector<double> trajectory, vector<vector<double>> sensor_fusion, vector<double> goal_s, vector<double> goal_d, double T, double EgoWidth);

	Road &road;	
	double Vs_desired; // Desired Speed in Frenet Coordinate S. This is usually the Road Speed Limit - 0.5m/s
	double Vt; // Relative Velocity Threshold (Frenet Coordinate S - longitudinal Speed)
	double d_max;// in meters. Maximum following distance between Ego car and the one infront. It will be dinamically calc based on our speed.
	
	const double time_offset_from_prev_path = 5.0; // Number of Secs from "prev_path" that we will use before we build the new traj.
	//Since dt = 0.02, if we want to use, for example, 0.5 sec from prev_path that means that we will take the first 0.5 / 0.02 = 25 points from prev_path
	vector<double> start_s, start_d;
	vector<double> new_path_x, new_path_y;
	double ref_theta;

	double distance(double x1, double y1, double x2, double y2);

	int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

	int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

	// Filters the Sensor Fusion data with Cars only on a specific lane
	vector<vector<double>> filter_sensor_fusion_by_lane(vector<vector<double>> sensor_fusion, int lane);

	void nearest(vector<vector<double>> filtered_by_lane, double Ego_s, vector<double> &nearest_infront, vector<double> &nearest_behind);	
	void nearestAbs(vector<vector<double>> filtered_by_lane, double Ego_s, vector<double> &nearest);

	int get_lane(double d);

	vector<vector<double>> PTG(vector<double> start_s, vector<double> start_d, vector<double> goal_s, vector<double> goal_d, double T);
	void perturb_goal(vector<double> goal_s, vector<double> goal_d, vector<double> &pert_goal_s, vector<double> &pert_goal_d);
	vector<double> JMT(vector< double> start, vector <double> end, double T);
	void addTrajectory(vector<double> s_coeff, vector<double> d_coeff, double T, vector<double> goal_s, vector<double> goal_d, vector<vector<double>> &trajectories);
	double lane2d(int lane);

	const double dt = 0.02;
	const double MAX_ACCEL = 10; // m/s2
	const double MAX_JERK = 50; // m/s3
	const int NUM_PATH_STEPS = 50;
	const double BUFFER_DISTANCE = 5; // min buffer distance (in meters) we want to keep with cars around (primerly in front, but also laterarly) 
	
	const int N_SAMPLES = 1; // These are the number of perturbed Goals Per time step (4 ahead and 4 passed the time of the goal) that we want to generate a "Trajectory" to make sure we explore if we really can achieve it
	const vector<double> SIGMA_S = { 10.0, 4.0, 2.0 };// Standard devaition parameters for s, s_dot, s_double_dot to generate appropriate perturbed goals
	const vector<double> SIGMA_D = { 1.0, 1.0, 1.0 }; // Standard devaition parameters for d, d_dot, d_double_dot to generate appropriate perturbed goals
	const double SIGMA_T = 2.0; // Standard deviation for time (as in the time taken to finish the maneuver		
	const double EXPECTED_JERK_IN_ONE_SEC = MAX_JERK / 5; // m/s2. This would be the filtered Jerk over one sec
	const double EXPECTED_ACC_IN_ONE_SEC = MAX_ACCEL / 10; // m / s

};

#endif //PATH_PLANNER_H
