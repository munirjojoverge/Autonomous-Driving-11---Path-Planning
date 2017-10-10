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
#include "utils.h"
#include "Road_Vehicle.h"
#include "Planner_Constants.h"
#include "structs.h"

using namespace std;
using namespace utils;

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


class PathPlanner {
public:
	PathPlanner(Road &_road) : road(_road) { 
		Vs_desired = road.speed_limit - mph2ms(2.5);
		following_count = 0;
	};
	virtual ~PathPlanner();	

	double calculate_cost(Trajectory trajectory, vector<Road_Vehicle> road_vehicles, vector<double> goal_s, vector<double> goal_d, double goal_duration);

	Road &road;
	double Vs_desired; // Desired Speed in Frenet Coordinate S. This is usually the Road Speed Limit - 0.5m/s
	
	int following_count;	
		
	double distance(double x1, double y1, double x2, double y2);

	int ClosestWaypoint(double x, double y);

	int NextWaypoint(double x, double y, double theta);

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d);

	// Filters the Sensor Fusion data with Cars only on a specific lane
	vector<Road_Vehicle> filter_vehicles_by_lane(vector<Road_Vehicle> sensor_fusion, int lane);	

	Road_Vehicle get_closest_vehicle(vector<Road_Vehicle> filtered_by_lane, double closest_to, bool only_infront = false);

	Road_Vehicle get_closest_vehicle_in_lane(int lane, double closest_to, bool only_infront, vector<Road_Vehicle> road_vehicles);
	
	int get_lane(double d);

	vector<Trajectory> PTG(Maneuver maneuver,vector<double> start_s, vector<double> start_d, vector<double> goal_s, vector<double> goal_d, double T);

	void perturb_goal(vector<double> goal_s, vector<double> goal_d, vector<double> &pert_goal_s, vector<double> &pert_goal_d);

	vector<double> JMT(vector< double> start, vector <double> end, double T);

	void addTrajectory(Maneuver maneuver,vector<double> s_coeff, vector<double> d_coeff, vector<double> start_s, vector<double> start_d, vector<double> goal_s, vector<double> goal_d, double T, vector<Trajectory> &trajectories);

	double lane2d(int lane);

};

#endif //PATH_PLANNER_H
