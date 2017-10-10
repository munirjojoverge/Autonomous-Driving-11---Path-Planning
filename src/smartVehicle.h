/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#ifndef SMARTVEHICE_H
#define SMARTVEHICLE_H


#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>    // std::min
#include "PathPlanner.h"



using namespace std;


class SmartVehicle {
public:
	/**
	* Constructor
	*/	
	SmartVehicle(PathPlanner &_pathPlanner) : planner(_pathPlanner) 
	{
		this->state = ST_CRUISE_CONTROL;		
		this->started_planning = false;
	};

	/**
	* Destructor
	*/
	virtual ~SmartVehicle();	
	
	double x;
	double y;
	double s;
	double d;	
	double yaw;
	double speed;
	int lane;
	int target_lane;
	double prev_target_speed;
	maneuver_params follow_maneuver_params;
	maneuver_params cruise_maneuver_params;
	
	//vector<double> prev_path_s;
	//vector<double> prev_path_d;
	
	PathPlanner &planner;
	bool follow_maneuver_added;
	bool started_planning;
	Ego_status Ego_start_status; // This is the status at the end of the last traj sent to SIM which is the START of the new traj planning

	/* To allow for super states, it will be better to use an Int or a LongInt where each bit represents a state.
	That way you can have more than one 1 active state: 000100100. But for now let's keep it
	very simple and work with a flat FST and use a simple enumeration (look above)
	*/
	Maneuver state;
	
	void update(vector<double> EgoData, vector< vector<double> > sensor_fusion, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	vector<Road_Vehicle> generate_road_vehicles(vector<vector<double>> sensor_fusion);

	void calculate_new_starting_point_4_Traj_Generation(int not_processed_traj_size);

	void update_state(vector<Road_Vehicle> road_vehicles, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	void update_possible_maneuvers(int target_lane, Road_Vehicle VehicleA, Road_Vehicle VehicleB,  Road_Vehicle VehicleB2, vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers);

	vector < tuple<Maneuver, maneuver_params > > generate_possible_maneuvers(vector<Road_Vehicle> road_vehicles);

	maneuver_params generate_follow_params(Road_Vehicle VehicleA);

	maneuver_params generate_emergency_params(Road_Vehicle VehicleA);

	maneuver_params generate_escape_params(Road_Vehicle Vehicle_A);

	maneuver_params generate_cruise_control_params(double target_speed);

	maneuver_params generate_change_lane_params(double target_lane, double target_speed);
	
	Trajectory generate_best_trajectory(vector<Road_Vehicle> road_vehicles, vector < tuple<Maneuver, maneuver_params > > possible_maneuvers);

	void smooth_trajectory_frenet2(Trajectory trajectory, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	double get_maneuver_time(Ego_status Ego, double desired_speed);

	double get_maneuver_accel(Ego_status Ego, double desired_speed, double maneuver_time);
		
	void add_lane_change_infront_B_maneuver(Road_Vehicle VechicleB, Road_Vehicle VechicleB2, int target_lane, vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers);

	void add_lane_change_behind_B_maneuver(Road_Vehicle VehicleB, int target_lane, vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers);

	void add_follow_maneuver(vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers);
};

#endif




