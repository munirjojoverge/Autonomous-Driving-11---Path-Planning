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


enum State
{	
	ST_CRUISE_CONTROL, 	// Cruise Control - Maintain a constant speed = SpeedLimit-buffer in the same lane
	ST_WAIT,			// SmartVehicle Following - Disntance with SmartVehicle in front is < d_min and we need to slow down & speed Up (calc accel) behind it until we can Change lane or move at the desired speed 
	ST_LANE_CHANGE,	
};

class SmartVehicle {
public:
	/**
	* Constructor
	*/
	SmartVehicle(); // Default Costructor
	SmartVehicle(PathPlanner &_pathPlanner) : planner(_pathPlanner) {
		this->state = ST_CRUISE_CONTROL;		
	};

	/**
	* Destructor
	*/
	virtual ~SmartVehicle();
	vector<double> vechileSize = { 4.5, 2 }; // Average SmartVehicle {Length, Width} in meters (https://en.wikipedia.org/wiki/Family_car)
	
	double x;
	double y;
	double s;
	double d;
	double vs;
	double vd;
	double as;
	double ad;
	double yaw;
	double speed;
	int lane;
	int target_lane;
	double target_speed;
	
	//vector<double> prev_path_s;
	//vector<double> prev_path_d;
	
	PathPlanner &planner;
	
	

	/* To allow for super states, it will be better to use an Int or a LongInt where each bit represents a state.
	That way you can have more than one 1 active state: 000100100. But for now let's keep it
	very simple and work with a flat FST and use a simple enumeration (look above)
	*/
	State state;

	//void update(vector<double> EgoData, vector< vector<double> > sensor_fusion, vector<double> prev_path_x, vector<double> prev_path_y, vector<double> &next_x_vals, vector<double> &next_y_vals);
	void update(vector<double> EgoData, vector< vector<double> > sensor_fusion, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	void update_state(vector<vector<double>> sensor_fusion, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	void calculate_new_starting_point(vector<double> prev_x_vals, vector<double> prev_y_vals);

	void update_possible_maneuvers(vector< vector<double> > sensor_fusion, int dir, double Va, vector < tuple<State, int > > &possible_maneuvers);

	void generate_possible_maneuvers(vector<vector<double>> sensor_fusion, vector<double> VehicleA, vector < tuple<State, int > > &possible_maneuvers);

	void get_vehicle_infront_data(vector<vector<double>> sensor_fusion, vector<double> &VehicleA);

	void generate_best_trajectory(vector<vector<double>> sensor_fusion, vector<double> VehicleA, vector < tuple<State, int > > possible_maneuvers, vector<double> &best_trajectory, State &Ego_new_state);

	void smooth_trajectory(vector<double> trajectory, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	void cruise_control(vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);

	void cruise_control2(vector<double> trajectory, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals);
};

#endif

