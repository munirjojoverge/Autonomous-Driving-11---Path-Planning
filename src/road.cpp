/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/
#include "road.h"

/**
* Initializes Road
*/
Road::Road(double speed_limit, int num_lanes, double lane_width, string waypoints_file) {

	this->num_lanes = num_lanes;
	this->speed_limit = speed_limit;
	this->lane_width = lane_width;

	ifstream in_map_(waypoints_file.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;

		this->map_waypoints_x.push_back(x);
		this->map_waypoints_y.push_back(y);
		this->map_waypoints_s.push_back(s);
		this->map_waypoints_dx.push_back(d_x);
		this->map_waypoints_dy.push_back(d_y);
	}
}

Road::~Road() {}



