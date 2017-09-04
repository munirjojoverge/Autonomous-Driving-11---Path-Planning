/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/
#ifndef ROAD_H
#define ROAD_H

#include <sstream>
#include <fstream>

#include <vector>
#include <string>

using namespace std;

class Road {
public:
	/**
	* Constructor
	*/
	Road(double speed_limit, int num_lanes, double lane_width, string waypoints_file);

	/**
	* Destructor
	*/
	virtual ~Road();

	int num_lanes;

	double lane_width; // in meters

	double speed_limit;			

	// This is where I will load up the waypoint's  from the map file (x,y,s and d normalized normal vectors)
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	const double max_s = 6945.554; // The max s value before wrapping around the track back to 0	
		
};
#endif //ROAD_H