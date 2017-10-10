/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>

using namespace std;

enum Maneuver // A.K.A State
{
	ST_CRUISE_CONTROL, 	// Cruise Control - Maintain a constant speed = SpeedLimit-buffer in the same lane
	ST_FOLLOW,			// SmartVehicle Following - Disntance with SmartVehicle in front is < d_min and we need to slow down & speed Up (calc accel) behind it until we can Change lane or move at the desired speed 
	ST_LANE_CHANGE,
};

struct Trajectory {
	Maneuver maneuver;
	vector<double> s_coeff;
	vector<double> d_coeff;
	vector<double> start_s;
	vector<double> start_d;
	vector<double> goal_s;
	vector<double> goal_d;
	double duration; // maneuver duration	
};

struct maneuver_params {
	int dir; // directiion: +1(left), 0 (stay in lane), -1 (right)
	double target_s; // target final s coordinate. Ego_s + maneuver len
	double target_speed; // target end_speed of the maneuver
	double duration; // target duration 
};

struct Ego_status {
	vector<double> s;
	vector<double> d;
	int lane;
	double prediction_time; // how far ahead in the future is EGO at the final point of the path we sent to the SIM
};

#endif
