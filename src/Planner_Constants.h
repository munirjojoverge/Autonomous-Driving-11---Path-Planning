/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#ifndef PLANNER_CONSTANTS_H
#define PLANNER_CONSTANTS_H

#include <vector>
#include "utils.h"
using namespace std;
using namespace utils;

namespace constants
{
	const vector<double> VEHICLE_SIZE = { 5, 2 }; // Average Vehicle {Length, Width} in meters (https://en.wikipedia.org/wiki/Family_car)
	const double dt = 0.02;
	const double MAX_ACCEL = 7.1; // m/s2
	const double MAX_DECEL = 8.0; // m/s2

	/*
	According to American Association of State Highway and Transportation Officials
	(AASHTO; 2001) standards values of Lateral Jerk ranging from 0,3 to 0,9 m/s3 have been
	used for highways. Some literature values of lateral Jerk are as follows:
	For highways; Zy = 0,6 m/s3
	(for residential areas), Zy = 0,3 m/s3
	(for rural highways)
	(Schofield; 2001), Zy = 0,6 m/s3 (Umar; Yayla; 1997), Zy = 0,6 m/s3 (Uren; Price; 1985),
	Zy = 0,5 m/s3
	(Manns; 1985).
	For Railways; Zy = 0,5 m/s3 (Megyeri; 1993), Zy = 0,2 m/s3 (Esveld; 1989), Zy = 0,4 m/s3
	(Förstberg; 2000), Zy = 0,5 m/s3 (Evren; 2002).
	*/

	/*
	Sample Chapter from "Traffic Flow Dynamics" written by M.Treiber and A.Kesting
	More information: http://www.traffic-flow-dynamics.org
	By courtesy of Springer publisher, http://www.springer.com
	 - CAR FOLLOWING MODELS BY RICHARD W. ROTHERY6

	" Typical values of a “comfortable” jerk are |J| ≤ 1.5m/s3" ( I beleive this is a typo and it's actually 1.5g/s
	*/
	const double Gs = 9.8;
	const double MAX_LATERAL_JERK = 0.9; // m/s3	
	const double MAX_JERK = 1.5;//*Gs; // m/s3	 

	const double MIN_TTC = 1.5 * 3; // since I'm going to add trajectories due to SIM issues, I will NOT be able to react faster than the min trajectory time which is going from 0 to Max speed in Cruise Control.
	const double MIN_TIV = 0.5 * 3;
	const double MIN_FOLLOW_DISTANCE = 1 * VEHICLE_SIZE[0]; // is the minimum distance in case of congestion (v = 0). 	
	const int N_PERTURB_SAMPLES = 1; // These are the number of perturbed Goals Per time step (4 ahead and 4 passed the time of the goal) that we want to generate a "Trajectory" to make sure we explore if we really can achieve it
	const vector<double> SIGMA_S = { 4, 1.0, 2.0 };// Standard devaition parameters for s, s_dot, s_double_dot to generate appropriate perturbed goals
	const vector<double> SIGMA_D = { 0.5, 1.0, 0.5 }; // Standard devaition parameters for d, d_dot, d_double_dot to generate appropriate perturbed goals
	const double SIGMA_T = 0.5; // Standard deviation for time (as in the time taken to finish the maneuver		
	const double EXPECTED_JERK_IN_ONE_SEC = 2; // m/s2. This would be the filtered Jerk over one sec
	const double EXPECTED_ACC_IN_ONE_SEC = 1; // m / s
	const double LANE_TOLERANCE = 0.1;
	const double DESIRED_TIME_GAP_FOLLOWING = 0.75; // pre-defined desired time - gap settings, usually ranging between 0.8 s and 2.2 s[ISO 15622,2010].	
	const double MIN_MANEUVER_TIME = 0.3;	
	//const double MAX_MANEUVER_TIME = 2.8;
	const double MAX_ACCUM_TRAJECTORY_TIME = 0.6;
	const int MAX_NUM_POINTS = int(MAX_ACCUM_TRAJECTORY_TIME / dt);
	const double LANE_CHANGE_SLOW_DOWN_FACTOR = 1;

	const double RELATIVE_VELOCITY_THRESHOLD = 12.0; //0.853// Relative Velocity Threshold m/s (Frenet Coordinate S - longitudinal Speed)	
	// FROM http://ncts.upd.edu.ph/tssp/wp-content/uploads/2016/08/Dimayacyac-Palmiano.pdf

	/*
	Intelligent driver model Parameters
	*/
	const double GAMMA = 2.75;
	const double DELTA = 4.0;

	const int LEFT  = -1;
	const int RIGHT =  1;
	const double pi = 3.14159265;
}
#endif
