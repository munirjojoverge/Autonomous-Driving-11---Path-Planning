/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#include <tuple>
#include <stdexcept>

#include "smartVehicle.h"
#include "utils.h"
#include "spline.h"

using namespace std;
using namespace utils;
using namespace constants;

/**
* Initializes SmartVehicle
*/

SmartVehicle::~SmartVehicle() {}
/**/
void SmartVehicle::update(vector<double> EgoData, vector<vector<double>> sensor_fusion, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	////cout << "SmartVehicle begining Updated" << endl;

	// 1) Unpack and update the ego vehicle data
	this->x = EgoData[0];
	this->y = EgoData[1];
	this->s = EgoData[2];
	this->d = EgoData[3];
	this->yaw = EgoData[4];
	this->speed = mph2ms(EgoData[5]);
	
	this->lane = planner.get_lane(this->d);

	if (this->s > this->planner.road.TRACK_LENGTH)
	{
		cout << "***** New round" << endl;
		this->s = 0;
	}
	//////cout << "****************************************************************" << endl;
	//////cout << "***************************NEW CYCLE****************************" << endl;
	//////cout << "****************************************************************" << endl;
	//////cout << "0 - Ego s: " << this->s << " Ego d: " << this->d << " Road w = " << this->planner.road.lane_width << endl;
	//////cout << "0 - Ego x: " << this->x << " Ego y: " << this->y << endl;
	//////cout << "0 - Ego Speed: " << this->speed << endl;
	//////cout << "0 - Ego Speed: " << this->speed << " Yaw: " << this->yaw << endl;
	//////cout << "****************************************************************" << endl;
	
	calculate_new_starting_point_4_Traj_Generation(prev_x_vals.size()); // This will always be the END connditions of the previous traj (s, s_dot, s_ddot, d_....)

	// we will convert all sensor fusion data into a convenient format
	vector<Road_Vehicle> road_vehicles = generate_road_vehicles(sensor_fusion);
	
	// Now we Update the "Maneuver/Maneuver" of Ego based on it's data and the sensor_fusion data
	update_state(road_vehicles, prev_x_vals, prev_y_vals, next_x_vals, next_y_vals);
		
}

vector<Road_Vehicle> SmartVehicle::generate_road_vehicles(vector<vector<double>> sensor_fusion)
{
	vector<Road_Vehicle> road_vehicles;
	for (auto vehicle_data : sensor_fusion)
	{
		int id    = int(vehicle_data[0]);
		double x  = vehicle_data[1];
		double y  = vehicle_data[2];
		double vx = vehicle_data[3];
		double vy = vehicle_data[4];
		double s  = vehicle_data[5];
		double d  = vehicle_data[6];
		Road_Vehicle vehicle(id, x, y, vx, vy, s, d);
		vehicle.s = vehicle.predict_frenet_at(this->Ego_start_status.prediction_time).s;
		road_vehicles.push_back(vehicle);
	}
	return road_vehicles;
}

void SmartVehicle::calculate_new_starting_point_4_Traj_Generation(int not_processed_traj_size)
{	
	//////cout << "****************************************************************" << endl;
	//////cout << "******************calculate_new_starting_point******************" << endl;

	if (started_planning) // that means that we sent a Traj before and we can use the "final_s and final_d" values to start our new trajectory
	{				
		Ego_start_status.prediction_time = not_processed_traj_size*dt;
		////cout << "not_processed_traj_size: " << not_processed_traj_size << " Prediction time: " << Ego_start_status.prediction_time << endl;
		
	}
	else
	{
		////cout << "Not enough points to re-use - Frenet " << endl;
		Ego_start_status.s = { this->s, 0.0, 0.0 };
		Ego_start_status.d = { this->d, 0.0, 0.0 };
		Ego_start_status.lane = planner.get_lane(Ego_start_status.d[0]);

		Ego_start_status.prediction_time = 0;
		started_planning = true;
	}	
	
}

void SmartVehicle::update_state(vector<Road_Vehicle> road_vehicles, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	/*
	Updates the "state/Maneuver" of the vehicle by assigning one of the
	following values to state':
	ST_CRUISE_CONTROL  // Cruise Control - Maintain a constant speed = SpeedLimit-buffer in the same lane
	ST_FOLLOW,		   // SmartVehicle Following - Disntance with SmartVehicle in front is < d_min and we need to slow down & speed Up (calc accel) behind it until we can Change lane or move at the desired speed
	ST_LANE_CHANGE,    // Change lane

	INPUTS
	- sensor_fusion
	The sensor_fusion variable contains all the information about the cars on the right-hand side of the road.
	The data format for each car is: [ id, x, y, vx, vy, s, d].
	-The id is a unique identifier for that car.
	-The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map.
	-Finally s and d are the Frenet coordinates for that car.

	The vx, vy values can be useful for predicting where the cars will be in the future.
	For instance, if you were to assume that the tracked car kept moving along the road,
	then its future predicted Frenet s value will be its current s value plus its (transformed) total velocity (m/s) multiplied by
	the time elapsed into the future (s).

	- Simulator "prev_path" data (points of the previos path sent to the sim that could not be achieved)
	{ {x0, y0}, ....{xn, yn} }
	where:
	{x0,y0} is the first point that the Sim did NOT got the Ego car on. (in theory).
	In reality, the Sim kept running and moving the car through our path while
	1) the telemetry pkg was getting prep to be sent (Sim running a thread to keep moving the car)
	2) the pkg was sent (comm letancy)
	3) the pkg was received
	4) our SmartVehicle class is processing it and generating a new path
	During all this time, the Sim has moved the car, (probably 1 or 2 steps - dt) and that's the reason why using the very first values
	of prev_path is not actually correct. In fact if you do so, your car will tray to move backwars for a few cycles and you'll get
	huge Accel, and jerks even though the path generated was smooth.

	*/

	
	// Go through the road vehicles (from sensor fusion) and generate possible maneuvers. This will lead me to a target (position, speed and time)
	vector<tuple<Maneuver, maneuver_params>> possible_maneuvers = generate_possible_maneuvers(road_vehicles);
	
	/*
	Let's now EXPLORE each and every one of these possible maneuvers, genetate smooth trajectories,
	calculate the COST of each one and select the BEST one...AND Update the Ego state (aka maneuver)
	*/
	Trajectory best_trajectory = generate_best_trajectory(road_vehicles, possible_maneuvers);
	//////cout << " Best Traj - Target Lane: " << planner.get_lane(best_trajectory.goal_d[0]) << endl;	

	// When we do a Lane change, we might got cut in the middle and we want ot remember target speed and target lane to avoid re-calc 
	this->target_lane = planner.get_lane(best_trajectory.goal_d[0]);
	this->prev_target_speed = best_trajectory.goal_s[1];

	this->state = best_trajectory.maneuver;

	planner.following_count += (best_trajectory.maneuver == ST_FOLLOW) ? 1 : -planner.following_count;

	// Finally let's generate the next_x_vals and next_y_vals	
	smooth_trajectory_frenet2(best_trajectory, prev_x_vals, prev_y_vals, next_x_vals, next_y_vals);

	// What we send to the simulator will be Executed...There is no turn back. Therefore, Now is when I can update the target lane we sent
	// let's update the target lane for next pass AND the new "State" (Which is the menuver that the vehicle is executing)
	//this->target_lane = planner.get_lane(planner.final_d[0]);
	////cout << " Actual - Target Lane: " << this->target_lane << endl;
	
}

vector < tuple<Maneuver, maneuver_params > > SmartVehicle::generate_possible_maneuvers(vector<Road_Vehicle> road_vehicles)
{
	//////cout << "****************************************************************" << endl;	
	//////cout << "**************** Generating possible maneuvers *****************" << endl;
	double Ego_s = Ego_start_status.s[0];
	double Ego_vs = Ego_start_status.s[1];
	int Ego_lane = Ego_start_status.lane;
	double Ego_d = Ego_start_status.d[0];
	/*
	We will have to make decisions NOT based on where EGO is but from WHERE it will be once we can actually execute
	the maneuver. That means that, since the SIM will execute 
	*/
	// let's create the return structure;
	vector < tuple<Maneuver, maneuver_params > > possible_maneuvers;

	// Let's Start by getting the DATA from the closest vehicle in front of Ego, AKA Vehicle A		
	Road_Vehicle Vehicle_A = planner.get_closest_vehicle_in_lane(Ego_lane, Ego_s, true, road_vehicles);

	/* ************* EMERGENCY MANEUVER ************
	   ********************************************/
	if (!Vehicle_A.empty()) 
	{					
		//Vehicle_A.set_TTC(Ego_s, Ego_vs);
		//Vehicle_A.set_TIV(Ego_s);
		/*
		If we have a car in front and we are TOO close, we need to go into WAIT (Vehicle Following)
		*/
		if (Vehicle_A.predict_frenet_at(0.0).s < this->s + VEHICLE_SIZE[0])
		{
			cout << "EMERGENCY..." << endl;
			maneuver_params params = generate_emergency_params(Vehicle_A);			
			possible_maneuvers.push_back(make_tuple(ST_FOLLOW, params));
			return possible_maneuvers;

		}		
	}

	/* ************* ESCAPE MANEUVER ************
	 We have been following A and stuck by B on one side or both sides
	********************************************/
	/*
	if (planner.following_count == 100) // this should be around 20 sec (aprox 30 points consumed by Sim * 0.02 * count = n secs)..depending on how fast the Sim responds back and how fast I process everything
	{
		cout << "ESCAPING..." << endl;
		maneuver_params params = generate_emergency_params(Vehicle_A);
		possible_maneuvers.push_back(make_tuple(ST_FOLLOW, params));
		return possible_maneuvers;
	}
	*/

	follow_maneuver_added = false;		

	double Vdes = planner.Vs_desired;
	double min_cruise_distance = MIN_FOLLOW_DISTANCE + Ego_vs * DESIRED_TIME_GAP_FOLLOWING;

	////cout << "WE ARE IN..." << endl;
	switch (this->state)
	{
		case ST_CRUISE_CONTROL:
		{
			cout << "CRUISE STATE(0): " << this->state << endl;
			////cout << "      min_cruise_distance: " << min_cruise_distance << endl;			
			/* If our car is in any lane num smaller than lanes available,
			and assuming lane 0 is on tha far right, we can always make a Lane-Change-Left
			*/
			if (Vehicle_A.empty() || ( (Vehicle_A.s > (Ego_s + min_cruise_distance)) /*&& Vehicle_A.TTC > MIN_TTC && Vehicle_A.TIV > MIN_TIV*/) ) // 
			{
				cout << "Continue cruising " << endl;
				maneuver_params params = generate_cruise_control_params(Vdes);
				possible_maneuvers.push_back(make_tuple(ST_CRUISE_CONTROL, params));
			}
			else // We have a vehicle in front AND we are too close... We will go into Vehicle Following State
			{
				cout << "We have a car in front..too close " << endl;			
				////cout << "Going into Following state " << endl;
				////cout << "      Vehicle_A.TTC: " << Vehicle_A.TTC << " MIN_TTC: " << MIN_TTC << endl;
				////cout << "      Vehicle_A.TIV: " << Vehicle_A.TIV << " MIN_TIV: " << MIN_TIV << endl;
				maneuver_params params = generate_follow_params(Vehicle_A);					
				possible_maneuvers.push_back(make_tuple(ST_FOLLOW, params));								
			}
			break;
		}

		case ST_FOLLOW:	
		{
			cout << "FOLLOWING STATE(1): " << this->state << endl;
			
			/* If We have NO car infront ...we can go into Cruise Control.
			   This is the case where, despite the fact that we were in WAIT state and therefore
			   we had a car infront and couldn't change lanes yet, the car in front actually 
			   changed lane leaving us with room to go into cruise control.
			*/
			if (Vehicle_A.empty()) 
			{
				////cout << "No cars infront! "<< endl;
				maneuver_params params = generate_cruise_control_params(Vdes);
				possible_maneuvers.push_back(make_tuple(ST_CRUISE_CONTROL, params));
			}
			else // We have a vehicle in front
			{
				////cout << "We have a car in front " << endl;					
				// We can always Stay following
				maneuver_params params = generate_follow_params(Vehicle_A);
				possible_maneuvers.push_back(make_tuple(ST_FOLLOW, params));

				/* If our car is in any lane num smaller than lanes available,
				and assuming lane 0 is the FAST LANE, then we can always make a Lane-Change-RIGHT
				*/
				if (Ego_lane < (this->planner.road.num_lanes - 1))
				{
					//////cout << "9 - Wait -> Right_lane " << endl;
					int Right_lane = Ego_lane + 1;
					
					////cout << "NO car on Right_lane: " << Right_lane << endl;
					maneuver_params params = generate_change_lane_params(Right_lane, Ego_vs * LANE_CHANGE_SLOW_DOWN_FACTOR);
					possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params));
					/*
					Road_Vehicle Vehicle_B  = planner.get_closest_vehicle_in_lane(Right_lane, Ego_s, false, road_vehicles);					
					if (Vehicle_B.empty()) // We just need to perform a lane change without hesitation, because we don't have anyone in the left lane
					{
						////cout << "NO car on Right_lane: " << Right_lane << endl;
						maneuver_params params = generate_change_lane_params(Right_lane, Vdes * LANE_CHANGE_SLOW_DOWN_FACTOR);
						possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params));
					}
					else
					{
						////cout << "We DO have a car on Right_lane: " << Right_lane << endl;
						// EVALUATE TRANSITION CONDITIONS THAT COULD EXIT THIS STATE						
						Vehicle_B.set_TTC(Ego_s, Ego_vs);
						Vehicle_B.set_TIV(Ego_s);
						// In case we do a change lane infront of B, we need also to see who might be enfront of B to determine the the appropriate FINAL SPEED...and distance
						Road_Vehicle Vehicle_B2 = planner.get_closest_vehicle_in_lane(Right_lane, Vehicle_B.s, true, road_vehicles);
						update_possible_maneuvers(Right_lane, Vehicle_A, Vehicle_B, Vehicle_B2, possible_maneuvers);
					}
				    */
				}
				/*
				if our car is in any lane but the FAST lane (0), it can always make a Lane-Change-LEFT
				*/
				if (Ego_lane > 0)
				{
					//////cout << "9 - Wait -> Right lane " << endl;
					int Left_lane = Ego_lane - 1;
					maneuver_params params = generate_change_lane_params(Left_lane, Ego_vs * LANE_CHANGE_SLOW_DOWN_FACTOR);
					possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params));
					/*
					Road_Vehicle Vehicle_B = planner.get_closest_vehicle_in_lane(Left_lane, Ego_s, false, road_vehicles);
					if (Vehicle_B.empty()) // WE just need to perform a lane change without hesitation, because we don't have anyone in the Right lane
					{
						////cout << "NO car on Left_lane: " << Left_lane << endl;
						maneuver_params params = generate_change_lane_params(Left_lane, Ego_vs* LANE_CHANGE_SLOW_DOWN_FACTOR);
						possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params));
					}
					else
					{
						////cout << "We DO have a car on Left_lane: " << Left_lane << endl;
						// EVALUATE TRANSITION CONDITIONS THAT COULD EXIT THIS STATE						
						Vehicle_B.set_TTC(Ego_s, Ego_vs);
						Vehicle_B.set_TIV(Ego_s);
						// In case we do a change lane infront of B, we need also to see who might be enfront of B to determine the the appropriate FINAL SPEED...and distance
						Road_Vehicle Vehicle_B2 = planner.get_closest_vehicle_in_lane(Left_lane, Vehicle_B.s, true, road_vehicles);
						update_possible_maneuvers(Left_lane, Vehicle_A, Vehicle_B, Vehicle_B2, possible_maneuvers);
					}
					*/

				}

			}
		
			break;
		}

		case ST_LANE_CHANGE:
		{
			/* For now, the only states OUT of these 2 states is "CRUISE CONTROL", but how do I abort?? How to I check the road while I'm
			performing the maneuver?? This is still an open issue that requires more time, brainstorming, research and testing
			*/
			cout << "LANE CHANGE STATE" << endl;
		
			if (fabs(Ego_d - planner.lane2d(this->target_lane)) < LANE_TOLERANCE) // Everytime we finish a manouver, we target to be at 0 accel.
			{				
				////cout << "LANE CHANGE FINISHED " << endl;
				/* If our car is in any lane num smaller than lanes available,
				and assuming lane 0 is on tha far right, we can always make a Lane-Change-Left
				*/
				if (Vehicle_A.empty() || ((Vehicle_A.s > (Ego_s + min_cruise_distance)) && Vehicle_A.TTC > MIN_TTC && Vehicle_A.TIV > MIN_TIV)) // 
				{
					cout << "Go into cruising " << endl;
					maneuver_params params = generate_cruise_control_params(Vdes);
					possible_maneuvers.push_back(make_tuple(ST_CRUISE_CONTROL, params));
				}
				else // We have a vehicle in front AND we are too close... We will go into Vehicle Following State
				{
					////cout << "We have a car in front..to close " << endl;
					cout << "Going into Following state " << endl;
					maneuver_params params = generate_follow_params(Vehicle_A);
					possible_maneuvers.push_back(make_tuple(ST_FOLLOW, params));
				}				
			}
			else
			{
				cout << "Continue with the lane Change. Target lane: " << this->target_lane  << endl;
				//The most efficient thing to do is to do nothing!! Send our previous goal conditions...i.e. now are our Start Conditions
				// But the problem is that we don't know how long will it take from NOW. So it's better just to recalculate it all
				maneuver_params params = generate_change_lane_params(this->target_lane, this->prev_target_speed);
				possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params));		
			}

			break;
		}
	
		default:
			break;
		}
	
	////cout << "Returning maneuvers... " << possible_maneuvers.size() << endl;
	return possible_maneuvers;
}

double SmartVehicle::get_maneuver_time(Ego_status Ego, double desired_speed)
{
	// for the S coordinate...Assuptions are: Accel_final = 0
	double T1 = Ego.s[2] / MAX_JERK;

	double T2;
	double delta_v = (desired_speed - Ego.s[1]);
	if (delta_v < 0) // we are decelerating
	{
		T2 = fabs(delta_v) / MAX_DECEL;
	}
	else
	{
		T2 = delta_v / MAX_ACCEL;
	}

	
	// for the d coordinate...Assuptions are: Accel_final = 0 AND also final speed = 0
	double T3 = Ego.d[2] / MAX_LATERAL_JERK;
	double T4 = Ego.d[1] / MAX_ACCEL;

	return max(max(max(T1, T2), max(T3, T4)), MIN_MANEUVER_TIME); // This should not be Zero so we can plan..even if we have to plan NOT moving
}

double SmartVehicle::get_maneuver_accel(Ego_status Ego, double desired_speed, double maneuver_time)
{
	// for the S coordinate...Assuptions are: Accel_final = 0
	double Accel_s = (desired_speed - Ego.s[1]) / maneuver_time;
	if (Accel_s < 0)
		Accel_s = max(Accel_s, -MAX_DECEL);
	else
		Accel_s = min(Accel_s, MAX_ACCEL);

	// for the d coordinate...We don't care since we HAVE ALL FINAL values we need: d_, d_dot, d_ddot. = target_lane, 0,0
	// In the case of S we need to calculate target S and therefore we need and Accel, apart from the initial speed and position which we have
	
	return Accel_s;
}

maneuver_params SmartVehicle::generate_follow_params(Road_Vehicle Vehicle_A)
{	
	/*

	This following algorithm is based on the Intelligent driver model
	https://en.wikipedia.org/wiki/Intelligent_driver_model

	*/

	/* These are always our starting conditions (for every cycle).
	They are the conditions at the end of the last Traj sent to the sim or
	if we are at time = 0, then they are the Ego speed and position at time
	NOTE: The decisions made by the planner are made based on ACTUAL ego conditions BUT executed
	at the END of the previous traj conditions.
	*/	
	double E_s  = Ego_start_status.s[0]; // position s
	double E_vs = Ego_start_status.s[1]; // velocity s
	double E_as = Ego_start_status.s[2]; // acceleration s
	double A_s = Vehicle_A.s;
	double A_vs = Vehicle_A.speed;	


	// *************************************************		

	//velocity difference
	double deltaV = E_vs - A_vs;

	//desired gap to leading vehicle
	double s_star = MIN_FOLLOW_DISTANCE + 
					max(E_vs * DESIRED_TIME_GAP_FOLLOWING, 0.0) +
					((E_vs * deltaV) / (2 * sqrt(MAX_ACCEL * MAX_DECEL)));

	// current gap
	double s_alpha = A_s - E_s - VEHICLE_SIZE[0];
	if (s_alpha < 0)  // deal with periodic boundary condition: i.e when we go all around the track and vehicles go back to s = 0;
	{
		//cout << " End of the Road???" << endl;
		s_alpha += this->planner.road.TRACK_LENGTH;
	}

	double gap_term = pow(s_star / s_alpha, GAMMA);	

	// speed limit Term	
	double limit_term = pow(E_vs / planner.Vs_desired, DELTA);

	double Temp = (1.0 - limit_term - gap_term);
	double desired_accel;
	if (Temp < 0)
		desired_accel = max(MAX_DECEL * Temp, -MAX_DECEL);
	else
		desired_accel = min(MAX_ACCEL * Temp, MAX_ACCEL);

	double duration = max(fabs(desired_accel - E_as) / MAX_JERK, MIN_MANEUVER_TIME);
	duration = int(duration / dt)*dt; // Truncating
	double target_s = E_s + (E_vs*duration) + (0.5*desired_accel * duration*duration);
	double target_vs = E_vs + (desired_accel*duration);
	// *************************************************
	
	maneuver_params params;
	params.dir = 0;
	params.target_s = max(target_s,E_s); // For now, I don't want the Ego to backup
	params.target_speed = min(target_vs, planner.road.speed_limit);
	params.duration = duration;
	
	
	//cout << "FOLLOW PARAMS: " << endl;
	//cout << "      dir: " << params.dir << endl;
	//cout << "      target_s: " << params.target_s << endl;
	//cout << "      target_speed: " << params.target_speed << endl;
	//cout << "      duration: " << params.duration << endl;
	
	return params;
}

maneuver_params SmartVehicle::generate_emergency_params(Road_Vehicle Vehicle_A)
{
	// For now, just try to follow Vehicle A but slow down 40%
	Road_Vehicle Dummy = Vehicle_A;
	Dummy.speed = Dummy.speed*0.6;

	return generate_follow_params(Dummy);		
}

maneuver_params SmartVehicle::generate_escape_params(Road_Vehicle Vehicle_A)
{
	// For now, just try to follow Vehicle A but slow down 10%...to give us a chance to move behind B
	Road_Vehicle Dummy = Vehicle_A;
	Dummy.speed = Dummy.speed*0.1;

	return generate_follow_params(Dummy);
}

maneuver_params SmartVehicle::generate_cruise_control_params(double target_speed)
{

	/* These are always our starting conditions (for every cycle).
	They are the conditions at the end of the last Traj sent to the sim or
	if we are at time = 0, then they are the Ego speed and position at time
	NOTE: The decisions made by the planner are made based on ACTUAL ego conditions BUT executed
	at the END of the previous traj conditions.
	*/
	double E_s = Ego_start_status.s[0];
	double E_vs = Ego_start_status.s[1];

	double time = get_maneuver_time(Ego_start_status, target_speed);	
	double accel = get_maneuver_accel(Ego_start_status, target_speed, time);

	double target_s = max(E_s + E_vs * time + 0.5*accel*time*time, E_s); // Preventing from Negative Speeds generating a pos behind us. Not likely ever
	//////cout << "   Wait Params: Target_s= " << target_s << " T: " << Time << endl;

	maneuver_params params;
	params.dir = 0;
	params.target_s = target_s;
	params.target_speed = min(target_speed, planner.road.speed_limit);
	params.duration = time;
	
	
	//cout << "CRUISE PARAMS: " << endl;
	//cout << "      dir: " << params.dir << endl;
	//cout << "      target_s: " << params.target_s << endl;
	//cout << "      target_speed: " << params.target_speed << endl;
	//cout << "      duration: " << params.duration << endl;
	
	return params;
}

maneuver_params SmartVehicle::generate_change_lane_params(double target_lane, double target_speed)
{		

	
	/* These are always our starting conditions (for every cycle).
	They are the conditions at the end of the last Traj sent to the sim or
	if we are at time = 0, then they are the Ego speed and position at time
	NOTE: The decisions made by the planner are made based on ACTUAL ego conditions BUT executed
	at the END of the previous traj conditions.
	*/
	
	double E_s = Ego_start_status.s[0];
	double E_vs = Ego_start_status.s[1];
	double E_d = Ego_start_status.d[0];
	/*
	double time_guess = get_maneuver_time(Ego_start_status, target_speed);
	double accel = fabs(get_maneuver_accel(Ego_start_status, target_speed, time_guess));	
	*/
	double accel = MAX_ACCEL/1.5;
	double speed_ini = planner.Vs_desired; //E_vs; //planner.Vs_desired; // min(target_speed, E_vs);
	cout << " Ego Speed - ini: " << speed_ini << endl;
	// Numerical Aproximation of Target S for the lane change (LC) maneuver (Really this is the distance of the maneuver and NOT the total distance. Total s = actual S + manuver S			
	double lane_width = planner.road.lane_width; ;// fabs(planner.lane2d(target_lane) - E_d); //planner.road.lane_width;


	/*
	For high enough	velocities, (say V ~ 5m / s), it was obvious that the
	optimal value of D* (distance on S coordinate to performe the maneuver) is approximately proportional to V (initial speed), to
	the square root of W, and inversely proportional to the
	square root of A (max Accelration)
	FROM: Overtaking a Slower-Moving Vehicle by an Autonomous Vehicle -  by T. Shamir
	*/
	if (speed_ini < 5) speed_ini = 5;

	double target_s = 2.4 * speed_ini * sqrt(lane_width / accel) ;
	
	// Numerical Aproximation of duration of the maneuver in sec.
	double time = ( sqrt(3)*pow(lane_width, 1.5)*sqrt(accel) / (speed_ini * speed_ini) ) + (target_s / speed_ini) ;
	
	target_s += E_s;
	
	/*
	double time = time_guess + 0.2;
	double target_s = max(E_s + E_vs * time + 0.5*accel*time*time, E_s);
	*/

	//maneuver_params params;
	maneuver_params params;
	params.dir = (target_lane - Ego_start_status.lane);	
	params.target_s = target_s;
	params.target_speed = min(target_speed, planner.road.speed_limit);
	params.duration = time;
	
	//cout << "Change lane PARAMS: " << endl;
	//cout << "      Guessed Time: " << time_guess << endl;
	//cout << "      Accel: " << accel << endl;
	//cout << "      dir: " << params.dir << endl;
	//cout << "      target_s: " << params.target_s << endl;
	//cout << "      target_speed: " << params.target_speed << endl;
	//cout << "      duration: " << params.duration << endl;

	return params;
}

void SmartVehicle::add_lane_change_infront_B_maneuver(Road_Vehicle VehicleB, Road_Vehicle VehicleB2, int target_lane, vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers)
{
	double E_s = Ego_start_status.s[0];
	double E_vs = Ego_start_status.s[1];
	double B_s = VehicleB.s;
	double B_vs = VehicleB.speed;
	double V_desired = planner.Vs_desired;

	double di = fabs(E_s - B_s) - VEHICLE_SIZE[0]; // Assuming both Ego and B cars have roughly the same length				

	if (VehicleB2.empty()) // We can aim at max speed
	{
		////cout << "No B2. We can go at max speed" << endl;
		maneuver_params params_change_lane_no_obstacle = generate_change_lane_params(target_lane, V_desired* LANE_CHANGE_SLOW_DOWN_FACTOR);		
		double d_min = E_s + (B_vs*(params_change_lane_no_obstacle.duration + 0.5) - di);
		params_change_lane_no_obstacle.target_s = max(params_change_lane_no_obstacle.target_s, d_min);
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params_change_lane_no_obstacle));
		return;
	}
	else
	{
		////cout << "B2 is present." << endl;
		maneuver_params params_change_lane_with_obstacle = generate_change_lane_params(target_lane, min(VehicleB2.speed,V_desired* LANE_CHANGE_SLOW_DOWN_FACTOR));				
		double d_min = E_s + (B_vs*(params_change_lane_with_obstacle.duration + 0.5) - di);
		params_change_lane_with_obstacle.target_s = max(params_change_lane_with_obstacle.target_s, d_min);
		if ( params_change_lane_with_obstacle.target_s > (VehicleB2.predict_frenet_at(params_change_lane_with_obstacle.duration).s - MIN_FOLLOW_DISTANCE) )
		{
			////cout << "We CAN change lane safely" << endl;
			possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params_change_lane_with_obstacle));
			return;
		}
		else
		{
			////cout << "We CAN NOT change lane safely -> going to FOLLOWING STATE" << endl;			
			add_follow_maneuver(possible_maneuvers);
			return;
		}
	}
}

void SmartVehicle::add_lane_change_behind_B_maneuver(Road_Vehicle VehicleB, int target_lane, vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers)
{
	double E_s = Ego_start_status.s[0];
	double E_vs = Ego_start_status.s[1];
	double B_s = VehicleB.s;
	double B_vs = VehicleB.speed;
	double V_desired = planner.Vs_desired; //Ego_start_status.s[1];// 

	maneuver_params params_change_lane_with_obstacle = generate_change_lane_params(target_lane, min(B_vs, V_desired* LANE_CHANGE_SLOW_DOWN_FACTOR));

	double di = fabs(E_s - B_s) - VEHICLE_SIZE[0]; // Assuming both Ego and B cars have roughly the same length					
	double d_max = E_s + (B_vs*(params_change_lane_with_obstacle.duration - 0.5) + di);

	params_change_lane_with_obstacle.target_s = min(params_change_lane_with_obstacle.target_s, d_max); // since vehicle B is going slower and is behind us, we need to make sure our final distance when we reach the top speed is actually bigger than this value so we are safe from B hitting or getting too close to us 		
	possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, params_change_lane_with_obstacle));

	return;
}

void SmartVehicle::add_follow_maneuver(vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers)
{
	possible_maneuvers.push_back(make_tuple(ST_FOLLOW,this->follow_maneuver_params));
	follow_maneuver_added = true;
	return;
}

void SmartVehicle::update_possible_maneuvers(int target_lane, Road_Vehicle Vehicle_A, Road_Vehicle Vehicle_B, Road_Vehicle VehicleB2, vector < tuple<Maneuver, maneuver_params > > &possible_maneuvers)
{					
	// Conditions to evaluate in relation to Velocity, Position, TTC and TIV
	bool Vel_Condition;
	bool Pos_Condition;
	bool TTC_condition;
	bool TIV_condition;
	bool Extra_condition;
	bool TransCond; // This is the Full "Transition Condition" (just for convenience)

	double E_s  = Ego_start_status.s[0]; // S coodinate of Ego car at the start of the planning trajectory. Which means 2-3 secs ahead and therefore HORRIBLE APROACH!!!
	double E_vs = Ego_start_status.s[1];
	double Vdes = this->planner.Vs_desired ; // Desired Speed = Speed Limit - 0.5mph or 0.22 m/s

	////cout << "EGO       - s: " << E_s << " vs: " << E_vs << " Vdes: " << Vdes << " Accel: " << Ego_start_status.s[2] << endl;
	// Vehicle A
	// WE should NOT worry about Vehicle A being Empty..since we look at it before executing this..always!
	double A_s = Vehicle_A.s;	
	double A_vs = Vehicle_A.speed; // this is "fair enough"  (as we should be comparing Speeds on S coordinate and not prure magnitudes)aprox since we are assuming that most of the speed vector is on the s direction and just a sliglty portion could go for a change lane.
	////cout << "Vehicle A - s: " << A_s << " vs: " << A_vs << endl;
	
	// Vehicle B 
	// WE should NOT worry about Vehicle B being Empty..since we look at it before executing this..always!	
	double B_s = Vehicle_B.s;
	double B_vs = Vehicle_B.speed; // this is "fair enough"  (as we should be comparing Speeds on S coordinate and not prure magnitudes)aprox since we are assuming that most of the speed vector is on the s direction and just a sliglty portion could go for a change lane.	
	double B_TTC = Vehicle_B.TTC;
	double B_TIV = Vehicle_B.TIV;	
	////cout << "Vehicle B - s: " << B_s << " vs: " << B_vs << endl;
	////cout << "          TTC: " << B_TTC << " TIV: " << B_TIV << endl;		
	////cout << "Follow Maneuver Added??:" << follow_maneuver_added << endl;

	
	// Pre-calculated manuver parameters
	this->follow_maneuver_params = generate_follow_params(Vehicle_A);		
		
	// Transition 1: From ST_FOLLOW --> ST_LANE_CHANGE
	/*
	This is the case where vehicle b has the fastest velocity and already driving
	longitudinally in front of the ego vehicle. In this case, a normal optimized lane
	change trajectory should be initiated i.e. the trajectory designed for the case
	where there is no obstacle...unless Vechicle B2 is present
	*/
	Vel_Condition = (A_vs < Vdes && Vdes < B_vs);
	Pos_Condition = (B_s > E_s + VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond) 
	{		
		////cout << "Transition 1: From ST_FOLLOW --> ST_LANE_CHANGE Leading Vehicle b" << endl;
		add_lane_change_infront_B_maneuver(Vehicle_B, VehicleB2, target_lane, possible_maneuvers);
		return;
	}

	// Transition 2: From ST_FOLLOW --> ST_FOLLOW	
	/* in this case, since vehicle B is driving at a higher speed than everybody else, and is within 1 car distance beside Ego,
	   Ego vehicle should wait until vehicle b pass before making a lane change menuver
	*/
	Vel_Condition = (A_vs < Vdes && Vdes < B_vs);
	Pos_Condition = (B_s < E_s + VEHICLE_SIZE[0]) && (B_s > E_s - VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond && !follow_maneuver_added)
	{		
		////cout << "Transition 2: From ST_FOLLOW --> ST_FOLLOW" << endl;		
		add_follow_maneuver(possible_maneuvers);
		return;
	}

	// Transition 3: From ST_FOLLOW --> ST_LANE_CHANGE, Lead Vehicle b after lane change
	/* Similar case than before, but now B is behind ego, at least by one car distance. If the TTC and TIV condicions are met,
	   Ego could iniciate an lane change ahead from B
	*/
	Vel_Condition = (A_vs < Vdes && Vdes < B_vs );
	Pos_Condition = (B_s < E_s - VEHICLE_SIZE[0]);
	TTC_condition = (B_TTC > MIN_TTC);
	TIV_condition = (B_TIV > MIN_TIV);
	Extra_condition =(B_vs - A_vs < RELATIVE_VELOCITY_THRESHOLD);
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition && Extra_condition);
	if (TransCond)
	{		
		////cout << "Transition 3: From ST_FOLLOW --> ST_LANE_CHANGE Leading Vehicle b" << endl;
		add_lane_change_infront_B_maneuver(Vehicle_B, VehicleB2, target_lane, possible_maneuvers);
		return;
	}

	// Transition 4: From ST_FOLLOW --> ST_FOLLOW (Otherwise)	
	/*	this case is when vehicle b is driving with a higher speed
		than the ego vehicle very closely. In this case, a command of wait is given
		until the vehicle b passed the ego vehicle and then initiate a lane change command
	*/
    //Vel_Condition = A_vs < Vdes && Vdes < B_vs) ;
	Pos_Condition = true;
	//TTC_condition = (B_TTC < MIN_TTC);
	//TIV_condition = (B_TIV < MIN_TIV);
	TransCond = (Vel_Condition && Pos_Condition && (!TTC_condition || !TIV_condition || !Extra_condition));
	if (TransCond && !follow_maneuver_added)
	{
		////cout << "Transition 4: From ST_FOLLOW --> ST_FOLLOW (Otherwise)" << endl;		
		add_follow_maneuver(possible_maneuvers);
		return;
	}

	// Transition 5: From ST_FOLLOW --> ST_LANE_CHANGE
	/* Here, this is the case very similar to Transition 1. However, the difference is the ego
	   vehicle wants to drive at a speed faster than vehicle b. In this case,
       still make an acceleration lane change but with the final velocity equal to the
       vehicle b and following behind it. Since we are going behind B, we don't really have to care about B2
	*/
	Vel_Condition = (A_vs < E_vs && E_vs < B_vs && B_vs < Vdes);
	Pos_Condition = (B_s > E_s + VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
	{
		////cout << "Transition 5: From ST_FOLLOW --> ST_LANE_CHANGE Behind Vehicle b " << endl;
		add_lane_change_behind_B_maneuver(Vehicle_B, target_lane, possible_maneuvers);
		return;
	}

	// Transition 6: From ST_FOLLOW --> ST_FOLLOW
	/* In this case, since the velocity of vehicle b is higher, ego vehicle should just
	   choose to wait until vehicle b pass. And then, it enters into the case of Transition 5
	   where an acceleration lane change maneuver will be initiated.
	*/
	Vel_Condition = (A_vs < E_vs && E_vs < B_vs && B_vs < Vdes);
	Pos_Condition = (B_s < E_s + VEHICLE_SIZE[0]) && (B_s > E_s - VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond && !follow_maneuver_added)
	{
		////cout << "Transition 6: From ST_FOLLOW --> ST_FOLLOW" << endl;				
		add_follow_maneuver(possible_maneuvers);
		return;
	}

	// Transition 7: From ST_FOLLOW --> ST_LANE_CHANGE Lead Vehicle b	
	/* Similar to the case of Event 3, but since vehicle b is driving at a slower velocity
	   than the desired velocity set by the driver, an acceleration lane change maneuver
       should be initiated with the nal velocity of the ego vehicle equal to the desired
       velocity.
	*/
	Vel_Condition = (A_vs < E_vs && E_vs < B_vs && B_vs < Vdes);
	Pos_Condition = (B_s < E_s - VEHICLE_SIZE[0]);
	TTC_condition = (B_TTC > MIN_TTC);
	TIV_condition = (B_TIV > MIN_TIV);
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
	{		
		////cout << "Transition 7: From ST_FOLLOW --> ST_LANE_CHANGE Leading Vehicle b" << endl;
		add_lane_change_infront_B_maneuver(Vehicle_B, VehicleB2, target_lane, possible_maneuvers);
		return;
	}

	// Transition 8: From ST_FOLLOW --> ST_FOLLOW (Otherwise)
	Vel_Condition = (A_vs < E_vs && E_vs < B_vs && B_vs < Vdes);
	Pos_Condition = true;
	TTC_condition = (B_TTC < MIN_TTC);
	TIV_condition = (B_TIV < MIN_TIV);
	TransCond = (Vel_Condition && Pos_Condition && (TTC_condition || TIV_condition));
	if (TransCond && !follow_maneuver_added)
	{
		////cout << "Transition 8: From ST_FOLLOW --> ST_FOLLOW (Otherwise)" << endl;	
		add_follow_maneuver(possible_maneuvers);
		return;
	}

	// Transition 9: From ST_FOLLOW --> ST_FOLLOW
	/* Since the vehicle b is driving at the lowest speed among the three vehicles, if
	   the vehicle b is leading the ego vehicle, then the ego vehicle can just wait until, both Ego and A 
       passed vehicle B...and then Ego can initiate a lane change maneuver
	*/
	Vel_Condition = (B_vs < A_vs && A_vs < Vdes);
	Pos_Condition = (B_s >= E_s);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond && !follow_maneuver_added)
	{		
		////cout << "Transition 9: From ST_FOLLOW --> ST_FOLLOW" << endl;		
		add_follow_maneuver(possible_maneuvers);
		return;
	}

	// Transition 10: From ST_FOLLOW --> ST_LANE_CHANGE	
	/*
	Now the vehicle b is surpassed by the ego vehicle or, in other words, the ego vehicle
	is leading, then a normal optimized lane change trajectory can be generated
	*/
	Vel_Condition = (B_vs < A_vs && A_vs < Vdes);
	Pos_Condition = (E_s > B_s);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
	{
		////cout << "Transition 10: From ST_FOLLOW --> ST_LANE_CHANGE Leading Vehicle b" << endl;
		add_lane_change_infront_B_maneuver(Vehicle_B, VehicleB2, target_lane, possible_maneuvers);
		return;
	}

	// Transition 11: From ST_FOLLOW --> ST_LANE_CHANGE
	/* Although both vehicle a and vehicle b are driving at a slower velocity than the
	desired velocity, the adjacent lane i.e. the lane of vehicle b is
	still faster than the current lane. Thus, a lane change maneuver is recommended,
	but the final velocity of the ego vehicle should equal to the velocity of vehicle
	b. Thus, this is a deceleration lane change maneuver
	*/
	Vel_Condition = (A_vs < B_vs && B_vs < E_vs );
	Pos_Condition = (B_s > E_s + VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = B_TIV > MIN_TIV; // This will allow me to make sure I can really fit behind B and slow down
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
	{
		////cout << "Transition 11: From ST_FOLLOW --> ST_LANE_CHANGE Behind Vehicle b (decel)" << endl;
		add_lane_change_behind_B_maneuver(Vehicle_B, target_lane, possible_maneuvers);
		return;
	}
	/*
	else if (Vel_Condition && Pos_Condition && TTC_condition && !follow_maneuver_added)
	{
		// We made all the requirements Vehicle B is violating the TIV condition
		// If we wait until it's slighly further away we should be able to initiate a change lane maneuver
		////cout << "Transition 11 BBBBB - ALMOST: From ST_FOLLOW --> ST_FOLLOW" << endl;
		add_follow_maneuver(possible_maneuvers);
		return;
	}
	*/

	// Transition 12: From ST_FOLLOW --> ST_FOLLOW
	/* In this case, the decision making can be depending on the type of driver. Some drivers
       might want to have an acceleration lane change trajectory while some drivers
       prefer to wait until the vehicle b pass and then do the lane change. Here, to be
       more conservative and ensuring safety, system recommends wait and then make
       the lane change.
	*/
	Vel_Condition = (A_vs < B_vs && B_vs < E_vs );
	Pos_Condition = (B_s < E_s + VEHICLE_SIZE[0]) && (B_s > E_s - VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond && !follow_maneuver_added)
	{
		////cout << "Transition 12: From ST_FOLLOW --> ST_FOLLOW" << endl;		
		add_follow_maneuver(possible_maneuvers);
		return;
	}

	// Transition 13: From ST_FOLLOW --> ST_LANE_CHANGE 
	/*
	In this case, since vehicle is already behind the ego vehicle and driving at a
	slower velocity than the ego vehicle, a lane change maneuver should be initiated..Leading B
	*/
	Vel_Condition = (A_vs < B_vs && B_vs < E_vs);
	Pos_Condition =  (B_s < E_s - VEHICLE_SIZE[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
	{
		////cout << "Transition 13: From ST_FOLLOW --> ST_LANE_CHANGE Leading Vehicle b" << endl;
		add_lane_change_infront_B_maneuver(Vehicle_B, VehicleB2, target_lane, possible_maneuvers);
		return;
	}

	if (possible_maneuvers.empty())
	{
		/* It turns out that when we first move into following state, Ego came from Cruise control and
		its speed was higher than A (that's why we went to Follow). But if ego could NOT change lane
		it will get into the situation where its speed will be "around" A's..sometimes even slower (numerical calc errors).
		The fluctuation below/above A speed can make possible not fitting any transition.
		Also this would happen if all cars come to the point that are at the same speed.
		For this reason, Keep following A and waiting for a slight change on the speed conditions 
		is the only way, unless we design a smarter behavioral planner that takes into account other lanes
		beyond the left/right ones from us. For now, I'll stick with this
		*/
		////cout << "*****************************************************" << endl;
		////cout << "Transition Unkown!!" << endl;		
		add_follow_maneuver(possible_maneuvers);
		return;
	}
}

Trajectory SmartVehicle::generate_best_trajectory(vector<Road_Vehicle> road_vehicles, vector < tuple<Maneuver, maneuver_params > > possible_maneuvers)
{
	//////cout << "****************************************************************" << endl;
	//////cout << "*******************Generating Best Trajectory ******************" << endl;	
	/*
	Let's actually calculate the COST associated to each "possible maneuver"
	Steps to perform:
	1) Go through all possible states
	2) Itentify the goal. Format of "goal: {s, s_dot, s_ddot, d, d_dot, d_ddot, T}
		Where:
		s & d = Frenet Coor,
		s_dot & d_dot = Velocity in frenet Coor
		s_ddot & d_ddot = accel in Frent Coor
		T - the desired time at which we will be at the goal (relative to now as t=0)
		All are target/goal conditions to meet at the end of the maneuver
	3) Perturb the goal to give better chances to fit an approapriate curve.
	4) Go throu all perturbed goals and find the Min Jerk Traj.
	5) Evaluate Cost for each trajectory and save it
	6) Find the min Cost Trajectory: (s_coeffs, d_coeffs, T)
	7) Create the next_x_vals and next_y_vals (thats on the next function - smooth_trajectory)
	*/

	/*
	I WILL BUILD THE TRAJECTORY STARTING FROM WHERE THE LAST TRAJ LEFT...
	*/
	
	// Let's start by building our Initial Vectors
	//vector<double> start_s = { planner.start_s[0], planner.start_s[1], planner.start_s[2] };
	//vector<double> start_d = { planner.start_d[0], planner.start_d[1], planner.start_d[2] };	
	vector<double> start_s = Ego_start_status.s;
	vector<double> start_d = Ego_start_status.d;

	//////cout << "1 - Start: s = " << start_s[0] << " s_dot = " << start_s[1] << " s_ddot = " << start_s[2] << endl;
	//////cout << "2 -        d = " << start_d[0] << " d_dot = " << start_d[1] << " d_ddot = " << start_d[2] << endl;

	vector<double> goal_s;
	vector<double> goal_d;
	vector<double> costs; // Vector that will store the BEST cost for each possible state. Each possible state would have a few trajectories to test
	vector<Trajectory> trajectories;	
	
	//double Ego_as_needed; //Accel needed to achieve our target_s with our initial conditions and limitations. 
	
	double target_d;   
	for (auto test_maneuver : possible_maneuvers)
	{
		// Let's extract the data from the test maneuver		
		Maneuver test_maneuver_id; // = std::get<0>(test_maneuver);
		maneuver_params params; // = std::get<1>(test_maneuver);
		std::tie(test_maneuver_id, params) = test_maneuver;		

		int dir = params.dir;		
		double target_s = params.target_s;
		double target_vs = params.target_speed;
		double goal_duration = params.duration; // Duration of the Maneuver
		target_d = this->planner.lane2d(Ego_start_status.lane + dir); // Our d coordinate target is always the center of the lane

		////cout << "Maneuver ID: " << test_maneuver_id << endl;		
		////cout << "    target_s: " << target_s << endl;
		////cout << "    dir: " << dir << " target_d: " << target_d << endl;
		////cout << "    target_Speed: " << target_vs << endl;
		////cout << "    duration: " << goal_duration << endl; // Duration Target

		
		goal_s = { target_s, target_vs, 0 };
		goal_d = { target_d, 0, 0 };

	   //////cout << "Going to PTG " << endl;
	
		vector<Trajectory> trajectories_per_state = this->planner.PTG(test_maneuver_id, start_s, start_d, goal_s, goal_d, goal_duration);
		vector<double> costs_per_state;
		
		for (auto trajectory : trajectories_per_state)
		{
			double cost = this->planner.calculate_cost(trajectory, road_vehicles, goal_s, goal_d, goal_duration);
			costs_per_state.push_back(cost);
			cout << "Cost for maneuver: " << trajectory.maneuver << " is: " << cost << endl;
		}

		//////cout << "Calc Cost Finished for Trajecs for single state. Costs Added for this state:" << costs_per_state.size() << endl;
		// Until this point we have all the costs for This specific Maneuver (with its multiple trajectories)
		vector<double>::iterator it = min_element(costs_per_state.begin(), costs_per_state.end());
		int BestCostIdx = std::distance(costs_per_state.begin(), it);
		//////cout << "BestCostIdx: " << BestCostIdx << endl;
		costs.push_back(costs_per_state[BestCostIdx]);
		//////cout << "Best Cost Added to Costs: " << costs_per_state[BestCostIdx] << endl;
		trajectories.push_back(trajectories_per_state[BestCostIdx]);

		
		//////cout << "Best Traj Added to Traj" << endl;
	}

	//////cout << "Num Best Traj:" << trajectories.size() << endl;
	//////cout << "Num Best cost:" << costs.size() << endl;
	// No we should have only 1 (best) cost per Maneuver and with a correlating vector of trajectories
	// Let's now calculate the very Best trajectory
	vector<double>::iterator it = min_element(costs.begin(), costs.end());
	int BestCostIdx = std::distance(costs.begin(), it);
	//////cout << "BestCostIdx: " << BestCostIdx << endl;
	//////cout << "Winner Traj Time Len: " << trajectories[BestCostIdx].duration << endl;			

	cout << "BEST TRAJ " << endl;
	cout << "    MANEUVER:  " << trajectories[BestCostIdx].maneuver << endl;
	cout << "    start_s:  " << trajectories[BestCostIdx].start_s[0] << endl;
	cout << "    start_vs: " << trajectories[BestCostIdx].start_s[1] << endl;
	cout << "    start_as: " << trajectories[BestCostIdx].start_s[2] << endl;
	cout << "    start_d:  " << trajectories[BestCostIdx].start_d[0] << endl;
	cout << "    start_vd: " << trajectories[BestCostIdx].start_d[1] << endl;
	cout << "    start_ad: " << trajectories[BestCostIdx].start_d[2] << endl;
	cout << "    goal_s:   " << trajectories[BestCostIdx].goal_s[0] << endl;
	cout << "    goal_vs:  " << trajectories[BestCostIdx].goal_s[1] << endl;
	cout << "    goal_as:  " << trajectories[BestCostIdx].goal_s[2] << endl;
	cout << "    goal_d:   " << trajectories[BestCostIdx].goal_d[0] << endl;
	cout << "    goal_vd:  " << trajectories[BestCostIdx].goal_d[1] << endl;
	cout << "    goal_ad:  " << trajectories[BestCostIdx].goal_d[2] << endl;
	cout << "    Duration: " << trajectories[BestCostIdx].duration << endl;

	
	return trajectories[BestCostIdx];
}

void SmartVehicle::smooth_trajectory_frenet2(Trajectory trajectory, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	//////cout << "********* TRAJECTORY SMOOTHER ************" << endl;
	// ********* TRAJECTORY SMOOTHER ************

	// No matter what, our next trajectory will be constructed at the end of the last one. Therefore, let's copy ALL previous_path points
	int prev_path_size = prev_x_vals.size();
	
	//////cout << "prev_path_size: " << prev_path_size << endl;
	if (prev_path_size > 0)
	{
		for (size_t i = prev_path_size - prev_path_size; i < prev_path_size; i++) // We add only the last 2 the points from prev.
		{
			next_x_vals.push_back(prev_x_vals[i]);
			next_y_vals.push_back(prev_y_vals[i]);
		}

	}
	// WE don't initialize to 0 because we DONT want to add the initial point which is exactly the same as the End point of prev					
	double time;
		
	for (time = dt; (time <= trajectory.duration && prev_path_size < MAX_NUM_POINTS) /*|| (state == ST_LANE_CHANGE && time <= trajectory.duration)*/; time += dt) // let's add 10 more points to use a spline...
	{
		double s = evaluate(trajectory.s_coeff, time);
		double d = evaluate(trajectory.d_coeff, time);
		if (s > Ego_start_status.s[0])
		{
			vector<double> next_wp = this->planner.getXY(s, d);
			next_x_vals.push_back(next_wp[0]);
			next_y_vals.push_back(next_wp[1]);
			prev_path_size++;
			////////cout << "at time: " << time << " s: " << s << " d: " << d << " x: " << next_wp[0] << " y: " << next_wp[1] << endl;
		}
			
	}		
	//////cout << "Final size: " << prev_path_size << endl;		

	// Where did we end up? This point should be the starting point of the next traj.
	time = time - dt;
	//////cout << "last time eval: " << time << endl;
	Ego_start_status.s = evaluate_f_and_N_derivatives(trajectory.s_coeff, time, 2);
	Ego_start_status.d = evaluate_f_and_N_derivatives(trajectory.d_coeff, time, 2);
	Ego_start_status.lane = planner.get_lane(Ego_start_status.d[0]);
	//////cout << "Final- s: " << planner.final_s[0] << " s_dot: " << planner.final_s[1] << " s_ddot: " << planner.final_s[2] << endl;
	//////cout << "Final- d: " << planner.final_d[0] << " d_dot: " << planner.final_d[1] << " d_ddot: " << planner.final_d[2] << endl;
	//////cout << "Sending next Path len = " << next_x_vals.size() << endl;
	
}

