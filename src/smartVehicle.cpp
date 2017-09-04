/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#include <tuple>
#include <stdexcept>
#include <math.h>       /* atan2 */
#include "smartVehicle.h"
#include "utils.h"
#include "spline.h"

using namespace std;
using namespace utils;

/**
* Initializes SmartVehicle
*/
//SmartVehicle::SmartVehicle() {}


SmartVehicle::~SmartVehicle() {}
/**/
void SmartVehicle::update(vector<double> EgoData, vector<vector<double>> sensor_fusion, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	//cout << "SmartVehicle begining Updated" << endl;

	// 1) Unpack nad update the ego vehicle data
	this->x = EgoData[0];
	this->y = EgoData[1];
	this->s = EgoData[2];
	this->d = EgoData[3];
	this->yaw = EgoData[4];
	this->speed = mph2ms(EgoData[5]);
	
	this->lane = planner.get_lane(this->d);

	if (this->s > this->planner.road.max_s)
		this->s = 0;

	cout << "****************************************************************" << endl;
	cout << "0 - Ego s: " << this->s << " Ego d: " << this->d << " Road w = " << this->planner.road.lane_width << endl;
	cout << "0 - Ego x: " << this->x << " Ego y: " << this->y << endl;
	cout << "0 - Ego Speed: " << this->speed << endl;
	cout << "****************************************************************" << endl;
	/*
	
	*/

	calculate_new_starting_point(prev_x_vals, prev_y_vals);
	

	// Now we Update the "State/Maneuver" of Ego based on it's data and the sensor_fusion data
	update_state(sensor_fusion, prev_x_vals, prev_y_vals, next_x_vals, next_y_vals);
	//cruise_control(prev_x_vals, prev_y_vals, next_x_vals, next_y_vals);
	
}

void SmartVehicle::calculate_new_starting_point(vector<double> prev_x_vals, vector<double> prev_y_vals)
{	
	cout << "****************************************************************" << endl;
	cout << "calculate_new_starting_point" << endl;

	/*
	We want to "know" where will we start generating then new trajectory.
	The idea is to build upon our previous Traj. Since the "sensor_fusion" data is only from "now", we will have
	to create a trajectory based from this data and from teh Ego car is reported to be even though we will have to 
	move in this new traj a time "t". We do this simply because while the Sim sent us the telemetry and we processed it, 
	the Ego car has moved a few (unkown) time steps ahead.
	For this reason we really don't know where the Ego car really is and we  don't want to get in this "dangerous" game 
	of trying to perfectly create a new Traj exactly where truly the Ego car is....because we cant!
	On the other hand, we also don't want just to fill-in more points at then end of the "prev_path". The reason is that we might be
	planning a Traj too far into the future.
	The "middle point" or "sweet spot" should be somewhere in the middle. Not to close to the beginning of prev_path but also
	not at the end.
	We know that every single point on "prev_path" is 0.02 sec away from the next. That way if we want to use, let's say, 0.5 sec of the
	prev_path we can take 0.5sec/0.02 = 25 points from prev_path and use this as a starting point for our new traj.
	
	For this reason, I created a new property of the Planner class that is "time_offset_from_prev_path" which actually describes
	where in "time" we want to start generating the new trajectory starting from pre_path beginning. Also we will have to be carefull about
	going too far that we might be outside prev_path. I'll have to make sure that if this time goes beyond prev_path, we just go to the
	end points of prev_path.	
	*/
	double s1;
	double d1;
	double vs1;
	double vd1;
	double as1;
	double ad1;

	double x1;
	double y1;
	double x2;
	double y2;
	double theta;
	double dt = this->planner.dt;
	int use_prev_points = this->planner.time_offset_from_prev_path / dt;
	int prev_path_size = prev_x_vals.size();
	if (use_prev_points > prev_path_size) use_prev_points = prev_path_size;

	this->planner.new_path_x.clear();
	this->planner.new_path_y.clear();

	cout << "Prev_path length: " << prev_path_size << endl;
	if (prev_path_size > 3 && prev_path_size >= use_prev_points)
	{		
		int start_idx = use_prev_points;
		// 1 st we will go to the last point (to re-use) and we will move backwards to calcualte the Speed and Accel that our Ego car should have				
		x1 = prev_x_vals[start_idx - 1];
		y1 = prev_y_vals[start_idx - 1];
		cout << " x1= " << x1 << " y1= " << y1 << endl;

		x2 = prev_x_vals[start_idx - 2];
		y2 = prev_y_vals[start_idx - 2];
		cout << " x2= " << x2 << " y2= " << y2 << endl;

		theta = atan2((y1 - y2), (x1 - x2));		
		vector<double> s_d = this->planner.getFrenet(x1,y1, theta, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);
		planner.ref_theta = theta;

		s1 = s_d[0];
		d1 = s_d[1];
		cout << " s1= " << s1 << " d1= " << d1 << endl;

		double x3 = prev_x_vals[start_idx - 3];
		double y3 = prev_y_vals[start_idx - 3];
		cout << " x3= " << x3 << " y3= " << y3 << endl;
		theta = atan2((y2 - y3), (x2 - x3));
		s_d = this->planner.getFrenet(x2, y2, theta, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);

		double s2 = s_d[0];
		double d2 = s_d[1];
		cout << " s2= " << s2 << " d2= " << d2 << endl;
		vs1 = (s1 - s2) / dt;
		vd1 = (d1 - d2) / dt;

		double x4 = prev_x_vals[start_idx - 4];
		double y4 = prev_y_vals[start_idx - 4];
		cout << " x4= " << x4 << " y4= " << y4 << endl;

		theta = atan2((y3 - y4), (x3 - x4));
		s_d = this->planner.getFrenet(x3, y3, theta, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);

		double s3 = s_d[0];
		double d3 = s_d[1];


		double vs2 = (s2 - s3) / dt;
		double vd2 = (d2 - d3) / dt;
		as1 = (vs1 - vs2) / dt;
		ad1 = (vd1 - vd2) / dt;


		for (int i = 0; i < use_prev_points; i++)
		{
			this->planner.new_path_x.push_back(prev_x_vals[i]);
			this->planner.new_path_y.push_back(prev_y_vals[i]);
		}

	}
	else
	{
		cout << "Not enough points to re-use" << endl;
		s1 = this->s;
		d1 = this->d;
		vs1 = this->speed;// Usually this would happen only at the firs scan, and therefore the Speed would be 0.
		vd1 = 0.0; // We are following the lane perfectly
		as1 = 0.0;
		ad1 = 0.0;
		x1 = this->x;
		y1 = this->y;
		theta = this->yaw;
		x2 = x1 - cos(theta);
		y2 = y1 - sin(theta);
		planner.ref_theta = theta;

		this->planner.new_path_x.push_back(x2);
		this->planner.new_path_x.push_back(x1);

		this->planner.new_path_y.push_back(y2);
		this->planner.new_path_y.push_back(y1);

	}

	// Now we Know where the Ego car need to start to generated the new Traj and also in its correct state ( speed and accel in both coordinates)	
	this->planner.start_s = { s1,vs1,as1 };
	this->planner.start_d = { d1,vd1,ad1 };
	

	cout << "start_s: s: " << s1 << " vs: " << vs1 << " as: " << as1 << endl;
	cout << "start_d: d: " << d1 << " vd: " << vd1 << " ad: " << ad1 << endl;

	cout << "new_path x: " << x2 << " x1: " << x1 << endl;
	cout << "new_path y: " << y2 << " y1: " << y1 << endl;

}

void SmartVehicle::update_state(vector<vector<double>> sensor_fusion, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	/*
	Updates the "state/Maneuver" of the vehicle by assigning one of the
	following values to state':
	ST_CRUISE_CONTROL  // Cruise Control - Maintain a constant speed = SpeedLimit-buffer in the same lane
	ST_WAIT,		   // SmartVehicle Following - Disntance with SmartVehicle in front is < d_min and we need to slow down & speed Up (calc accel) behind it until we can Change lane or move at the desired speed
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

	/*
	To make it easier, I'll ADD the "possible target states" depending on EGO's actual state and lane num
	*/

	// Let's Start by getting the DATA from the closest vehicle in front of Ego, AKA Vehicle A
	vector<double> VehicleA; // { Va,TTC_a,TIV_a}
	get_vehicle_infront_data(sensor_fusion, VehicleA);

	// This vector holds the possible maneouvers: { tuple (maneouver1, params1), tuple (maneouver2, params2),...},	
	vector < tuple<State, int > > possible_maneuvers;

	// Go through the sensor fusion data and generate possible trajectories
	generate_possible_maneuvers(sensor_fusion, VehicleA, possible_maneuvers);


	/*
	Let's now EXPLORE each and every one of these possible maneuvers, genetate smooth trajectories,
	calculate the COST of each one and select the BEST one...AND Update the Ego state (aka maneuver)
	*/
	vector<double> best_trajectory;
	generate_best_trajectory(sensor_fusion, VehicleA, possible_maneuvers, best_trajectory, this->state);

	// Now that we have Best Trajectory, let's actually update the State (maneuver)

	// Finally let's generate the next_x_vals and next_y_vals	
	//smooth_trajectory(best_trajectory, prev_x_vals, prev_y_vals, next_x_vals, next_y_vals);
	cruise_control2(best_trajectory, prev_x_vals, prev_y_vals, next_x_vals, next_y_vals);

}

void SmartVehicle::generate_possible_maneuvers(vector<vector<double>> sensor_fusion, vector<double> VehicleA, vector < tuple<State, int > > &possible_maneuvers)
{
	cout << "*******************Generating possible maneuvers *********************" << endl;

	double Va = VehicleA[0];
	double TTC_a = VehicleA[1];
	double TIV_a = VehicleA[2];

	switch (this->state)
	{
	case ST_WAIT: // If I'm in WAIT it's ONLY because I must have other car in-front!! (No need to check for nearest_infront being empty i.e Va > 0)
	{
		cout << "8 - Wait" << endl;
		/* If our car is in any lane num smaller than lanes available,
		and assuming lane 0 is on tha far right, we can always make a Lane-Change-Left
		*/
		if (this->lane < (this->planner.road.num_lanes - 1))
		{
			cout << "9 - Wait -> Left lane " << endl;
			// EVALUATE TRANSITION CONDITIONS THAT COULD EXIT ST_WAIT TOWARDS "LEFT" (+1 LANE)
			update_possible_maneuvers(sensor_fusion, 1, Va, possible_maneuvers);
		}

		/*
		if our car is in any lane but the first lane, it can always make a Lane-Change-Right
		*/
		if (this->lane > 0)
		{
			cout << "10 - Wait -> Right lane " << endl;
			// EVALUATE TRANSITION CONDITIONS THAT COULD EXIT ST_WAIT TOWARDS "RIGHT" (-1 LANE)
			update_possible_maneuvers(sensor_fusion, -1, Va, possible_maneuvers);
		}

		/*
		I the car in front is far away or it's speeding up, we can actually try to achieve our "desired Speed"
		*/
		if (TTC_a > 1 || TIV_a > 0.5) // Then we can speed up
		{
			cout << "11 - Wait -> move to Cruise Control " << endl;
			possible_maneuvers.push_back(make_tuple(ST_CRUISE_CONTROL, 0));
		}
		else
		{
			cout << "12 - Wait -> Keep Waiting " << endl;
			possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));
		}

		break;
	}

	case ST_LANE_CHANGE:
	{
		/* For now, the only state out of these 2 states is "CRUISE CONTROL", but how do I abort?? How to I check the road while I'm
		performing the maneuver?? This is still an open issue that requires more time, brainstorming, research and testing
		*/
		cout << "stop 12" << endl;
		
		if (this->lane == this->target_lane)
		{
			cout << "Lane change -> Cruise Control" << endl;
			possible_maneuvers.push_back(make_tuple(ST_CRUISE_CONTROL, 0));
		}
		else
		{
			cout << "Continue with the lane Change" << endl;
			possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, this->target_lane));
		}

		break;
	}

	case ST_CRUISE_CONTROL:
	{

		cout << "14 - Cruise Control " << endl;
		if (TTC_a < 1 && TIV_a < 0.5) // Then we need to slow down
		{
			cout << "15 - Cruise Control -> Wait " << endl;
			possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));
		}
		else
		{
			cout << "15 - Cruise Control -> Continue with Cruise Control" << endl;
			possible_maneuvers.push_back(make_tuple(ST_CRUISE_CONTROL, 0));
		}
		break;
	}

	default:
		break;
	}

}

void SmartVehicle::update_possible_maneuvers(vector< vector<double> > sensor_fusion, int dir, double Va, vector < tuple<State, int > > &possible_maneuvers)
{
	// the caller of this function should check that there is a left or Right adjacent lanes available.
	//  for left changes, "dir = +1", for Right changes "dir = -1".
	int adjLane = this->lane + dir;	
	
		
	// Initialize needed variables
	vector<double> params; // parameters of the maneouver
	// We know we always add the direction
	params.push_back(dir);

	double Se = this->s; // S coodinate of Ego car
	double Vdes = this->planner.road.speed_limit; // Desired Speed
	
	// Conditions to evaluate in relation to Velocity, Position, TTC and TIV
	bool Vel_Condition;
	bool Pos_Condition;
	bool TTC_condition;
	bool TIV_condition;
	bool TransCond; // This is the Full "Transition Condition" (just for convenience)

	
	// FIND NEREST VEHICLES IN EGO'S ADJACENT LANE. Vectors format [ id, x, y, vx, vy, s, d]	
	vector<vector<double>> filtered_adj_lane = planner.filter_sensor_fusion_by_lane(sensor_fusion, adjLane);
	vector<double> nearest_adj_lane;
	planner.nearestAbs(filtered_adj_lane, this->s, nearest_adj_lane);	

	// Let's check if we actually have a vehicle on the adjacent lane close to Ego (aka Vehicle B)
	double Sb = 0.0; // Frenet S distance of vehicle B
	double Vb = 0.0; // Speed of Vehicle B
	double di = 0.0; // this is the longitudinal distance between Ego and B (nearest car in adjacent lane) taking into account the vehicle lengths
	
	if (!nearest_adj_lane.empty())
	{
		double Vb_x = nearest_adj_lane[3];
		double Vb_y = nearest_adj_lane[4];
		Vb = sqrt(Vb_x*Vb_x + Vb_y*Vb_y);
		Sb = nearest_adj_lane[5];
		di = fabs(this->s - Sb) - this->vechileSize[0]; // Assuming both Ego and B cars have roughly the same length						
	}
	
	double TTC = 1000;
	if (fabs(this->vs - Vb) > 0.001)
		TTC = ((this->s - Sb - this->vechileSize[0]) / (Vb - this->vs));

	double TIV = ((this->s - Sb - this->vechileSize[0]) / (Vb));

	// Numerical Aproximation of Target S for the change lane maneuver
	double target_s = 2.4 * this->vs * sqrt(this->planner.road.lane_width / this->planner.MAX_ACCEL);
	// Numerical Aproximation of duration of the maneuver in sec.
	double T = sqrt(3)*pow(this->planner.road.lane_width, 1.5)*sqrt(this->planner.MAX_ACCEL) / (this->vs * this->vs) + (target_s / this->vs); 

	double Accel_s_needed;

	// Transition 1: From ST_WAIT --> ST_LANE_CHANGE
	/*
	This is the case where vehicle b has the fastest velocity and already driving
	longitudinally in front of the ego vehicle. In this case, a normal optimized lane
	change trajectory should be initiated i.e. the trajectory designed for the case
	where there is no obstacle
	*/
	Vel_Condition = (Va < Vdes && Vdes < Vb);
	Pos_Condition = (Sb > this->s + this->vechileSize[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	Accel_s_needed = 0.0; // Lane change without obstacles. 
	if (TransCond)
	{

		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));
	}

	// Transition 2: From ST_WAIT --> ST_WAIT	
	Vel_Condition = (Va < Vdes && Vdes < Vb);
	Pos_Condition = (Sb < this->s + this->vechileSize[0]) && (Sb > this->s - this->vechileSize[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));

	// Transition 3: From ST_WAIT --> ST_LANE_CHANGE with vEf = vb, Lead Vehicle b after lane change	
	Vel_Condition = (Va < Vdes && Vdes < Vb && Vb - Va < this->planner.Vt);
	Pos_Condition = (Sb < this->s - this->vechileSize[0] && target_s > (Vb*(T+0.5) - di));
	TTC_condition = (TTC > 1.0);
	TIV_condition = (TIV > 0.5);
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));

	// Transition 4: From ST_WAIT --> ST_WAIT (Otherwise)	
	Vel_Condition = (Va < Vdes && Vdes < Vb);
	Pos_Condition = true;
	TTC_condition = (TTC < 1.0);
	TIV_condition = (TIV < 0.5);
	TransCond = (Vel_Condition && Pos_Condition && (TTC_condition || TIV_condition));
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));

	// Transition 5: From ST_WAIT --> ST_LANE_CHANGE
	Vel_Condition = (Va < this->vs && this->vs < Vb && Vb < Vdes);
	Pos_Condition = (Sb > this->s + this->vechileSize[0] && target_s < Vb*T-0.5*this->vs + di);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));

	// Transition 6: From ST_WAIT --> ST_WAIT
	Vel_Condition = (Va < this->vs && this->vs < Vb && Vb < Vdes);
	Pos_Condition = (Sb < this->s + this->vechileSize[0]) && (Sb > this->s - this->vechileSize[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));

	// Transition 7: From ST_WAIT --> ST_LANE_CHANGE with vEf = Vmax, Lead Vehicle b after lane change	
	Vel_Condition = (Va < this->vs && this->vs < Vb && Vb < Vdes);
	Pos_Condition = (Sb < this->s - this->vechileSize[0]);
	TTC_condition = (TTC > 1.0);
	TIV_condition = (TIV > 0.5);
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));

	// Transition 8: From ST_WAIT --> ST_WAIT (Otherwise)
	Vel_Condition = (Va < this->vs && this->vs< Vb && Vb < Vdes);
	Pos_Condition = true;
	TTC_condition = (TTC < 1.0);
	TIV_condition = (TIV < 0.5);
	TransCond = (Vel_Condition && Pos_Condition && (TTC_condition || TIV_condition));
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));

	// Transition 9: From ST_WAIT --> ST_WAIT	
	Vel_Condition = (Vb < Va && Va < Vdes);
	Pos_Condition = (Sb >= this->s );
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));


	// Transition 10: From ST_WAIT --> ST_LANE_CHANGE	
	Vel_Condition = (Vb < Va && Va < Vdes);
	Pos_Condition = (Sb < this->s);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));

	// Transition 11: From ST_WAIT --> ST_LANE_CHANGE
	
	Vel_Condition = (Va < Vb && Vb < this->vs && this->vs < Vdes);
	Pos_Condition = (Sb > this->s + this->vechileSize[0] && target_s< (Vb*(T-0.5)+ di));
	TTC_condition = true;
	TIV_condition = (TIV > 0.5);
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));

	// Transition 12: From ST_WAIT --> ST_WAIT
	Vel_Condition = (Va < Vb && Vb < this->vs && this->vs < Vdes);
	Pos_Condition = (Sb < this->s + this->vechileSize[0]) && (Sb > this->s - this->vechileSize[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_WAIT, 0));

	// Transition 13: From ST_WAIT --> ST_LANE_CHANGE	
	Vel_Condition = (Va < Vb && Vb < this->vs && this->vs < Vdes);
	Pos_Condition =  (Sb < this->s - this->vechileSize[0]);
	TTC_condition = true;
	TIV_condition = true;
	TransCond = (Vel_Condition && Pos_Condition && TTC_condition && TIV_condition);
	if (TransCond)
		possible_maneuvers.push_back(make_tuple(ST_LANE_CHANGE, dir));
}

void SmartVehicle::get_vehicle_infront_data(vector<vector<double>> sensor_fusion, vector<double> &VehicleA)
{
	// FIND NEREST VEHICLES IN EGO'S SAME LANE. Vector's format [ id, x, y, vx, vy, s, d]	
	vector<vector<double>> filtered_same_lane = this->planner.filter_sensor_fusion_by_lane(sensor_fusion, this->lane);
	vector<double> nearest_same_lane;
	this->planner.nearestAbs(filtered_same_lane, this->s, nearest_same_lane);
	cout << "6 - Same lane filtered: " << nearest_same_lane.size() << endl;

	double Va = 0.0; // Speed of the car infront of Ego
	double TTC_a = 1000.0; // Time To Collision with vehicle infront of us (Aka Vhicle a).
	double TIV_a = 1000.0; // Inter-Vehicular time (Aka Vhicle a).


	if (!nearest_same_lane.empty() && nearest_same_lane[5] > this->s)
	{
		double Va_x = nearest_same_lane[3];
		double Va_y = nearest_same_lane[4];
		double Sa = nearest_same_lane[5];
		Va = sqrt(Va_x*Va_x + Va_y*Va_y);
		if ( (fabs(this->vs - Va) > 0.001) && (this->vs > Va) )
			TTC_a = ((Sa - this->s - this->vechileSize[0]) / fabs(this->vs - Va));

		TIV_a = ((Sa - this->s - this->vechileSize[0]) / Va);
		cout << "7 - Same lane NOT empty: Va = " << Va << " TTC-a: " << TTC_a << " TIV-a: " << TIV_a << endl;
	}
	VehicleA.clear();
	VehicleA.push_back(Va);
	VehicleA.push_back(TTC_a);
	VehicleA.push_back(TIV_a);
}

void SmartVehicle::generate_best_trajectory(vector<vector<double>> sensor_fusion, vector<double> VehicleA, vector < tuple<State, int > > possible_maneuvers, vector<double> &best_trajectory, State &Ego_new_state)
{
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

	double Va = VehicleA[0];
	double TTC_a = VehicleA[1];
	double TIV_a = VehicleA[2];

	// Let's start by building our Initial Vectors
	vector<double> start_s = planner.start_s;
	vector<double> start_d = planner.start_d;

	double Max_speed = this->planner.road.speed_limit - 0.22; // m/s - always 0.5 mph below the limit 	
	double Max_accel = this->planner.MAX_ACCEL * 0.90; // m/s2 - 

	vector<double> goal_s;
	vector<double> goal_d;
	vector<double> costs; // Vector that will store the BEST cost for each possible state. Each possible state would have a few trajectories to test
	vector<vector<double>> trajectories;
	vector<State> associated_maneuver; // I'll store here the associated maneuver to a trajectory. That way, once i have the "Best" Traj, i can update the Ego State (aka maneuver)

	double target_s; // just the s value..for convenience use
	double Accel_s_needed; //Accel needed to achieve our target_s with our initial conditions and limitations. 
	double T; // Duration of the Maneuver
	double target_d;

	for (auto test_maneuver : possible_maneuvers)
	{
		State test_maneuver_name;
		int dir;
		std::tie(test_maneuver_name, dir) = test_maneuver;
		cout << "test_maneuver_name: " << test_maneuver_name << endl;
		cout << "dir: " << dir << endl;
		target_d = this->planner.lane2d(this->lane + dir); // Our d coordinate target is always the center of the lane
		switch (test_maneuver_name)
		{
			case ST_WAIT: // The only reason we are here is because we got too close to Vehicle A (TTC_a < 1 and TIV_a < 0.5)
			{
				// We need to slow down and maintain a safe distance with vehicle A (in front) until we can pass
				//
				T = TTC_a; // we will plan for the time it will take us to reach A				
				cout << "stop 17" << endl;
				Accel_s_needed = min((Va - this->vs) / T, -Max_accel);
				target_s = this->s + this->vs*T + 0.5*Accel_s_needed*T*T - this->planner.BUFFER_DISTANCE;
				goal_s = { target_s, Va, 0 };
				goal_d = { target_d, 0, 0 };
				break;
			}

			case ST_CRUISE_CONTROL:
			{
				// We need just to mantain a constant speed as close to the MAX legal speed as possible				
				//
				T = (Max_speed) / Max_accel; // we will plan a path for a few sec ahead: Worst case scenario is the time taken to go from 0m/s to max speed WITHOUT breaking the Accel Rule 
				cout << "stop 18" << endl;
				Accel_s_needed = min((Max_speed - this->vs) / T, Max_accel);
				target_s = this->s + this->vs*T + 0.5*Accel_s_needed*T*T;
				goal_s = { target_s, Max_speed, 0 };
				goal_d = { target_d, 0, 0 };

				break;
			}

			case ST_LANE_CHANGE:
			{
				// We will change lane
				// it can be proven that with high enough velocity, the optimal solutions for the problem can be obtained
				// with a Numerical aprox based on:																					
				target_s = 2.4 * this->vs * sqrt(this->planner.road.lane_width / Max_accel);    // Numerical Aproximation of Target S for the change lane maneuver								
				target_s += this->s;
				T = sqrt(3)*pow(this->planner.road.lane_width, 1.5)*sqrt(Max_accel) / (this->vs * this->vs) + (target_s / this->vs); // duration of the maneuver in sec.				
				goal_s = { target_s, Max_speed, 0 };				
				goal_d = { target_d, 0, 0 };

				break;
			}

			default:
			{
				cout << "Strange default" << endl;
				break;
			}
		}

		cout << "19 - Maneuver Params: T = " << T << " target_S = " << goal_s[0] << " target_d = " << goal_d[0] << " target Speed = " << goal_s[1] << " target Acc = " << goal_s[2] << endl;

		vector<vector<double>> trajectories_per_state = this->planner.PTG(start_s, start_d, goal_s, goal_d, T);
		vector<double> costs_per_state;

		cout << "stop - PTG returned all trajectories for a single state. Num Traj per State: " << trajectories_per_state.size() << endl;

		for (auto trajectory : trajectories_per_state)
		{
			double cost = this->planner.calculate_cost(trajectory, sensor_fusion, goal_s, goal_d, T, this->vechileSize[1]);
			costs_per_state.push_back(cost);
			//cout << "Cost: " << cost << endl;
		}

		cout << "Calc Cost Finished for Trajecs for single state. Costs Added for this state:" << costs_per_state.size() << endl;
		// Until this point we have all the costs for This specific State (with its multiple trajectories)
		vector<double>::iterator it = min_element(costs_per_state.begin(), costs_per_state.end());
		int BestCostIdx = std::distance(costs_per_state.begin(), it);
		cout << "BestCostIdx: " << BestCostIdx << endl;
		costs.push_back(costs_per_state[BestCostIdx]);
		cout << "Best Cost Added to Costs: " << costs_per_state[BestCostIdx] << endl;
		trajectories.push_back(trajectories_per_state[BestCostIdx]);
		associated_maneuver.push_back(test_maneuver_name);
		cout << "Best Traj Added to Traj" << endl;
	}

	cout << "Num Best Traj:" << trajectories.size() << endl;
	cout << "Num Best cost:" << costs.size() << endl;
	// No we should have only 1 (best) cost per State and with a correlating vector of trajectories
	// Let's now calculate the very Best trajectory
	vector<double>::iterator it = min_element(costs.begin(), costs.end());
	int BestCostIdx = std::distance(costs.begin(), it);
	cout << "BestCostIdx: " << BestCostIdx << endl;
	cout << "Winner Traj Time Len: " << trajectories[BestCostIdx][12] << endl;
	
	best_trajectory = trajectories[BestCostIdx];
	Ego_new_state = associated_maneuver[BestCostIdx];
}

void SmartVehicle::smooth_trajectory(vector<double> trajectory, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	cout << "********* TRAJECTORY SMOOTHER ************" << endl;
	// ********* TRAJECTORY SMOOTHER ************
	
	vector<double> new_path_x;
	vector<double> new_path_y;
	
	double last_x;
	double last_y;
	double last_yaw;
	double x_prev;
	double y_prev;

	/*
	We need to know what's actually the Ego motion state. We know it's location ans Speed and yaw but we are missing it's accel.
	We need the accel to make sure that once we "stich" the next trajectory, it will transtion smoothly. We need to locate the next
	2 points in such a way that speed and accel transtion smoothly.
	*/
	int prev_path_size = prev_x_vals.size();
	cout << "prev_path_size " << prev_path_size << endl;
	if (prev_path_size < 2)
	{		
		cout << "point 1 "  << endl;
		
		last_x = this->x;
		last_y = this->y;		
		last_yaw = deg2rad(this->yaw);

		x_prev = last_x - cos(last_yaw);
		y_prev = last_y - sin(last_yaw);

		
		
	}
	else
	{
		cout << "going through x1-x2 " << endl;

		int start_idx = prev_path_size;
		last_x = prev_x_vals[start_idx - 1];
		last_y = prev_y_vals[start_idx - 1];

		x_prev = prev_x_vals[start_idx - 2];
		y_prev = prev_y_vals[start_idx - 2];

		last_yaw = atan2((last_y - y_prev),(last_x - x_prev));		
	}
	//add previous
	new_path_x.push_back(x_prev);
	new_path_y.push_back(y_prev);
	cout << "1 x= " << x_prev << " y= " << y_prev << endl;

	//add actual
	new_path_x.push_back(last_x);
	new_path_y.push_back(last_y);
	cout << "2 x= " << last_x << " y= " << last_y << endl;

	
	// Let's extract the coefficient from the trajectory that we'll need later
	vector<double> s_coeff;
	std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

	vector<double> d_coeff;
	std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

	double T = trajectory[12]; // time duration of the Best Trajectory Generated
	
	int num_anchor_points = 10;
	double dt = T / num_anchor_points; // (prev_path_size / 3)); // let's add 1/3 of the new JMT trajectory	

	for (int i = 1; i <= num_anchor_points; i++)
	{
		double at_time = i*dt;
		double s = evaluate(s_coeff, at_time);
		double d = evaluate(d_coeff, at_time);
		cout << i << " s= " << s << " d= " << d << endl;
		vector<double> next_wp = this->planner.getXY(s, d, this->planner.road.map_waypoints_s, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);		
		new_path_x.push_back(next_wp[0]);
		new_path_y.push_back(next_wp[1]);
		cout << i << " x= " << next_wp[0] << " y= " << next_wp[1] << endl;
	}

	
	int new_path_len = new_path_x.size();
	for (size_t i = 0; i < new_path_len; i++)
	{
		double shift_x = new_path_x[i] - last_x;
		double shift_y = new_path_y[i] - last_y;
		
		new_path_x[i] = (shift_x * cos(0 - last_yaw) - shift_y * sin(0 - last_yaw));
		new_path_y[i] = (shift_x * sin(0 - last_yaw) + shift_y * cos(0 - last_yaw));
	}

	
	// Now we create a smooth spline
	cout << "Creating Spline. new_path_points (x): " << new_path_len << " (y): " << new_path_y.size() << endl;
	tk::spline spl;
	spl.set_points(new_path_x, new_path_y);

	vector<double> goal_s;
	std::copy(trajectory.begin() + 13, trajectory.begin() + 16, std::back_inserter(goal_s));

	double target_x = new_path_x[new_path_len - 1];// goal_s[0];
	double target_y = spl(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);
	double target_vs = goal_s[1];

	double N = target_dist / (target_vs * this->planner.dt);
	double x_final = 0;
	double y_final;
	cout << "Spline divided in (N): " << N << endl;

	for (size_t i = 0; i < 100; i++)
	{
		x_final += target_x / N;
		y_final = spl(x_final);

		double x_point = last_x + (x_final * cos(last_yaw) - y_final * sin(last_yaw));
		double y_point = last_y + (x_final * sin(last_yaw) + y_final * sin(last_yaw));
		
		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}
		
	cout << "Sending next Path len = " << next_x_vals.size() << endl;
}

void SmartVehicle::cruise_control(vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	cout << "********* TRAJECTORY SMOOTHER ************" << endl;
	// ********* TRAJECTORY SMOOTHER ************

	vector<double> new_path_x;
	vector<double> new_path_y;

	double last_x;
	double last_y;
	double last_yaw;
	double x_prev;
	double y_prev;

	/*
	We need to know what's actually the Ego motion state. We know it's location ans Speed and yaw but we are missing it's accel.
	We need the accel to make sure that once we "stich" the next trajectory, it will transtion smoothly. We need to locate the next
	2 points in such a way that speed and accel transtion smoothly.
	*/
	int prev_path_size = prev_x_vals.size();
	cout << "prev_path_size " << prev_path_size << endl;
	if (prev_path_size < 2)
	{
		cout << "point 1 " << endl;

		last_x = this->x;
		last_y = this->y;
		last_yaw = deg2rad(this->yaw);

		x_prev = last_x - cos(last_yaw);
		y_prev = last_y - sin(last_yaw);



	}
	else
	{
		cout << "going through x1-x2 " << endl;

		int start_idx = prev_path_size;
		last_x = prev_x_vals[start_idx - 1];
		last_y = prev_y_vals[start_idx - 1];

		x_prev = prev_x_vals[start_idx - 2];
		y_prev = prev_y_vals[start_idx - 2];

		last_yaw = atan2((last_y - y_prev), (last_x - x_prev));
	}
	//add previous
	new_path_x.push_back(x_prev);
	new_path_y.push_back(y_prev);
	cout << "1 x= " << x_prev << " y= " << y_prev << endl;

	//add actual
	new_path_x.push_back(last_x);
	new_path_y.push_back(last_y);
	cout << "2 x= " << last_x << " y= " << last_y << endl;

	/*
	// Let's extract the coefficient from the trajectory that we'll need later
	vector<double> s_coeff;
	std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

	vector<double> d_coeff;
	std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

	double T = trajectory[12]; // time duration of the Best Trajectory Generated

	int num_anchor_points = 10;
	double dt = T / num_anchor_points; // (prev_path_size / 3)); // let's add 1/3 of the new JMT trajectory	

	for (int i = 1; i <= num_anchor_points; i++)
	{
		double at_time = i*dt;
		double s = evaluate(s_coeff, at_time);
		double d = evaluate(d_coeff, at_time);
		cout << i << " s= " << s << " d= " << d << endl;
		vector<double> next_wp = this->planner.getXY(s, d, this->planner.road.map_waypoints_s, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);
		new_path_x.push_back(next_wp[0]);
		new_path_y.push_back(next_wp[1]);
		cout << i << " x= " << next_wp[0] << " y= " << next_wp[1] << endl;
	}

	*/

	for (int i = 1; i < 4; i++)
	{
		vector<double> next_wp = this->planner.getXY(this->s + 30 * i, this->planner.lane2d(this->lane), this->planner.road.map_waypoints_s, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);
		new_path_x.push_back(next_wp[0]);
		new_path_y.push_back(next_wp[1]);
	}


	int new_path_len = new_path_x.size();
	for (size_t i = 0; i < new_path_len; i++)
	{
		double shift_x = new_path_x[i] - last_x;
		double shift_y = new_path_y[i] - last_y;

		new_path_x[i] = (shift_x * cos(0 - last_yaw) - shift_y * sin(0 - last_yaw));
		new_path_y[i] = (shift_x * sin(0 - last_yaw) + shift_y * cos(0 - last_yaw));
	}


	// Now we create a smooth spline
	cout << "Creating Spline. new_path_points (x): " << new_path_len << " (y): " << new_path_y.size() << endl;
	tk::spline spl;
	spl.set_points(new_path_x, new_path_y);
	
	for (size_t i = 0; i < prev_x_vals.size(); i++)
	{
		next_x_vals.push_back(prev_x_vals[i]);
		next_y_vals.push_back(prev_y_vals[i]);
	}

	double target_x = 30;// new_path_x[new_path_len - 1];// goal_s[0];
	double target_y = spl(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);
	double target_vs = this->planner.road.speed_limit - mph2ms(0.25) ;

	double N = target_dist / (target_vs * this->planner.dt);
	double x_final = 0;
	double y_final;
	cout << "Spline divided in (N): " << N << endl;

	for (size_t i = 1; i < 50-prev_x_vals.size(); i++)
	{
		x_final += target_x / N;
		y_final = spl(x_final);

		double x_point = last_x + (x_final * cos(last_yaw) - y_final * sin(last_yaw));
		double y_point = last_y + (x_final * sin(last_yaw) + y_final * cos(last_yaw));

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}

	cout << "Sending next Path len = " << next_x_vals.size() << endl;
}

void SmartVehicle::cruise_control2(vector<double> trajectory, vector<double> prev_x_vals, vector<double> prev_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	cout << "********* TRAJECTORY SMOOTHER ************" << endl;
	// ********* TRAJECTORY SMOOTHER ************
	cout << "new Path len = " << planner.new_path_x.size() << endl;

	
	int new_path_len = this->planner.new_path_x.size();
	double last_x = this->planner.new_path_x[new_path_len - 1];
	double last_y = this->planner.new_path_y[new_path_len - 1];
	double ref_theta = planner.ref_theta;
	cout << "last point from prev x= " << last_x << " y= " << last_y << endl;
	
		
	// Let's extract the coefficient from the trajectory that we'll need later
	vector<double> s_coeff;
	std::copy(trajectory.begin(), trajectory.begin() + 6, std::back_inserter(s_coeff));

	vector<double> d_coeff;
	std::copy(trajectory.begin() + 6, trajectory.begin() + 12, std::back_inserter(d_coeff));

	double T = trajectory[12]; // time duration of the Best Trajectory Generated

	/*
	 Now we need to find the closest point on this new Traj to the "prev_path" point that we want to start building upon. i.e. this
	 point is stored already on planner.new_path_x and new_path_y
	*/

	//vector<double> s_d = planner.getFrenet(last_x, last_y, ref_theta, planner.road.map_waypoints_x, planner.road.map_waypoints_y);
	double t_start = 0.0;
	double target_s = planner.start_s[0];
	
	//double target_d = s_d[1];
	double s = this->s;
	cout << "target_s= " << target_s << " ini s= " << s << endl;
	while (s < target_s)
	{
		t_start += planner.dt;
		s = evaluate(s_coeff, t_start);
		cout << "s= " << s << endl;
	}
	
	
	/*
	int use_prev_points = this->planner.time_offset_from_prev_path / this->planner.dt;
	int prev_path_size = prev_x_vals.size();

	t_start = ((use_prev_points > prev_path_size)? prev_path_size: use_prev_points) * this->planner.dt;
	*/

	cout << "t-start= " << t_start << endl;

	for (double at_time = t_start; at_time <= T; at_time += planner.dt)
		{		
		double s = evaluate(s_coeff, at_time);
		double d = evaluate(d_coeff, at_time);
		//cout << at_time << " s= " << s << " d= " << d << endl;
		vector<double> next_wp = this->planner.getXY(s, d, this->planner.road.map_waypoints_s, this->planner.road.map_waypoints_x, this->planner.road.map_waypoints_y);
		planner.new_path_x.push_back(next_wp[0]);
		planner.new_path_y.push_back(next_wp[1]);
		//cout << at_time << " x= " << next_wp[0] << " y= " << next_wp[1] << endl;
		}
	
	
	for (size_t i = 0; i < planner.new_path_x.size(); i++)
	{
		next_x_vals.push_back(planner.new_path_x[i]);
		next_y_vals.push_back(planner.new_path_y[i]);
	}

	cout << "Sending next Path len = " << next_x_vals.size() << endl;
}