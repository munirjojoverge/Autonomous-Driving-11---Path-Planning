/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#include <algorithm>    // std::max
#include "Road_Vehicle.h"
#include "Planner_Constants.h"

using namespace std;
using namespace constants;

Road_Vehicle::Road_Vehicle()
{
	this->id = -1;
}

Road_Vehicle::Road_Vehicle(int id, double x, double y, double vx, double vy, double s, double d)
{
	this->id = id;
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy = vy;
	this->s = s;
	this->d = d;
	this->speed = sqrt(vx*vx + vy*vy);	
}

Road_Vehicle::~Road_Vehicle()
{
}

Frenet_Position Road_Vehicle::predict_frenet_at(double time)
{
	/*
	For this NON-EGO vehicles on the road we will assume:
		1) They move with constant acceleration = 0 (in both s and d dimentions);
		2) We will also assume that their d_dot (frent speed on the d dimension) is also zero and the vehicle only moves on the s direction at s_dot = this->speed

	*/
	Frenet_Position pos;
	pos.s = this->s + this->speed*time;
	pos.d = this->d;

	return pos;
}

bool Road_Vehicle::empty()
{
	return (this->id == -1);
}

void Road_Vehicle::set_TIV(double Ego_s)
{	
	double TIV = MIN_TIV + 0.01; // Inter-Vehicular Time with vehicle Ego.	
	if (fabs(this->speed) > 0.001)
		TIV = ((this->s - Ego_s - VEHICLE_SIZE[0]) / this->speed);

	this->TIV = TIV;
}

void Road_Vehicle::set_TTC(double Ego_s, double Ego_speed)
{	
	double TTC = MIN_TTC + 0.01; // Time To Collision with Ego	
	if ((fabs(Ego_speed - this->speed) > 0.001))
			TTC = max(TTC, ((this->s - Ego_s - VEHICLE_SIZE[0]) / (Ego_speed - this->speed)));

	this->TTC = TTC;
}

