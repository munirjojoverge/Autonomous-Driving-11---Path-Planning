/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#ifndef OTHER_VEHICLE_H
#define OTHER_VEHICLE_H

#include <math.h>
#include <vector>

using namespace std;

struct Frenet_Position
{
	double s;
	double d;
};

class Road_Vehicle
{
public:
	// METHODS
	Road_Vehicle();
	Road_Vehicle(int id, double x, double y, double vx, double vy, double s, double d);
	~Road_Vehicle();
	Frenet_Position predict_frenet_at(double time);
	bool empty();
	void set_TIV(double Ego_s);
	void set_TTC(double Ego_s, double Ego_speed);

	// PROPERTIES
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	double speed;
	double TTC;
	double TIV;
	
private:

};

#endif