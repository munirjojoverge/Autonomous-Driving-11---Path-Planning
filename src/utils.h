/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <math.h>
#include <stdlib.h>     /* div, div_t */

using namespace std;

namespace utils
{
	const double pi = 3.14159265;

	double evaluate(vector<double> JMT_coefficients, double t);

	vector<double> differentiate(vector<double> coefficients);

	vector<double> evaluate_f_and_N_derivatives(vector<double>coefficients, double t, int N = 3);

	double logistic(double x);

	void solve_quadratic(double a, double b, double c, double &x1, double &x2);

	double mph2ms(double mph);

	double deg2rad(double x);

	double rad2deg(double x);

}
#endif
