/**********************************************
* Self-Driving Car Nano-degree - Udacity
*  Created on: August 12, 2017
*      Author: Munir Jojo-Verge
**********************************************/

#include <iostream>
#include "utils.h"

using namespace std;

namespace utils
{
	double evaluate(vector<double> JMT_coefficients, double t)
	{
		//cout << "evaluate - Coeff: " << JMT_coefficients.size() << endl;
		double total = 0.0;
		for (size_t i = 0; i < JMT_coefficients.size(); i++)
		{			
			total += JMT_coefficients[i] * pow(t, i);
			//cout << "evaluate - i = " << i << " coeff: " << JMT_coefficients[i] << " total: " << total << endl;
		}
		return total;
	}

	vector<double> differentiate(vector<double> coefficients)
	{
		/*
		Calculates the derivative of a polynomial and returns
		the corresponding coefficients.
		*/
		vector<double> derivative_coeff;
		for (size_t i = 1; i < coefficients.size(); i++)
		{
			derivative_coeff.push_back(coefficients[i] * i);
			//cout << "diff - Coeff[" << i-1 << "] = " << derivative_coeff[i-1] << endl;
		}

		return derivative_coeff;
	}

	vector<double> evaluate_f_and_N_derivatives(vector<double>coefficients, double t, int N)
	{
		vector<double> values;
		values.push_back(evaluate(coefficients, t));
		for (int i = 1; i <= N; i++)
		{
			vector<double> d_coeff = differentiate(coefficients);
			values.push_back(evaluate(d_coeff, t));
		}
		return values;
	}

	double logistic(double x)
	{
		/*
		A function that returns a value between 0 and 1 for x in the
		range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].

		Useful for cost functions.
		*/
		//cout << "Logistic In. x = " << x << endl;

		return 2.0 / (1 + exp(-x)) - 1.0;
	}

	void solve_quadratic(double a, double b, double c, double &x1, double &x2)
	{
		double discriminant = b*b - 4 * a*c;

		if (discriminant > 0) 
		{
			x1 = (-b + sqrt(discriminant)) / (2 * a);
			x2 = (-b - sqrt(discriminant)) / (2 * a);
			/*
			cout << "Roots are real and different." << endl;
			cout << "x1 = " << x1 << endl;
			cout << "x2 = " << x2 << endl;
			*/
		}

		else if (discriminant == 0) 
		{
			x1 = (-b + sqrt(discriminant)) / (2 * a);
			x2 = x1;
			/*
			cout << "Roots are real and same." << endl;			
			cout << "x1 = x2 =" << x1 << endl;
			*/
		}

		else {
			/*
			realPart = -b / (2 * a);
			imaginaryPart = sqrt(-discriminant) / (2 * a);
			cout << "Roots are complex and different." << endl;
			cout << "x1 = " << realPart << "+" << imaginaryPart << "i" << endl;
			cout << "x2 = " << realPart << "-" << imaginaryPart << "i" << endl;
			*/
		}
	}

	double mph2ms(double mph) 
	{ 
		return mph * 0.44704;
	}
		
	double deg2rad(double x) { return x * pi / 180; }
	double rad2deg(double x) { return x * 180 / pi; }

}