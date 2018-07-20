#pragma once

#include <string>
#include <iostream>
#include <Eigen/Eigen>

using namespace std;

struct Sensor
{
	std::string id,
		label;
	int resolution_width,
		resolution_height;
	double pixel_width,
		pixel_height,
		focal_length;

	// calibration
	int calibration_res_width,
		calibration_res_height;

	double f, fx, fy, cx, cy,
		skew = 0,
		k1 = 0,
		k2 = 0,
		k3 = 0,
		k4 = 0,
		p1 = 0,
		p2 = 0,
		p3 = 0,
		p4 = 0,
		b1 = 0,
		b2 = 0;

	template <typename Derived, typename Derived2>
	void projectPoint(const Eigen::MatrixBase<Derived> &point, Eigen::MatrixBase<Derived2> &sensorCoords)
	{
		EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<Derived>)
		EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<Derived2>)
		
		double x = point[0] / point[2];
		double y = point[1] / point[2];
		double r = std::sqrt(x*x + y*y);
		double r2 = r*r; // r^2
		double r4 = r2*r2; // r^4
		double r6 = r4*r2; // r^6
		double r8 = r4*r4; // r^6
				
		double a = (1 + k1*r2 + k2*r4 + k3*r6 + k4*r8);
		double b = (1 + p3*r2 + p4*r4);

		double corr_x = (x * a) + b * (p1 * (r2 + 2 * x*x) + (2 * p2 * x * y));
		double corr_y = (y * a) + b * (p2 * (r2 + 2 * y*y) + (2 * p1 * x * y));

		double u;
		double v;
		double fxcalc;
		double fycalc;
		double xline;
		double yline;

		if (fx == 0 || fy == 0){			
			xline = x * a + ((p1 * (r2 + 2 * (x*x)) + (2*p2*x*y)) * b);
			yline = y * a + ((p2 * (r2 + 2 * (y*y)) + (2*p1*x*y)) * b);
		
			u = resolution_width * 0.5 + (cx + xline*f) + (xline*b1) + (yline * b2);
			v = resolution_height * 0.5 + cy + yline*f;

			sensorCoords[0] = u;
			sensorCoords[1] = v;
		} else {
			u = (corr_x * fx) + (corr_x * b1) + (corr_y * b2);
			v = (corr_y * fy);
			sensorCoords[0] = cx + u;
			sensorCoords[1] = cy + v;
		}
	
	}

	template <typename Derived>
	inline Eigen::Vector2f projectPoint(const Eigen::MatrixBase<Derived> &point)
	{
		EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<Derived>)

		Eigen::Vector2f sensorCoords;
		projectPoint(point, sensorCoords);
		return sensorCoords;
	}


};