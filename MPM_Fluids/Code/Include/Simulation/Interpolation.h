#pragma once

#include <glm/glm.hpp>


namespace Interpolation
{
	// Interpolate of the non-normalized interval [x,y]. Interval size is (y-x)
	double Cubic(const double (&points)[4], double intervalSize, double alpha);
	double Bicubic(const double(&points)[4][4], double intervalSize, const glm::dvec2& alpha);
	double Tricubic(const double(&points)[4][4][4], double intervalSize, const glm::dvec3& alpha);

	double FiniteDifference(double point0, double point1, double intervalSize);                 // Returns tangent gradient at point 0.
	double CardinalSpline(double point0, double point2, double intervalSize, double tension);   // Returns tangent gradient at point 1.
	double CatmullRomSpline(double point0, double point2, double intervalSize);                 // Returns tangent gradient at point 1.

	double Linear(const double input);
	double GradLinear(const double input);

	double QuadBSpline(const double input); //TODO description.
	double CubicBSpline(const double input);

	double GradQuadBSpline(const double input);
	double GradCubicBSpline(const double input);

	const double PRECISION = 0.0000000001;
}