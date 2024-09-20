
#include "Interpolation.h"

#include <iostream>

double Interpolation::Cubic(const double(&points)[4], double intervalSize, double alpha)
{
	double alphaSqr = alpha * alpha;
	double alphaCube = alphaSqr * alpha;

	double h00 = 2.0 * alphaCube - 3.0 * alphaSqr + 1.0;
	double h10 = alphaCube - 2.0 * alphaSqr + alpha;
	double h01 = -2.0 * alphaCube + 3.0 * alphaSqr;
	double h11 = alphaCube - alphaSqr;

	double m1 = CardinalSpline(points[0], points[2], intervalSize, 0.5);
	double m2 = CardinalSpline(points[1], points[3], intervalSize, 0.5);

	return (h00 * points[1]) + (h10 * intervalSize * m1) + (h01 * points[2]) + (h11 * intervalSize * m2);
}

double Interpolation::Bicubic(const double(&points)[4][4], double intervalSize, const glm::dvec2& alpha)
{
	double p0 = Cubic(points[0], intervalSize, alpha.x);
	double p1 = Cubic(points[1], intervalSize, alpha.x);
	double p2 = Cubic(points[2], intervalSize, alpha.x);
	double p3 = Cubic(points[3], intervalSize, alpha.x);

	double cubicPoints[] = { p0, p1, p2, p3 };

	return Cubic(cubicPoints, intervalSize, alpha.y);
}

double Interpolation::Tricubic(const double(&points)[4][4][4], double intervalSize, const glm::dvec3& alpha)
{
	// TODO: check this function.
	double p0 = Bicubic(points[0], intervalSize, glm::vec2(alpha.x, alpha.y));
	double p1 = Bicubic(points[1], intervalSize, glm::vec2(alpha.x, alpha.y));
	double p2 = Bicubic(points[2], intervalSize, glm::vec2(alpha.x, alpha.y));
	double p3 = Bicubic(points[3], intervalSize, glm::vec2(alpha.x, alpha.y));

	double cubicPoints[] = { p0, p1, p2, p3 };

	return Cubic(cubicPoints, intervalSize, alpha.z);
}

double Interpolation::FiniteDifference(double point0, double point1, double intervalSize)
{
	double p0p1 = point1 - point0;
	double p1p0 = point0 - point1;

	return ((p0p1 / intervalSize) - (p1p0 / intervalSize)) * 0.5f;
}

double Interpolation::CardinalSpline(double point0, double point2, double intervalSize, double tension)
{
	if (tension < 0.0 || tension > 1.0)
	{
		// Tension must be in range [0,1].
		std::cout << "WARNING: Tension value for cardinal spline not in range [0,1] \n";
		return 0.0;
	}

	double p0p2 = point2 - point0;

	return (1.0 - tension) * (p0p2 / (2.0 * intervalSize));
}

double Interpolation::CatmullRomSpline(double point0, double point2, double intervalSize)
{
	// Equivalent to cardinal spline with tension 0.5.
	return CardinalSpline(point0, point2, intervalSize, 0.5);
}

double Interpolation::Linear(const double input)
{
	double x = abs(input);
	 
	if (x < 1.0)
	{
		return 1.0 - x;
	}

	return 0.0;
}

double Interpolation::GradLinear(const double input)
{
	if (input < 0.0 && input > -1.0)
	{
		return 1.0;
	}
	if (input > 0.0 && input < 1.0)
	{
		return -1.0;
	}

	return 0.0;
}

double Interpolation::QuadBSpline(const double input)
{
	double x = abs(input);
	double result = 0.0;

	if (x < 0.5)
	{
		result =  0.75 - (x * x);
	}
	else if (x < 1.5)
	{
		result = 0.5 * (1.5 - x) * (1.5 - x);
	}

	return result;
}

double Interpolation::GradQuadBSpline(const double input)
{
	double result = 0.0;

	if (input > -0.5 && input < 0.5)
	{
		result = -2.0 * input;
	}
	else if (input >= 0.5 && input < 1.5)
	{
		result = input - 1.5;
	}
	else if (input <= -0.5 && input > -1.5)
	{
		result = input + 1.5;
	}

	return result;
}

double Interpolation::CubicBSpline(const double input)
{
	double x = abs(input);
	double result = 0.0;

	if (x < 1.0)
	{
		result = (0.5 * x * x * x) - (x * x) + (2.0 / 3.0);
	}
	else if (x < 2.0)
	{
		result = ((-1.0 / 6.0) * x * x * x) + (x * x) - (2 * x) + (4.0 / 3.0);
	}

	return result;
}

double Interpolation::GradCubicBSpline(const double input)
{
	double result = 0.0;

	if (input >= 0.0 && input < 1.0)
	{
		result = input * (1.5 * input - 2.0);
	}
	else if (input < 0.0 && input > -1.0)
	{
		result = input * (-1.5 * input - 2.0);
	}
	else if (input >= 1.0 && input < 2.0)
	{
		result = -0.5 * (input - 2.0) * (input - 2.0);
	}
	else if (input <= -1.0 && input > -2.0)
	{
		result = 0.5 * (input + 2.0) * (input + 2.0);
	}

	return result;
}