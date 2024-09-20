#include "Spline.h"

double Spline::BezierCurvature(const glm::dvec2& P0, const glm::dvec2& P1, const glm::dvec2& P2, const glm::dvec2& P3, const double t)
{
	// First derivatives
	double dx = P0.x * (-3.0 * (1.0 - t) * (1.0 - t)) +
				P1.x * (9.0 * t * t - 12.0 * t + 3.0) +
				P2.x * (-9.0 * t * t + 6.0 * t) +
				P3.x * (3.0 * t * t);

	double dy = P0.y * (-3.0 * (1.0 - t) * (1.0 - t)) +
				P1.y * (9.0 * t * t - 12.0 * t + 3.0) +
				P2.y * (-9.0 * t * t + 6.0 * t) +
				P3.y * (3.0 * t * t);


	// Second derivative
	double dxdx = P0.x * (6.0 - 6.0 * t) +
				  P1.x * (18.0 * t - 12.0) +
				  P2.x * (-18.0 * t + 6.0) +
				  P3.x * (6.0 * t);

	double dydy = P0.y * (6.0 - 6.0 * t) +
				  P1.y * (18.0 * t - 12.0) +
				  P2.y * (-18.0 * t + 6.0) +
				  P3.y * (6.0 * t);

	return Curvature(dx, dy, dxdx, dydy);
}

double Spline::CatmullRomCurvature(const glm::dvec2& P0, const glm::dvec2& P1, const glm::dvec2& P2, const glm::dvec2& P3, const double t, const double tension)
{
	// First derivatives
	double dx = P0.x * (-tension + 4.0 * tension * t - 3.0 * t * t * tension) +
				P1.x * (2 * t * (tension - 3.0) + 3.0 * t * t * (2.0 - tension)) +
				P2.x * (tension + 2.0 * t * (3.0 - 2.0 * tension) + 3.0 * t * t * (tension - 2.0)) +
				P3.x * (-2.0 * t * tension + 3.0 * t * t * tension);

	double dy = P0.y * (-tension + 4.0 * tension * t - 3.0 * t * t * tension) +
				P1.y * (2 * t * (tension - 3.0) + 3.0 * t * t * (2.0 - tension)) +
				P2.y * (tension + 2.0 * t * (3.0 - 2.0 * tension) + 3.0 * t * t * (tension - 2.0)) +
				P3.y * (-2.0 * t * tension + 3.0 * t * t * tension);


	// Second derivative
	double dxdx = P0.x * (4.0 * tension - 6.0 * t * tension) +
				  P1.x * (2.0 * (tension - 3.0) + 6.0 * t * (2.0 - tension)) +
				  P2.x * (2.0 * (3.0 - 2.0 * tension) + 6.0 * t * (tension - 2.0)) +
				  P3.x * (-2.0 * tension + 6.0 * t * tension);

	double dydy = P0.y * (4.0 * tension - 6.0 * t * tension) +
				  P1.y * (2.0 * (tension - 3.0) + 6.0 * t * (2.0 - tension)) +
				  P2.y * (2.0 * (3.0 - 2.0 * tension) + 6.0 * t * (tension - 2.0)) +
				  P3.y * (-2.0 * tension + 6.0 * t * tension);

	return Curvature(dx, dy, dxdx, dydy);
}

double Spline::BSplineCurvature(const glm::dvec2& P0, const glm::dvec2& P1, const glm::dvec2& P2, const glm::dvec2& P3, const double t)
{
	// First derivatives
	double dx = (P0.x * (-3.0 + 6.0 * t - 3.0 * t * t) +
				 P1.x * (-12.0 * t + 9.0 * t * t) +
				 P2.x * (3.0 + 6.0 * t - 9.0 * t * t) +
				 P3.x * (3.0 * t * t)) / 6.0;

	double dy = (P0.y * (-3.0 + 6.0 * t - 3.0 * t * t) +
				 P1.y * (-12.0 * t + 9.0 * t * t) +
				 P2.y * (3.0 + 6.0 * t - 9.0 * t * t) +
				 P3.y * (3.0 * t * t)) / 6.0;


	// Second derivative
	double dxdx = P0.x * (1.0 - t) +
				  P1.x * (3.0 * t - 2.0) +
				  P2.x * (1.0 - 3.0 * t) +
				  P3.x * t;

	double dydy = P0.y * (1.0 - t) +
				  P1.y * (3.0 * t - 2.0) +
				  P2.y * (1.0 - 3.0 * t) +
				  P3.y * t;

	return Curvature(dx, dy, dxdx, dydy);
}

double Spline::Curvature(const double dx, const double dy, const double dxdx, const double dydy)
{
	return abs(dx * dydy - dy * dxdx) / pow(dx * dx + dy * dy, 1.5);
}