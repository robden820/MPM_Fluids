#pragma once

#include <glm/glm.hpp>

namespace Spline
{
	double BezierCurvature(const glm::dvec2& P0, const glm::dvec2& P1, const glm::dvec2& P2, const glm::dvec2& P3, const double t = 0.5);
	double CatmullRomCurvature(const glm::dvec2& P0, const glm::dvec2& P1, const glm::dvec2& P2, const glm::dvec2& P3, const double t = 0.5, const double tension = 0.5);
	double BSplineCurvature(const glm::dvec2& P0, const glm::dvec2& P1, const glm::dvec2& P2, const glm::dvec2& P3, const double t = 0.5);

	double Curvature(const double dx, const double dy, const double dxdx, const double dydy);
}