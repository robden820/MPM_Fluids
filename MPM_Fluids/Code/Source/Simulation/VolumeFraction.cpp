#include "VolumeFraction.h"

#include <array>
#include <algorithm>

double VolumeFraction::LineSquareVolume(const glm::vec2& l0, const glm::vec2& l1, const double lineDiameter, const glm::vec3& sqrPos, const double halfEdgeLength)
{
	double xMin = sqrPos.x - halfEdgeLength;
	double xMax = sqrPos.x + halfEdgeLength;
	double yMin = sqrPos.y - halfEdgeLength;
	double yMax = sqrPos.y + halfEdgeLength;

	glm::vec2 direction = l1 - l0;

	double tXMin = 0.0, tYMin = 0.0;
	double tXMax = 1.0, tYMax = 1.0;

	if (direction.x != 0.0)
	{
		double invX = 1.0 / direction.x;

		tXMin = (xMin - l0.x) * invX;
		tXMax = (xMax - l0.x) * invX;
	}

	if (direction.y != 0.0)
	{
		double invY = 1.0 / direction.y;

		tYMin = (yMin - l0.y) * invY;
		tYMax = (yMax - l0.y) * invY;
	}

	std::array<double, 4> t = {tXMin, tYMin, tXMax, tYMax };
	std::sort(t.begin(), t.end());

	double length = (t[2] - t[1]) * glm::length(direction);

	return length * lineDiameter;
}

double VolumeFraction::LineCubeVolume(const glm::vec3& l0, const glm::vec3& l1, const double lineDiameter, const glm::vec3& cubePos, const double halfEdgeLength)
{
	return 0.0;
}