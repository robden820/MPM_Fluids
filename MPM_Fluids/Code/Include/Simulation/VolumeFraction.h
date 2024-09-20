#pragma once

#include <glm/glm.hpp>

namespace VolumeFraction
{
	double LineSquareVolume(const glm::vec2& l0, const glm::vec2& l1, const double lineDiameter, const glm::vec3& sqrPos, const double halfEdgeLength); // Use vec3 because of how we store simulation data.
	double LineCubeVolume(const glm::vec3& l0, const glm::vec3& l1, const double lineDiameter, const glm::vec3& cubePos, const double halfEdgeLength);
}