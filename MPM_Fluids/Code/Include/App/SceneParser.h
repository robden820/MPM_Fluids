#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "SimulationData.h"

class SceneParser
{

public:

	SceneParser(const char* scenePath, SimulationData& simData);
	~SceneParser() = default;
};