#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "SimulationData.h"

// Used for loading the information of a simulation state from a file to run the simulation from that configuration.
class StateLoader
{
public:

	StateLoader(const std::array<char, 256>& scenePath, SimulationData& simData, int& iterations);
	~StateLoader() = default;

private:
	void ClearSimData(SimulationData& simData);
};