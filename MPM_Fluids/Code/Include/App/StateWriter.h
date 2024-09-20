#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "SimulationData.h"

enum WriteType
{
	eSIMULATION = 0,
	eRENDER = 1,
	eMETA = 2
};

// Used for writing out all the information of a simulation state to a file that can be reloaded to run the simulation.
class StateWriter
{
public:

	StateWriter(const int iterations, const char* fileName, const SimulationData& simData, WriteType writeType);
	~StateWriter() = default;

private:

	void WriteSimulation(std::ofstream& outputFile, const int iterations, const SimulationData& simData);
	void WriteRender(std::ofstream& outputFile, const int iterations, const SimulationData& simData);
	void WriteMeta(std::ofstream& outputFile, const int iterations, const SimulationData& simData);
};
