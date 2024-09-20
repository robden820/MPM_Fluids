#include "StateLoader.h"

#include <glm/glm.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <direct.h>

// -- START: Copied code --
// FROM https://stackoverflow.com/questions/236129/how-do-i-iterate-over-the-words-of-a-string

template <typename Out>
void split(const std::string& s, char delim, Out result) {
	std::istringstream iss(s);
	std::string item;
	while (std::getline(iss, item, delim)) {
		*result++ = item;
	}
}

std::vector<std::string> split(const std::string& s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, std::back_inserter(elems));
	return elems;
}
// -- END: Copied code --

StateLoader::StateLoader(const std::array<char, 256>& scenePath, SimulationData& simData, int& iterations)
{
	ClearSimData(simData);

	// Open the given text file
	std::ifstream inputFile(scenePath.data());

	// TODO: give some error if the file doesn't load and bail out early before overwriting the simulation data.

	std::string line;
	while (std::getline(inputFile, line))
	{
		std::vector<std::string> values = split(line, ' ');

		if (values[0] == "m")
		{
			iterations = std::stoi(values[1]);
		}
		if (values[0] == "mp") // particle meta
		{
			simData.numParticles = std::stoi(values[1]);

			simData.particlePositions.reserve(simData.numParticles);
			simData.particleVelocityX.reserve(simData.numParticles);
			simData.particleVelocityY.reserve(simData.numParticles);

			simData.particleVelocityDerivativeX.reserve(simData.numParticles);
			simData.particleVelocityDerivativeY.reserve(simData.numParticles);
		}
		else if (values[0] == "p") // particle info
		{
			glm::dvec2 pos = glm::dvec2(std::stod(values[1]), std::stod(values[2]));

			simData.particlePositions.insert(simData.particlePositions.end(), pos);
			simData.particleVelocityX.insert(simData.particleVelocityX.end(), std::stod(values[3]));
			simData.particleVelocityY.insert(simData.particleVelocityY.end(), std::stod(values[4]));
			
			glm::dvec2 velDervX = glm::dvec2(std::stod(values[5]), std::stod(values[6]));
			glm::dvec2 velDervY = glm::dvec2(std::stod(values[7]), std::stod(values[8]));

			simData.particleVelocityDerivativeX.insert(simData.particleVelocityDerivativeX.end(), velDervX);
			simData.particleVelocityDerivativeY.insert(simData.particleVelocityDerivativeY.end(), velDervY);
		}
	}

	// Close the file
	inputFile.close();
}

void StateLoader::ClearSimData(SimulationData& simData)
{
	simData.particlePositions.clear();

	simData.particleVelocityX.clear();
	simData.particleVelocityY.clear();

	simData.particleVelocityDerivativeX.clear();
	simData.particleVelocityDerivativeY.clear();
}