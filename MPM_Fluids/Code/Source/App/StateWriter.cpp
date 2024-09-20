#include "StateWriter.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <direct.h>

StateWriter::StateWriter(const int iterations, const char* fileName, const SimulationData& simData, WriteType writeType)
{
	std::stringstream iterationsSS;
	iterationsSS << std::setw(6) << std::setfill('0') << iterations;

	// TODO: don't want to be making a directory every time and update path automatically
	std::string dir = "Output/test_small";
	if (_mkdir(dir.c_str()) == -1)
	{
		std::array<char, 256> error;
		strerror_s(error.data(), 256, errno);
		//std::cout << error.data();
	}

	// TODO: update this path automatically from the input scene.
	std::array<char, 256> scenePath;
	sprintf_s(scenePath.data(), 256, "Output/test_small/%s_%s.txt", fileName, iterationsSS.str().c_str());

	// Create and open a text file
	// TODO: error if can't create /find filefile.
	std::ofstream outputFile(scenePath.data());

	if (writeType == WriteType::eSIMULATION)
	{
		WriteSimulation(outputFile, iterations, simData);
	}
	else if (writeType == WriteType::eRENDER)
	{
		WriteRender(outputFile, iterations, simData);
	}
	else if (writeType == WriteType::eMETA)
	{
		WriteMeta(outputFile, iterations, simData);
	}

	// Close the file
	outputFile.close();
}

void StateWriter::WriteSimulation(std::ofstream& outputFile, const int iterations, const SimulationData& simData)
{
	// Write to the file
	outputFile << "m " << iterations << std::endl;

	//Particle meta info
	outputFile << "mp ";
	outputFile << simData.numParticles << std::endl;

	// Particle info
	for (int pIndex = 0; pIndex < simData.numParticles; pIndex++)
	{
		outputFile << "p "; // 0
		outputFile << simData.particlePositions[pIndex].x << " " << simData.particlePositions[pIndex].y << " "; // 1, 2
		outputFile << simData.particleVelocityX[pIndex] << " " << simData.particleVelocityY[pIndex] << " "; // 3, 4

		outputFile << simData.particleVelocityDerivativeX[pIndex].x << " " << simData.particleVelocityDerivativeX[pIndex].y << " "; // 5, 6
		outputFile << simData.particleVelocityDerivativeY[pIndex].x << " " << simData.particleVelocityDerivativeY[pIndex].y << " "; // 7, 8

		outputFile << std::endl;
	}
}

void StateWriter::WriteRender(std::ofstream& outputFile, const int iterations, const SimulationData& simData)
{
	// Particle info
	for (int pIndex = 0; pIndex < simData.numParticles; pIndex++)
	{
		outputFile << "p "; // 0
		outputFile << simData.particlePositions[pIndex].x << " " << simData.particlePositions[pIndex].y << " "; // 1, 2

		outputFile << std::endl;
	}
}

void StateWriter::WriteMeta(std::ofstream& outputFile, const int iterations, const SimulationData& simData)
{
	outputFile << "volume ";
	outputFile << simData.totalLiquidVolume << " ";

	outputFile << "density_iterations ";
	outputFile << simData.densitySolveIterations << " ";
	
	outputFile << "density_error ";
	outputFile << simData.densitySolveError << " ";

	outputFile << "timestep ";
	outputFile << simData.timeForStep << " ";

	outputFile << std::endl;
}
