#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Core>

#include "Solid.h"

enum CellType
{
	eAIR = 0,         // Contains only air particles
	eFLUID = 1,       // Contains only fluid particles
	eFIXED = 2,       // Contains a rigid solid
	eUNKNOWN = 3
};

enum PICMethod
{
	ePIC = 0,
	eAPIC = 1,
	ePOLYPIC = 2
};

struct SimulationData
{
	// Simulation Data
	double totalSimulationTime;
	double elapsedSimulationTime;
	double deltaTime;  // Time step used for each iteration, determined by max velocity.
	double timeStep; // Max time step set by simulation

	double gravity;

	int picMethod;
	double percentFLIP;
	int polyPICNumScalarModes; // Max is 27 for 3D, 9 for 2D

	// Fluid data
	double fluidDensity;
	double fluidSurfaceTensionCoefficient;
	// Air data
	double airDensity;

	// Useful meta data for writing out
	double totalLiquidVolume;
	double densitySolveError;
	int densitySolveIterations;
	double timeForStep;

	// Grid data - using a staggered MAC grid approach
	int gridResolutionWidth;
	int gridResolutionHeight;
	int numGridCells;
	int numParticlesPerCell;
	double cellSize;
	double halfCellSize;
	double inverseCellSize;
	double cellVolume;
	double inverseCellVolume;
	double scaledInvCellVolume; // Scaled to average cellMasses X Y Z, so for 3D scaled by 3, 2D scaled by 2. Then inverse.
	double domainMinX, domainMaxX;
	double domainMinY, domainMaxY;

	std::vector<glm::dvec2> gridCellPositions; // Center of each grid cell.

	// Face information of each cell, stored on left, lower and back cell faces.

	std::vector<double> gridVelX;
	std::vector<double> gridVelY;

	std::vector<double> gridPrevVelX;
	std::vector<double> gridPrevVelY;

	std::vector<double> gridForceX;
	std::vector<double> gridForceY;

	std::vector<double> gridMassX;
	std::vector<double> gridMassY;

	std::vector<double> gridSolidPhiX; // This phi is stored at cell faces.
	std::vector<double> gridSolidPhiY; // This phi is stored at cell faces.

	Eigen::VectorXd gridDivergence;   // Divergence of each grid cell.
	Eigen::VectorXd gridPressure;     // Pressure of each grid cell.

	Eigen::VectorXd gridDensityError;
	Eigen::VectorXd gridDensityControl;

	std::vector<CellType> cellType;    // Tags for if a cell is a fluid cell.
	std::vector<int> fluidCellIndices;  // contains the indices of fluid cells.

	// Particle data
	int numParticles; // Number of particles in the simulation.
	bool numParticlesChanged;

	std::vector<glm::dvec2> particlePositions;

	std::vector<double> particleVelocityX;
	std::vector<double> particleVelocityY;

	double particleMass;

	std::vector<glm::dvec2> particleVelocityDerivativeX;
	std::vector<glm::dvec2> particleVelocityDerivativeY;

	std::vector<Eigen::Matrix2d> particleDeformationGradient;

	// PolyPIC Data
	std::vector<double> polypicCoefficientsX;
	std::vector<double> polypicCoefficientsY;

	std::vector<double> polypicContributionsX;
	std::vector<double> polypicContributionsY;

	std::vector<double> polypicCoefficientScales;
	std::vector<double> polypicScalarModesX;
	std::vector<double> polypicScalarModesY;

	std::vector<double> polypicX0X;
	std::vector<double> polypicX0Y;

	std::vector<double> polypicX1X;
	std::vector<double> polypicX1Y;

	std::vector<double> polypicX2X;
	std::vector<double> polypicX2Y;

	std::vector<double> polypicG0X;
	std::vector<double> polypicG0Y;

	std::vector<double> polypicG1X;
	std::vector<double> polypicG1Y;

	std::vector<double> polypicG2X;
	std::vector<double> polypicG2Y;

	// Particle-Cell data
	std::vector<std::array<int, 16>> particleCellMap; // contains the grid indicies for each particle

	// Other scene data
	std::vector<Solid> solidObjects;
};