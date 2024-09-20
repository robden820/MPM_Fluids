
#include "SimulationScene.h"

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "Interpolation.h"
#include "PolyPICHelper.h"

#include <onetbb/oneapi/tbb.h>
#include <Eigen/Dense>

SimulationScene::SimulationScene()
{
}

// TODO: can we roll this into the constructor of the simulation scene?
void SimulationScene::SetSimulationData(SimulationData inSimulationData)
{
	mSimulationData = inSimulationData;

	UpdateSimulationDataValues();
}

void SimulationScene::UpdateSimulationDataValues()
{
	mSimulationData.deltaTime = mSimulationData.timeStep;
	mSimulationData.elapsedSimulationTime = 0.0;

	mSimulationData.gravity = -981.0; // 981 as we are in cm, not meters.

	// Useful value to be saved.
	mSimulationData.halfCellSize = mSimulationData.cellSize * 0.5;
	mSimulationData.inverseCellSize = 1.0 / mSimulationData.cellSize;
	mSimulationData.cellVolume = mSimulationData.cellSize * mSimulationData.cellSize;
	mSimulationData.inverseCellVolume = 1.0 / mSimulationData.cellVolume;

	mSimulationData.numParticlesPerCell = 4; //TODO: set from file

	mSimulationData.picMethod = PICMethod::eAPIC; // TODO: set from file;
	mSimulationData.percentFLIP = 0.0;            // TODO: set from file;

	if (mSimulationData.picMethod == PICMethod::ePOLYPIC)
	{
		mSimulationData.polyPICNumScalarModes = 9; // TODO; this should come from a file.
		InitialisePolyPIC();
	}
	else
	{
		mSimulationData.polyPICNumScalarModes = 0;
	}

	UpdateParticleSimulationData();
	UpdateGridSimulationData();
}

void SimulationScene::UpdateParticleSimulationData()
{
	mSimulationData.particleCellMap.resize(mSimulationData.numParticles);

	if (mSimulationData.picMethod != PICMethod::ePIC)
	{
		mSimulationData.particleVelocityDerivativeX.resize(mSimulationData.numParticles);
		mSimulationData.particleVelocityDerivativeY.resize(mSimulationData.numParticles);

		if (mSimulationData.picMethod == PICMethod::ePOLYPIC)
		{
			mSimulationData.polypicCoefficientsX.resize(mSimulationData.numParticles * mSimulationData.polyPICNumScalarModes);
			mSimulationData.polypicCoefficientsY.resize(mSimulationData.numParticles * mSimulationData.polyPICNumScalarModes);

			mSimulationData.polypicContributionsX.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicContributionsY.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicCoefficientScales.resize(9);
			mSimulationData.polypicScalarModesX.resize(mSimulationData.numParticles * mSimulationData.polyPICNumScalarModes * weightMask);
			mSimulationData.polypicScalarModesY.resize(mSimulationData.numParticles * mSimulationData.polyPICNumScalarModes * weightMask);
			mSimulationData.polypicX0X.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicX0Y.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicX1X.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicX1Y.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicG0X.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicG0Y.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicG1X.resize(mSimulationData.numParticles * weightMask);
			mSimulationData.polypicG1Y.resize(mSimulationData.numParticles * weightMask);
		}
	}

	mSimulationData.numParticlesChanged = false; // We have update the particle data, so can set this back to false
}

void SimulationScene::UpdateGridSimulationData()
{
	// Resize simulation data vectors that aren't initialized through the scene file.

	mSimulationData.gridVelX.resize(mSimulationData.numGridCells);
	mSimulationData.gridVelY.resize(mSimulationData.numGridCells);

	mSimulationData.gridPrevVelX.resize(mSimulationData.numGridCells);
	mSimulationData.gridPrevVelY.resize(mSimulationData.numGridCells);

	mSimulationData.cellType.resize(mSimulationData.numGridCells);

	mSimulationData.gridMassX.resize(mSimulationData.numGridCells);
	mSimulationData.gridMassY.resize(mSimulationData.numGridCells);

	mSimulationData.gridSolidPhiX.resize(mSimulationData.numGridCells);
	mSimulationData.gridSolidPhiY.resize(mSimulationData.numGridCells);
}

void SimulationScene::InterpolatePToG()
{
	std::fill(mSimulationData.gridMassX.begin(), mSimulationData.gridMassX.end(), 0.0);
	std::fill(mSimulationData.gridMassY.begin(), mSimulationData.gridMassY.end(), 0.0);

	std::vector<double> nodeLiquidMomentumX(mSimulationData.numGridCells);
	std::vector<double> nodeLiquidMomentumY(mSimulationData.numGridCells);

	switch (mSimulationData.picMethod)
	{
	case PICMethod::ePIC:
		PICParticlesToGrid(nodeLiquidMomentumX, nodeLiquidMomentumY);
		break;
	case PICMethod::eAPIC:
		APICParticlesToGrid(nodeLiquidMomentumX, nodeLiquidMomentumY);
		break;
	case PICMethod::ePOLYPIC:
		PolyPICParticlesToGrid(nodeLiquidMomentumX, nodeLiquidMomentumY);
		break;
	default:
		break;
	}

	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		int x, y;
		std::tie(x, y) = GetXYFromIndex(cIndex);

		int leftNeighbour = GetIndexFromXY(x - 1, y);
		int lowerNeighbour = GetIndexFromXY(x, y - 1);

		if (mSimulationData.cellType[cIndex] == CellType::eFIXED || mSimulationData.cellType[leftNeighbour] == CellType::eFIXED || mSimulationData.cellType[lowerNeighbour] == CellType::eFIXED)
		{
			mSimulationData.gridVelX[cIndex] = 0.0;
			mSimulationData.gridVelY[cIndex] = 0.0;

			continue;
		}

		if (mSimulationData.gridMassX[cIndex] > 0.0)
		{
			mSimulationData.gridVelX[cIndex] = nodeLiquidMomentumX[cIndex] / mSimulationData.gridMassX[cIndex];
		}
		else
		{
			mSimulationData.gridVelX[cIndex] = 0.0;
		}

		if (mSimulationData.gridMassY[cIndex] > 0.0)
		{
			mSimulationData.gridVelY[cIndex] = nodeLiquidMomentumY[cIndex] / mSimulationData.gridMassY[cIndex];
		}
		else
		{
			mSimulationData.gridVelY[cIndex] = 0.0;
		}
	}

	SaveVelocity();
}

void SimulationScene::InterpolateGToP()
{
	std::fill(mSimulationData.particleVelocityDerivativeX.begin(), mSimulationData.particleVelocityDerivativeX.end(), glm::dvec2(0.0));
	std::fill(mSimulationData.particleVelocityDerivativeY.begin(), mSimulationData.particleVelocityDerivativeY.end(), glm::dvec2(0.0));

	switch (mSimulationData.picMethod)
	{
	case PICMethod::ePIC:
		PICGridToParticles();
		break;
	case PICMethod::eAPIC:
		APICGridToParticles();
		break;
	case PICMethod::ePOLYPIC:
		PolyPICGridToParticles();
		break;
	default:
		break;
	}
}

void SimulationScene::PICParticlesToGrid(std::vector<double>& nodeLiquidMomentumX, std::vector<double>& nodeLiquidMomentumY)
{
	// Standard particles
	//tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		int offset = 0;

		for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
		{
			if (*cellIter < 0)
			{
				offset++;
				continue;
			}

			glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

			glm::dvec2 diffX(diff.x - mSimulationData.halfCellSize, diff.y);
			glm::dvec2 diffY(diff.x, diff.y - mSimulationData.halfCellSize);

			double weightX = Interpolation::Linear(diffX.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffX.y * mSimulationData.inverseCellSize);
			double weightY = Interpolation::Linear(diffY.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffY.y * mSimulationData.inverseCellSize);

			nodeLiquidMomentumX[*cellIter] += mSimulationData.particleVelocityX[pIndex] * mSimulationData.particleMass * weightX;
			nodeLiquidMomentumY[*cellIter] += mSimulationData.particleVelocityY[pIndex] * mSimulationData.particleMass * weightY;

			mSimulationData.gridMassX[*cellIter] += mSimulationData.particleMass * weightX;
			mSimulationData.gridMassY[*cellIter] += mSimulationData.particleMass * weightY;

			offset++;
		}
	}
}

void SimulationScene::PICGridToParticles()
{
	// Standard particles
	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
	{
		double PICVelX = 0.0;
		double PICVelY = 0.0;

		double FLIPVelX = mSimulationData.particleVelocityX[pIndex];
		double FLIPVelY = mSimulationData.particleVelocityY[pIndex];

		for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
		{
			if (*cellIter < 0)
			{
				continue;
			}

			glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

			glm::dvec2 diffX(diff.x - mSimulationData.halfCellSize, diff.y);
			glm::dvec2 diffY(diff.x, diff.y - mSimulationData.halfCellSize);

			double weightX = Interpolation::Linear(diffX.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffX.y * mSimulationData.inverseCellSize);
			double weightY = Interpolation::Linear(diffY.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffY.y * mSimulationData.inverseCellSize);

			FLIPVelX += (mSimulationData.gridVelX[*cellIter] - mSimulationData.gridPrevVelX[*cellIter]) * weightX;
			FLIPVelY += (mSimulationData.gridVelY[*cellIter] - mSimulationData.gridPrevVelY[*cellIter]) * weightY;

			PICVelX += mSimulationData.gridVelX[*cellIter] * weightX;
			PICVelY += mSimulationData.gridVelY[*cellIter] * weightY;
		}

		mSimulationData.particleVelocityX[pIndex] = mSimulationData.percentFLIP * FLIPVelX + (1.0 - mSimulationData.percentFLIP) * PICVelX;
		mSimulationData.particleVelocityY[pIndex] = mSimulationData.percentFLIP * FLIPVelY + (1.0 - mSimulationData.percentFLIP) * PICVelY;
	});
}

void SimulationScene::APICParticlesToGrid(std::vector<double>& nodeLiquidMomentumX, std::vector<double>& nodeLiquidMomentumY)
{
	//tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		int offset = 0;

		for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
		{
			if (*cellIter < 0 || mSimulationData.cellType[*cellIter] == CellType::eFIXED)
			{
				offset++;
				continue;
			}

			glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

			glm::dvec2 diffX(diff.x - mSimulationData.halfCellSize, diff.y);
			glm::dvec2 diffY(diff.x, diff.y - mSimulationData.halfCellSize);

			double weightX = Interpolation::Linear(diffX.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffX.y * mSimulationData.inverseCellSize);
			double weightY = Interpolation::Linear(diffY.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffY.y * mSimulationData.inverseCellSize);

			const double velX = mSimulationData.particleVelocityX[pIndex] + glm::dot(mSimulationData.particleVelocityDerivativeX[pIndex], diffX);
			const double velY = mSimulationData.particleVelocityY[pIndex] + glm::dot(mSimulationData.particleVelocityDerivativeY[pIndex], diffY);

			nodeLiquidMomentumX[*cellIter] += velX * mSimulationData.particleMass * weightX;
			nodeLiquidMomentumY[*cellIter] += velY * mSimulationData.particleMass * weightY;

			mSimulationData.gridMassX[*cellIter] += mSimulationData.particleMass * weightX;
			mSimulationData.gridMassY[*cellIter] += mSimulationData.particleMass * weightY;

			offset++;
		}
	}
}

void SimulationScene::APICGridToParticles()
{
	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
	{
			int offset = 0;

			double PICVelX = 0.0;
			double PICVelY = 0.0;

			double FLIPVelX = mSimulationData.particleVelocityX[pIndex];
			double FLIPVelY = mSimulationData.particleVelocityY[pIndex];

			for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
			{
				if (*cellIter < 0 || mSimulationData.cellType[*cellIter] == CellType::eFIXED)
				{
					offset++;
					continue;
				}

				glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

				glm::dvec2 diffX(diff.x - mSimulationData.halfCellSize, diff.y);
				glm::dvec2 diffY(diff.x, diff.y - mSimulationData.halfCellSize);

				double weightX = Interpolation::Linear(diffX.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffX.y * mSimulationData.inverseCellSize);
				double weightY = Interpolation::Linear(diffY.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffY.y * mSimulationData.inverseCellSize);

				FLIPVelX += (mSimulationData.gridVelX[*cellIter] - mSimulationData.gridPrevVelX[*cellIter]) * weightX;
				FLIPVelY += (mSimulationData.gridVelY[*cellIter] - mSimulationData.gridPrevVelY[*cellIter]) * weightY;

				PICVelX += mSimulationData.gridVelX[*cellIter] * weightX;
				PICVelY += mSimulationData.gridVelY[*cellIter] * weightY;

				mSimulationData.particleVelocityDerivativeX[pIndex] += mSimulationData.gridVelX[*cellIter] * weightX * diffX * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize * 4.0;
				mSimulationData.particleVelocityDerivativeY[pIndex] += mSimulationData.gridVelY[*cellIter] * weightY * diffY * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize * 4.0;

				offset++;
			}

			mSimulationData.particleVelocityX[pIndex] = mSimulationData.percentFLIP * FLIPVelX + (1.0 - mSimulationData.percentFLIP) * PICVelX;
			mSimulationData.particleVelocityY[pIndex] = mSimulationData.percentFLIP * FLIPVelY + (1.0 - mSimulationData.percentFLIP) * PICVelY;
	});
}

void SimulationScene::PolyPICParticlesToGrid(std::vector<double>& nodeLiquidMomentumX, std::vector<double>& nodeLiquidMomentumY)
{
	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
		{
			int offset = 0;

			Eigen::Matrix2d angular = Eigen::Matrix2d::Identity() + mSimulationData.deltaTime * Eigen::Matrix2d{ { mSimulationData.particleVelocityDerivativeX[pIndex].x, mSimulationData.particleVelocityDerivativeX[pIndex].y },
																												 { mSimulationData.particleVelocityDerivativeY[pIndex].x, mSimulationData.particleVelocityDerivativeY[pIndex].y } };
			Eigen::Matrix2d angularInv = angular.inverse();

			for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
			{
				if (*cellIter < 0)
				{
					mSimulationData.polypicX0X[pIndex * weightMask + offset] = 100.0;
					mSimulationData.polypicX1X[pIndex * weightMask + offset] = 100.0;
					mSimulationData.polypicX0Y[pIndex * weightMask + offset] = 100.0;
					mSimulationData.polypicX1Y[pIndex * weightMask + offset] = 100.0;

					offset++;
					continue;
				}

				glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

				Eigen::Vector2d diffX(diff.x - mSimulationData.halfCellSize, diff.y);
				Eigen::Vector2d diffY(diff.x, diff.y - mSimulationData.halfCellSize);

				Eigen::Vector2d inputX = angularInv * diffX;
				Eigen::Vector2d inputY = angularInv * diffY;

				mSimulationData.polypicX0X[pIndex * weightMask + offset] = inputX.x();
				mSimulationData.polypicX1X[pIndex * weightMask + offset] = inputX.y();
				mSimulationData.polypicX0Y[pIndex * weightMask + offset] = inputY.x();
				mSimulationData.polypicX1Y[pIndex * weightMask + offset] = inputY.y();

				offset++;
			}
		});

	PolyPIC::G(mSimulationData.polypicX0X, mSimulationData.polypicG0X, mSimulationData.cellSize);
	PolyPIC::G(mSimulationData.polypicX1X, mSimulationData.polypicG1X, mSimulationData.cellSize);
	PolyPIC::G(mSimulationData.polypicX0Y, mSimulationData.polypicG0Y, mSimulationData.cellSize);
	PolyPIC::G(mSimulationData.polypicX1Y, mSimulationData.polypicG1Y, mSimulationData.cellSize);

	PolyPIC::CalculateScalarModes2D(mSimulationData.polypicScalarModesX, mSimulationData.polypicX0X, mSimulationData.polypicX1X, mSimulationData.polypicG0X, mSimulationData.polypicG1X, mSimulationData.numParticles, weightMask);
	PolyPIC::CalculateScalarModes2D(mSimulationData.polypicScalarModesY, mSimulationData.polypicX0Y, mSimulationData.polypicX1Y, mSimulationData.polypicG0Y, mSimulationData.polypicG1Y, mSimulationData.numParticles, weightMask);

	PolyPIC::Contribution(mSimulationData.polypicScalarModesX, mSimulationData.polypicCoefficientsX, mSimulationData.polypicContributionsX, 3, mSimulationData.numParticles, weightMask, false); // TODO; this 4.
	PolyPIC::Contribution(mSimulationData.polypicScalarModesY, mSimulationData.polypicCoefficientsY, mSimulationData.polypicContributionsY, 3, mSimulationData.numParticles, weightMask, false); // TODO; this 4.

	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
		{
			int offset = 0;

			for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
			{
				if (*cellIter < 0)
				{
					offset++;
					continue;
				}

				glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

				glm::dvec2 diffX(diff.x - mSimulationData.halfCellSize, diff.y);
				glm::dvec2 diffY(diff.x, diff.y - mSimulationData.halfCellSize);

				double weightX = Interpolation::Linear(diffX.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffX.y * mSimulationData.inverseCellSize);
				double weightY = Interpolation::Linear(diffY.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffY.y * mSimulationData.inverseCellSize);

				nodeLiquidMomentumX[*cellIter] += mSimulationData.particleMass * weightX * mSimulationData.polypicContributionsX[pIndex * weightMask + offset];
				nodeLiquidMomentumY[*cellIter] += mSimulationData.particleMass * weightY * mSimulationData.polypicContributionsY[pIndex * weightMask + offset];

				mSimulationData.gridMassX[*cellIter] += mSimulationData.particleMass * weightX;
				mSimulationData.gridMassY[*cellIter] += mSimulationData.particleMass * weightY;

				offset++;
			}
		});
}

void SimulationScene::PolyPICGridToParticles()
{
	std::vector<int> indicies(mSimulationData.numParticles * weightMask);
	std::vector<double> velsX(mSimulationData.numParticles * weightMask);
	std::vector<double> velsY(mSimulationData.numParticles * weightMask);

	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
		{
			int offset = 0;

			double PICVelX = 0.0;
			double PICVelY = 0.0;

			double FLIPVelX = mSimulationData.particleVelocityX[pIndex];
			double FLIPVelY = mSimulationData.particleVelocityY[pIndex];

			for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
			{
				if (*cellIter < 0)
				{
					mSimulationData.polypicX0X[pIndex * weightMask + offset] = 100.0;
					mSimulationData.polypicX1X[pIndex * weightMask + offset] = 100.0;
					mSimulationData.polypicX0Y[pIndex * weightMask + offset] = 100.0;
					mSimulationData.polypicX1Y[pIndex * weightMask + offset] = 100.0;

					offset++;
					continue;
				}

				glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

				glm::dvec2 diffX(diff.x - mSimulationData.halfCellSize, diff.y);
				glm::dvec2 diffY(diff.x, diff.y - mSimulationData.halfCellSize);

				double weightX = Interpolation::Linear(diffX.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffX.y * mSimulationData.inverseCellSize);
				double weightY = Interpolation::Linear(diffY.x * mSimulationData.inverseCellSize) * Interpolation::Linear(diffY.y * mSimulationData.inverseCellSize);

				FLIPVelX += (mSimulationData.gridVelX[*cellIter] - mSimulationData.gridPrevVelX[*cellIter]) * weightX;
				FLIPVelY += (mSimulationData.gridVelY[*cellIter] - mSimulationData.gridPrevVelY[*cellIter]) * weightY;

				PICVelX += mSimulationData.gridVelX[*cellIter] * weightX;
				PICVelY += mSimulationData.gridVelY[*cellIter] * weightY;

				mSimulationData.particleVelocityDerivativeX[pIndex] += mSimulationData.gridVelX[*cellIter] * weightX * diffX * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize * 4.0;
				mSimulationData.particleVelocityDerivativeY[pIndex] += mSimulationData.gridVelY[*cellIter] * weightY * diffY * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize * 4.0;

				velsX[pIndex * weightMask + offset] = mSimulationData.gridVelX[*cellIter];
				velsY[pIndex * weightMask + offset] = mSimulationData.gridVelY[*cellIter];

				indicies[pIndex * weightMask + offset] = offset;

				mSimulationData.polypicX0X[pIndex * weightMask + offset] = diffX.x;
				mSimulationData.polypicX1X[pIndex * weightMask + offset] = diffX.y;
				mSimulationData.polypicX0Y[pIndex * weightMask + offset] = diffY.x;
				mSimulationData.polypicX1Y[pIndex * weightMask + offset] = diffY.y;

				offset++;
			}

			mSimulationData.particleVelocityX[pIndex] = mSimulationData.percentFLIP * FLIPVelX + (1.0 - mSimulationData.percentFLIP) * PICVelX;
			mSimulationData.particleVelocityY[pIndex] = mSimulationData.percentFLIP * FLIPVelY + (1.0 - mSimulationData.percentFLIP) * PICVelY;
		});

	PolyPIC::G(mSimulationData.polypicX0X, mSimulationData.polypicG0X, mSimulationData.cellSize);
	PolyPIC::G(mSimulationData.polypicX1X, mSimulationData.polypicG1X, mSimulationData.cellSize);
	PolyPIC::G(mSimulationData.polypicX0Y, mSimulationData.polypicG0Y, mSimulationData.cellSize);
	PolyPIC::G(mSimulationData.polypicX1Y, mSimulationData.polypicG1Y, mSimulationData.cellSize);

	PolyPIC::CalculateNodeCoefficients2D(mSimulationData.polypicCoefficientsX, mSimulationData.polypicCoefficientScales,
		mSimulationData.polypicX0X, mSimulationData.polypicX1X,
		mSimulationData.polypicG0X, mSimulationData.polypicG1X,
		velsX, indicies, mSimulationData.inverseCellSize, mSimulationData.numParticles, weightMask);

	PolyPIC::CalculateNodeCoefficients2D(mSimulationData.polypicCoefficientsY, mSimulationData.polypicCoefficientScales,
		mSimulationData.polypicX0Y, mSimulationData.polypicX1Y,
		mSimulationData.polypicG0Y, mSimulationData.polypicG1Y,
		velsY, indicies, mSimulationData.inverseCellSize, mSimulationData.numParticles, weightMask);
}
/*
void SimulationScene::CalculateInterpolationWeights()
{
	// Standard particles
	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
	{
		int offset = 0;

		for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
		{
			if (*cellIter < 0)
			{
				mSimulationData.interpolationWeights2D[pIndex * weightMask + offset] = glm::dvec2(0.0);
				mSimulationData.interpolationWeightGradientsX2D[pIndex * weightMask + offset] = glm::dvec2(0.0);
				mSimulationData.interpolationWeightGradientsY2D[pIndex * weightMask + offset] = glm::dvec2(0.0);

				offset++;
				continue;
			}

			glm::dvec2 diff = mSimulationData.particlePositions[pIndex] - mSimulationData.gridCellPositions[*cellIter];

			glm::dvec2 kX = glm::dvec2(diff.x + mSimulationData.halfCellSize, diff.y) * mSimulationData.inverseCellSize;
			glm::dvec2 kY = glm::dvec2(diff.x, diff.y + mSimulationData.halfCellSize) * mSimulationData.inverseCellSize;

			mSimulationData.interpolationWeights2D[pIndex * weightMask + offset].x = Interpolation::Linear(kX.x) * Interpolation::Linear(kX.y);
			mSimulationData.interpolationWeights2D[pIndex * weightMask + offset].y = Interpolation::Linear(kY.x) * Interpolation::Linear(kY.y);

			mSimulationData.interpolationWeightGradientsX2D[pIndex * weightMask + offset].x = Interpolation::GradLinear(kX.x) * Interpolation::Linear(kX.y) * mSimulationData.inverseCellSize;
			mSimulationData.interpolationWeightGradientsX2D[pIndex * weightMask + offset].y = Interpolation::Linear(kX.x) * Interpolation::GradLinear(kX.y) * mSimulationData.inverseCellSize;

			mSimulationData.interpolationWeightGradientsY2D[pIndex * weightMask + offset].x = Interpolation::GradLinear(kY.x) * Interpolation::Linear(kY.y) * mSimulationData.inverseCellSize;
			mSimulationData.interpolationWeightGradientsY2D[pIndex * weightMask + offset].y = Interpolation::Linear(kY.x) * Interpolation::GradLinear(kY.y) * mSimulationData.inverseCellSize;

			offset++;
		}
	});
}
*/
void SimulationScene::SaveVelocity()
{
	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		mSimulationData.gridPrevVelX[cIndex] = mSimulationData.gridVelX[cIndex];
		mSimulationData.gridPrevVelY[cIndex] = mSimulationData.gridVelY[cIndex];
	}
}

void SimulationScene::UpdateParticleCellMap()
{
	tbb::parallel_for(0, mSimulationData.numParticles, 1, [&](int pIndex)
	{
		int closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);
		if (closestCell == -1)
		{
			mSimulationData.particleCellMap[pIndex].fill(-1);
		}
		else
		{
			GetCellNeighbours(closestCell, mSimulationData.particleCellMap[pIndex]);
		}
	});
}

void SimulationScene::CalculateSolidPhi()
{
	mSimulationData.gridSolidPhiX.assign(mSimulationData.numGridCells, 10000000.0);
	mSimulationData.gridSolidPhiY.assign(mSimulationData.numGridCells, 10000000.0);

	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		glm::dvec2 leftFace = mSimulationData.gridCellPositions[cIndex] - glm::dvec2(mSimulationData.halfCellSize, 0.0);
		glm::dvec2 lowerFace = mSimulationData.gridCellPositions[cIndex] - glm::dvec2(0.0, mSimulationData.halfCellSize);

		for (int sIndex = 0; sIndex < mSimulationData.solidObjects.size(); sIndex++)
		{
			double leftDistance;
			double lowerDistance;
			glm::dvec2 norm; // TODO: separate functions to remove this
			mSimulationData.solidObjects[sIndex].GetDistanceNormal(leftFace, leftDistance, norm);
			mSimulationData.solidObjects[sIndex].GetDistanceNormal(lowerFace, lowerDistance, norm);

			if (mSimulationData.solidObjects[sIndex].ContainsPoint(leftFace)) leftDistance *= -1.0;
			if (mSimulationData.solidObjects[sIndex].ContainsPoint(lowerFace)) lowerDistance *= -1.0;

			if (leftDistance < mSimulationData.gridSolidPhiX[cIndex]) mSimulationData.gridSolidPhiX[cIndex] = leftDistance;
			if (lowerDistance < mSimulationData.gridSolidPhiY[cIndex]) mSimulationData.gridSolidPhiY[cIndex] = lowerDistance;
		}
	}
}

void SimulationScene::ApplyExternalForces()
{
	double gravity = mSimulationData.gravity * mSimulationData.deltaTime;

	for(int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		mSimulationData.gridVelY[cIndex] += gravity;
	}
}

void SimulationScene::AdvectParticles()
{
	// Explicit RK3.

	std::vector<glm::dvec2> pos = mSimulationData.particlePositions;

	std::vector<glm::dvec2> particleVelDervsX = mSimulationData.particleVelocityDerivativeX;
	std::vector<glm::dvec2> particleVelDervsY = mSimulationData.particleVelocityDerivativeY;

	// K1 values are the original velocities.
	std::vector<double> k1X = mSimulationData.particleVelocityX;
	std::vector<double> k1Y = mSimulationData.particleVelocityY;

	// Calculate K2 values.
	for (int p = 0; p < mSimulationData.numParticles; p++)
	{
		mSimulationData.particlePositions[p].x = pos[p].x + 0.5 * mSimulationData.deltaTime * k1X[p];
		mSimulationData.particlePositions[p].y = pos[p].y + 0.5 * mSimulationData.deltaTime * k1Y[p];
	}

	UpdateParticleCellMap();
	InterpolateGToP();

	std::vector<double> k2X = mSimulationData.particleVelocityX;
	std::vector<double> k2Y = mSimulationData.particleVelocityY;

	// Calculate K3 values.
	for (int p = 0; p < mSimulationData.numParticles; p++)
	{
		mSimulationData.particlePositions[p].x = pos[p].x + 0.75 * mSimulationData.deltaTime * k2X[p];
		mSimulationData.particlePositions[p].y = pos[p].y + 0.75 * mSimulationData.deltaTime * k2Y[p];
	}

	UpdateParticleCellMap();
	InterpolateGToP();

	// Calculate final particle positions.
	double ninth = mSimulationData.deltaTime / 9.0;

	for (int p = 0; p < mSimulationData.numParticles; p++)
	{
		mSimulationData.particlePositions[p].x = pos[p].x + (2.0 * ninth * k1X[p]) + (3.0 * ninth * k2X[p]) + (4.0 * ninth * mSimulationData.particleVelocityX[p]);
		mSimulationData.particlePositions[p].y = pos[p].y + (2.0 * ninth * k1Y[p]) + (3.0 * ninth * k2Y[p]) + (4.0 * ninth * mSimulationData.particleVelocityY[p]);

		mSimulationData.particleVelocityX[p] = (mSimulationData.particlePositions[p].x - pos[p].x) / mSimulationData.deltaTime;
		mSimulationData.particleVelocityY[p] = (mSimulationData.particlePositions[p].y - pos[p].y) / mSimulationData.deltaTime;
	}

	mSimulationData.particleVelocityDerivativeX = particleVelDervsX;
	mSimulationData.particleVelocityDerivativeY = particleVelDervsY;
	
	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		int closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);

		if (closestCell != -1 && mSimulationData.cellType[closestCell] == CellType::eFIXED)
		{
			ProjectOutSolid(pIndex);
		}
	}
}

void SimulationScene::ProjectOutSolid(const int pIndex)
{
	int closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);
	int itr = 0;

	while (itr < 15 && closestCell != -1 && mSimulationData.cellType[closestCell] == CellType::eFIXED)
	{
		int x, y;
		std::tie(x, y) = GetXYFromIndex(closestCell);

		int rightNeighbour = GetIndexFromXY(x + 1, y);
		int upperNeighbour = GetIndexFromXY(x, y + 1);

		double solidPhiGradX = (mSimulationData.gridSolidPhiX[rightNeighbour] - mSimulationData.gridSolidPhiX[closestCell]) * mSimulationData.inverseCellSize;
		double solidPhiGradY = (mSimulationData.gridSolidPhiY[upperNeighbour] - mSimulationData.gridSolidPhiY[closestCell]) * mSimulationData.inverseCellSize;

		mSimulationData.particlePositions[pIndex].x += mSimulationData.deltaTime * solidPhiGradX;
		mSimulationData.particlePositions[pIndex].y += mSimulationData.deltaTime * solidPhiGradY;

		mSimulationData.particleVelocityX[pIndex] = 0.0;
		mSimulationData.particleVelocityY[pIndex] = 0.0;

		closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);
		++itr;
	}
}

void SimulationScene::UpdateCellVelocity()
{
	double scale = mSimulationData.deltaTime * mSimulationData.inverseCellSize / mSimulationData.fluidDensity;

	double fixedVelX = 0.0;
	double fixedVelY = 0.0;

	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		double thisCellPressure = mSimulationData.gridPressure[cIndex];

		int x, y;
		std::tie(x, y) = GetXYFromIndex(cIndex);

		if (x > 0)
		{
			int leftNeighbour = GetIndexFromXY(x - 1, y);

			double leftCellPressure = mSimulationData.gridPressure[leftNeighbour];

			if (mSimulationData.cellType[cIndex] == CellType::eFLUID || mSimulationData.cellType[leftNeighbour] == CellType::eFLUID)
			{
				if (mSimulationData.cellType[cIndex] == CellType::eFIXED || mSimulationData.cellType[leftNeighbour] == CellType::eFIXED)
				{
					mSimulationData.gridVelX[cIndex] = fixedVelX;
				}
				else
				{
					mSimulationData.gridVelX[cIndex] -= scale * (thisCellPressure - leftCellPressure + mSimulationData.gridDensityControl[cIndex] - mSimulationData.gridDensityControl[leftNeighbour]);
				}
			}
		}

		if (y > 0)
		{
			int lowerNeighbour = GetIndexFromXY(x, y - 1);

			double lowerCellPressure = mSimulationData.gridPressure[lowerNeighbour];

			if (mSimulationData.cellType[cIndex] == CellType::eFLUID || mSimulationData.cellType[lowerNeighbour] == CellType::eFLUID)
			{
				if (mSimulationData.cellType[cIndex] == CellType::eFIXED || mSimulationData.cellType[lowerNeighbour] == CellType::eFIXED)
				{
					mSimulationData.gridVelY[cIndex] = fixedVelY;
				}
				else
				{
					mSimulationData.gridVelY[cIndex] -= scale * (thisCellPressure - lowerCellPressure + mSimulationData.gridDensityControl[cIndex] - mSimulationData.gridDensityControl[lowerNeighbour]);
				}
			}
		}
	}
}

void SimulationScene::ExtrapolateVelocityFields(const int maxSearchDepth)
{
	std::vector<int> marker(mSimulationData.numGridCells);
	std::fill(marker.begin(), marker.end(), maxSearchDepth);

	std::vector<int> searchIndices;

	// Initialise first wave.
	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		if (mSimulationData.cellType[cIndex] == CellType::eFIXED)
		{
			continue;
		}

		// First if we know the grid cell velocity, mark it as zero.
		if (mSimulationData.cellType[cIndex] == CellType::eFLUID)
		{
			marker[cIndex] = 0;
		}
		// If at least one neighbour is a fluid, add the cell to the first wavefront (if neighbour is type, then it has a newly interpolated velocity)
		else if (GetNumTypeNeighbours(cIndex, CellType::eFLUID) > 0)
		{
			marker[cIndex] = 1;
			searchIndices.insert(searchIndices.end(), cIndex);
		}
	}

	int iteration = 0;

	// Perform breadth first search extrapolation.
	while (iteration < searchIndices.size())
	{
		int cellIndex = searchIndices[iteration];
		int cellMarker = marker[cellIndex];

		int numSearchedNeighbours = 0;
		double sumVelocitiesX = 0.0;
		double sumVelocitiesY = 0.0;

		double sumPrevVelocitiesX = 0.0;
		double sumPrevVelocitiesY = 0.0;

		int x, y;
		std::tie(x, y) = GetXYFromIndex(cellIndex);

		if (x > 0)
		{
			int leftNeighbour = GetIndexFromXY(x - 1, y);
			if (marker[leftNeighbour] < cellMarker)
			{
				sumVelocitiesX += mSimulationData.gridVelX[leftNeighbour];
				sumVelocitiesY += mSimulationData.gridVelY[leftNeighbour];

				sumPrevVelocitiesX += mSimulationData.gridPrevVelX[leftNeighbour];
				sumPrevVelocitiesY += mSimulationData.gridPrevVelY[leftNeighbour];

				++numSearchedNeighbours;
			}
			else if (marker[leftNeighbour] == maxSearchDepth && mSimulationData.cellType[leftNeighbour] != CellType::eFIXED)
			{
				marker[leftNeighbour] = cellMarker + 1;
				searchIndices.insert(searchIndices.end(), leftNeighbour);
			}
		}
		if (x < mSimulationData.gridResolutionWidth - 1)
		{
			int rightNeighbour = GetIndexFromXY(x + 1, y);
			if (marker[rightNeighbour] < cellMarker)
			{
				sumVelocitiesX += mSimulationData.gridVelX[rightNeighbour];
				sumVelocitiesY += mSimulationData.gridVelY[rightNeighbour];

				sumPrevVelocitiesX += mSimulationData.gridPrevVelX[rightNeighbour];
				sumPrevVelocitiesY += mSimulationData.gridPrevVelY[rightNeighbour];

				++numSearchedNeighbours;
			}
			else if (marker[rightNeighbour] == maxSearchDepth && mSimulationData.cellType[rightNeighbour] != CellType::eFIXED)
			{
				marker[rightNeighbour] = cellMarker + 1;
				searchIndices.insert(searchIndices.end(), rightNeighbour);
			}
		}
		if (y > 0)
		{
			int lowerNeighbour = GetIndexFromXY(x, y - 1);
			if (marker[lowerNeighbour] < cellMarker)
			{
				sumVelocitiesX += mSimulationData.gridVelX[lowerNeighbour];
				sumVelocitiesY += mSimulationData.gridVelY[lowerNeighbour];

				sumPrevVelocitiesX += mSimulationData.gridPrevVelX[lowerNeighbour];
				sumPrevVelocitiesY += mSimulationData.gridPrevVelY[lowerNeighbour];

				++numSearchedNeighbours;
			}
			else if (marker[lowerNeighbour] == maxSearchDepth && mSimulationData.cellType[lowerNeighbour] != CellType::eFIXED)
			{
				marker[lowerNeighbour] = cellMarker + 1;
				searchIndices.insert(searchIndices.end(), lowerNeighbour);
			}
		}
		if (y < mSimulationData.gridResolutionHeight - 1)
		{
			int upperNeighbour = GetIndexFromXY(x, y + 1);
			if (marker[upperNeighbour] < cellMarker)
			{
				sumVelocitiesX += mSimulationData.gridVelX[upperNeighbour];
				sumVelocitiesY += mSimulationData.gridVelY[upperNeighbour];

				sumPrevVelocitiesX += mSimulationData.gridPrevVelX[upperNeighbour];
				sumPrevVelocitiesY += mSimulationData.gridPrevVelY[upperNeighbour];

				++numSearchedNeighbours;
			}
			else if (marker[upperNeighbour] == maxSearchDepth && mSimulationData.cellType[upperNeighbour] != CellType::eFIXED)
			{
				marker[upperNeighbour] = cellMarker + 1;
				searchIndices.insert(searchIndices.end(), upperNeighbour);
			}
		}

		if (numSearchedNeighbours > 0)
		{
			double scale = 1.0 / (double)numSearchedNeighbours;

			mSimulationData.gridVelX[cellIndex] = sumVelocitiesX * scale;
			mSimulationData.gridVelY[cellIndex] = sumVelocitiesY * scale;

			mSimulationData.gridPrevVelX[cellIndex] = sumPrevVelocitiesX * scale;
			mSimulationData.gridPrevVelY[cellIndex] = sumPrevVelocitiesY * scale;
		}

		++iteration;
	}
}

void SimulationScene::DeleteOutOfBoundsParticles()
{
	for (int p = 0; p < mSimulationData.numParticles; )
	{
		if (IsParticleOutOfBounds(mSimulationData.particlePositions[p]))
		{
			DeleteParticle(p);
			mSimulationData.numParticlesChanged = true;
		}
		else
		{
			++p;
		}
	}

	if (mSimulationData.numParticlesChanged)
	{
		UpdateParticleSimulationData();
	}
}

bool SimulationScene::IsParticleOutOfBounds(const glm::dvec2& position) const
{
	if (position.x < mSimulationData.domainMinX) return true;
	if (position.x > mSimulationData.domainMaxX) return true;

	if (position.y < mSimulationData.domainMinY) return true;
	if (position.y > mSimulationData.domainMaxY) return true;

	if (GetClosestCellIndex(position) == -1) return true;

	return false;
}

void SimulationScene::AssignFixedCellTags()
{
	// Assign fixed tags for cells containing solids.
	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		mSimulationData.cellType[cIndex] = CellType::eUNKNOWN;

		for (int sIndex = 0; sIndex < mSimulationData.solidObjects.size(); sIndex++)
		{
			if (mSimulationData.solidObjects[sIndex].PointInBounds(mSimulationData.gridCellPositions[cIndex]))
			{
				if (mSimulationData.solidObjects[sIndex].ContainsPoint(mSimulationData.gridCellPositions[cIndex]))
				{
					mSimulationData.cellType[cIndex] = CellType::eFIXED;
					break;
				}
			}
		}
	}
}

void SimulationScene::AssignCellTypeTags()
{
	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		int closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);

		if (mSimulationData.cellType[closestCell] != CellType::eFIXED)
		{
			mSimulationData.cellType[closestCell] = CellType::eFLUID;
		}
	}
}

const int SimulationScene::GetNumTypeNeighbours(const int cellIndex, const CellType cellType) const
{
	int numNeighbours = 0;

	int x, y;
	std::tie(x, y) = GetXYFromIndex(cellIndex);

	if (x > 0)
	{
		int leftNeighbour = GetIndexFromXY(x - 1, y);
		if (mSimulationData.cellType[leftNeighbour] == cellType)
		{
			++numNeighbours;
		}
	}
	if (x < mSimulationData.gridResolutionWidth - 1)
	{
		int rightNeighbour = GetIndexFromXY(x + 1, y);
		if (mSimulationData.cellType[rightNeighbour] == cellType)
		{
			++numNeighbours;
		}
	}
	if (y > 0)
	{
		int lowerNeighbour = GetIndexFromXY(x, y - 1);
		if (mSimulationData.cellType[lowerNeighbour] == cellType)
		{
			++numNeighbours;
		}
	}
	if (y < mSimulationData.gridResolutionHeight - 1)
	{
		int upperNeighbour = GetIndexFromXY(x, y + 1);
		if (mSimulationData.cellType[upperNeighbour] == cellType)
		{
			++numNeighbours;
		}
	}

	return numNeighbours;
}

void SimulationScene::GetCellNeighbours(int cellIndex, std::array<int, 16>& indices)
{
	int x, y;
	std::tie(x, y) = GetXYFromIndex(cellIndex);

	for (int cIndex = 0; cIndex < 16; cIndex++)
	{
		int yOffset = (cIndex % 4) - 1;
		int xOffset = static_cast<int>(floor(cIndex / 4.0)) - 1;

		indices[cIndex] = GetIndexFromXY(x + xOffset, y + yOffset);
	}
}

int SimulationScene::GetClosestCellIndex(const glm::dvec2& position) const
{
	double leftBoundary = mSimulationData.gridCellPositions[0].x;
	double bottomBoundary = mSimulationData.gridCellPositions[0].y;

	int x = static_cast<int>(round((position.x - leftBoundary) * mSimulationData.inverseCellSize));
	int y = static_cast<int>(round((position.y - bottomBoundary) * mSimulationData.inverseCellSize));

	return GetIndexFromXY(x, y);
}

std::tuple<int, int> SimulationScene::GetXYFromIndex(int index) const
{
	int y = index % mSimulationData.gridResolutionHeight;
	int x = static_cast<int>(floor(index / mSimulationData.gridResolutionHeight)) % mSimulationData.gridResolutionWidth;

	return std::tuple<int, int>(x, y);
}

int SimulationScene::GetIndexFromXY(int inX, int inY, bool isCorner) const
{
	int corner = isCorner ? 1 : 0;

	if (inX < 0 || inX > mSimulationData.gridResolutionWidth - 1 + corner || inY < 0 || inY > mSimulationData.gridResolutionHeight - 1 + corner)
	{
		return -1;
	}

	return inY + inX * (mSimulationData.gridResolutionHeight + corner);
}

void SimulationScene::RebuildGrid()
{
	if (mSimulationData.numParticles == 0)
	{
		return;
	}

	double minX = 10000.0;
	double maxX = -10000.0;
	double minY = 10000.0;
	double maxY = -10000.0;

	// Find the simulation domain bounds
	// TODO: for fixed solids this won't ever update
	for (int sIndex = 0; sIndex < mSimulationData.solidObjects.size(); sIndex++)
	{
		if (mSimulationData.solidObjects[sIndex].GetLeftBound() < minX) minX = ceil(mSimulationData.solidObjects[sIndex].GetLeftBound());
		if (mSimulationData.solidObjects[sIndex].GetRightBound() > maxX) maxX = floor(mSimulationData.solidObjects[sIndex].GetRightBound());
		if (mSimulationData.solidObjects[sIndex].GetLowerBound() < minY) minY = ceil(mSimulationData.solidObjects[sIndex].GetLowerBound());
		if (mSimulationData.solidObjects[sIndex].GetUpperBound() > maxY) maxY = floor(mSimulationData.solidObjects[sIndex].GetUpperBound());
	}

	mSimulationData.gridResolutionWidth = static_cast<int>(floor((maxX - minX) * mSimulationData.inverseCellSize));
	mSimulationData.gridResolutionHeight = static_cast<int>(floor((maxY - minY) * mSimulationData.inverseCellSize));

	int newNumGridCells = mSimulationData.gridResolutionWidth * mSimulationData.gridResolutionHeight;

	std::vector<glm::dvec2> newGridPositions(newNumGridCells);

	int cIndex = 0;

	for (int x = 0; x < mSimulationData.gridResolutionWidth; x++)
	{
		for (int y = 0; y < mSimulationData.gridResolutionHeight; y++)
		{
			double posX = x * mSimulationData.cellSize + minX;
			double posY = y * mSimulationData.cellSize + minY;

			newGridPositions[cIndex] = glm::dvec2(posX, posY);

			cIndex++;
		}
	}

	if (mSimulationData.numGridCells != newNumGridCells)
	{
		mSimulationData.numGridCells = newNumGridCells;
		mSimulationData.gridCellPositions = newGridPositions;
		UpdateGridSimulationData();

	}
	AssignFixedCellTags();
}

glm::dvec2 SimulationScene::RandomPointInCell(const glm::dvec2& cellPosition)
{
	double posX = (static_cast<double>(rand()) / RAND_MAX - 0.5) * mSimulationData.cellSize;
	double posY = (static_cast<double>(rand()) / RAND_MAX - 0.5) * mSimulationData.cellSize;

	return glm::dvec2(cellPosition.x + posX, cellPosition.y + posY);
}

void SimulationScene::CreateNewParticle(const glm::dvec2& newPosition)
{
	mSimulationData.particlePositions.push_back(newPosition);

	mSimulationData.particleVelocityX.push_back(0.0);
	mSimulationData.particleVelocityY.push_back(0.0);

	++mSimulationData.numParticles;

	UpdateParticleSimulationData();
}

void SimulationScene::DeleteParticle(const int particleIndex)
{
	std::iter_swap(mSimulationData.particleVelocityX.begin() + particleIndex, &mSimulationData.particleVelocityX.back());
	std::iter_swap(mSimulationData.particleVelocityY.begin() + particleIndex, &mSimulationData.particleVelocityY.back());
	std::iter_swap(mSimulationData.particlePositions.begin() + particleIndex, &mSimulationData.particlePositions.back());

	mSimulationData.particleVelocityX.pop_back();
	mSimulationData.particleVelocityY.pop_back();
	mSimulationData.particlePositions.pop_back();

	if (mSimulationData.picMethod != PICMethod::ePIC)
	{
		std::iter_swap(mSimulationData.particleVelocityDerivativeX.begin() + particleIndex, &mSimulationData.particleVelocityDerivativeX.back());
		std::iter_swap(mSimulationData.particleVelocityDerivativeY.begin() + particleIndex, &mSimulationData.particleVelocityDerivativeY.back());

		mSimulationData.particleVelocityDerivativeX.pop_back();
		mSimulationData.particleVelocityDerivativeY.pop_back();
	}

	--mSimulationData.numParticles;
}

void SimulationScene::InitialisePolyPIC()
{
	PolyPIC::CalculateCoefficientScales2D(mSimulationData.polypicCoefficientScales, mSimulationData.inverseCellSize);
}

void SimulationScene::SetDeltaTime()
{
	double maxVel = 0.0;

	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		if (abs(mSimulationData.particleVelocityX[pIndex]) > maxVel) maxVel = abs(mSimulationData.particleVelocityX[pIndex]);
		if (abs(mSimulationData.particleVelocityY[pIndex]) > maxVel) maxVel = abs(mSimulationData.particleVelocityY[pIndex]);
	}

	double subStepDT = maxVel != 0.0 ? mSimulationData.cellSize / maxVel : mSimulationData.timeStep;

	mSimulationData.deltaTime = min(mSimulationData.timeStep, subStepDT);
}