
#include "MPMScene.h"

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "Interpolation.h"
#include "PolyPICHelper.h"

#include <onetbb/oneapi/tbb.h>
#include <Eigen/Dense>

MPMScene::MPMScene()
{
}

void MPMScene::UpdateParticleSimulationData()
{
	mSimulationData.particleDeformationGradient.resize(mSimulationData.numParticles, Eigen::Matrix2d::Identity());

	SimulationScene::UpdateParticleSimulationData();
}

void MPMScene::UpdateGridSimulationData()
{
	mSimulationData.gridForceX.resize(mSimulationData.numGridCells);
	mSimulationData.gridForceY.resize(mSimulationData.numGridCells);

	SimulationScene::UpdateGridSimulationData();
}

void MPMScene::CalculateGridForce()
{
	std::fill(mSimulationData.gridForceX.begin(), mSimulationData.gridForceX.end(), 0.0);
	std::fill(mSimulationData.gridForceY.begin(), mSimulationData.gridForceY.end(), 0.0);

	double k = 100.0;
	double gamma = 7.0;

	double particleVolume0 = mSimulationData.cellVolume / mSimulationData.numParticlesPerCell;
	
	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		double J = mSimulationData.particleDeformationGradient[pIndex].determinant();

		if (J == 0.0)
		{
			continue;
		}

		double pressure = k * (1.0 / pow(J, gamma) - 1.0);
		
		Eigen::Matrix2d stress = Eigen::Matrix2d{ {-pressure, 0.0}, {0.0, -pressure} };

		for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
		{
			if (*cellIter < 0)
			{
				continue;
			}

			glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

			Eigen::Vector2d diffX(diff.x - mSimulationData.halfCellSize, diff.y);
			Eigen::Vector2d diffY(diff.x, diff.y - mSimulationData.halfCellSize);

			double weightX = Interpolation::QuadBSpline(diffX.x() * mSimulationData.inverseCellSize) * Interpolation::QuadBSpline(diffX.y() * mSimulationData.inverseCellSize);
			double weightY = Interpolation::QuadBSpline(diffY.x() * mSimulationData.inverseCellSize) * Interpolation::QuadBSpline(diffY.y() * mSimulationData.inverseCellSize);

			mSimulationData.gridForceX[*cellIter] -= (particleVolume0 * stress * weightX * diffX * 4.0 * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize).x();
			mSimulationData.gridForceY[*cellIter] -= (particleVolume0 * stress * weightY * diffY * 4.0 * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize).y();
		}
	}
}

void MPMScene::UpdateDeformationGradient()
{
	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		Eigen::Matrix2d newDeformationGrad = Eigen::Matrix2d::Identity();

		for (std::array<int, 16>::iterator cellIter = mSimulationData.particleCellMap[pIndex].begin(); cellIter < mSimulationData.particleCellMap[pIndex].end(); cellIter++)
		{
			if (*cellIter < 0 || mSimulationData.cellType[*cellIter] == CellType::eFIXED)
			{
				continue;
			}

			glm::dvec2 diff = mSimulationData.gridCellPositions[*cellIter] - mSimulationData.particlePositions[pIndex];

			Eigen::Vector2d diffX(diff.x - mSimulationData.halfCellSize, diff.y);
			Eigen::Vector2d diffY(diff.x, diff.y - mSimulationData.halfCellSize);

			double weightX = Interpolation::QuadBSpline(diffX.x() * mSimulationData.inverseCellSize) * Interpolation::QuadBSpline(diffX.y() * mSimulationData.inverseCellSize);
			double weightY = Interpolation::QuadBSpline(diffY.x() * mSimulationData.inverseCellSize) * Interpolation::QuadBSpline(diffY.y() * mSimulationData.inverseCellSize);

			Eigen::Vector2d weightGrad( weightX * diffX.x() * 4.0 * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize,
										weightY * diffY.y() * 4.0 * mSimulationData.inverseCellSize * mSimulationData.inverseCellSize );

			Eigen::Vector2d velocity( mSimulationData.gridVelX[*cellIter], mSimulationData.gridVelY[*cellIter] );
			
			newDeformationGrad += mSimulationData.deltaTime * velocity * weightGrad.transpose();
		}
		
		mSimulationData.particleDeformationGradient[pIndex] = newDeformationGrad * mSimulationData.particleDeformationGradient[pIndex];
	}
}

void MPMScene::UpdateCellVelocity()
{
	double scale = mSimulationData.deltaTime / (mSimulationData.cellVolume * mSimulationData.fluidDensity);

	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		if (mSimulationData.cellType[cIndex] == CellType::eFIXED)
		{
			mSimulationData.gridVelX[cIndex] = 0.0;
			mSimulationData.gridVelY[cIndex] = 0.0;
		}

		int x, y;
		std::tie(x, y) = GetXYFromIndex(cIndex);

		int leftNeighbour = GetIndexFromXY(x - 1, y);
		int lowerNeighbour = GetIndexFromXY(x, y - 1);

		if (leftNeighbour != -1 && mSimulationData.cellType[leftNeighbour] == CellType::eFIXED)
		{
			mSimulationData.gridVelX[cIndex] = 0.0;
		}

		if (lowerNeighbour != -1 && mSimulationData.cellType[lowerNeighbour] == CellType::eFIXED)
		{
			mSimulationData.gridVelY[cIndex] = 0.0;
		}

		mSimulationData.gridVelX[cIndex] += scale * mSimulationData.gridForceX[cIndex];
		mSimulationData.gridVelY[cIndex] += scale * mSimulationData.gridForceY[cIndex];
	}
}