
#include "PICScene.h"

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "Interpolation.h"
#include "PolyPICHelper.h"

#include <onetbb/oneapi/tbb.h>
#include <Eigen/Dense>

PICScene::PICScene()
{
}

void PICScene::UpdateParticleSimulationData()
{
	SimulationScene::UpdateParticleSimulationData();
}

void PICScene::UpdateGridSimulationData()
{
	mSimulationData.gridDivergence.resize(mSimulationData.numGridCells);
	mSimulationData.gridPressure.resize(mSimulationData.numGridCells);
	mSimulationData.gridDensityError.resize(mSimulationData.numGridCells);
	mSimulationData.gridDensityControl.resize(mSimulationData.numGridCells);

	SimulationScene::UpdateGridSimulationData();
}

void PICScene::BumpParticlesDensity()
{
	double scale = -mSimulationData.deltaTime * mSimulationData.deltaTime / mSimulationData.fluidDensity;

	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		int closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);

		if (mSimulationData.cellType[closestCell] == CellType::eFIXED)
		{
			continue;
		}

		int x, y;
		std::tie(x, y) = GetXYFromIndex(closestCell);

		int leftNeighbour = GetIndexFromXY(x - 1, y);
		int rightNeighbour = GetIndexFromXY(x + 1, y);
		int bottomNeighbour = GetIndexFromXY(x, y - 1);
		int topNeighbour = GetIndexFromXY(x, y + 1);

		double left = mSimulationData.gridDensityControl[closestCell];
		double right = mSimulationData.gridDensityControl[closestCell];
		double bottom = mSimulationData.gridDensityControl[closestCell];
		double top = mSimulationData.gridDensityControl[closestCell];

		double sumX = 0.0;
		double sumY = 0.0;

		if (leftNeighbour != -1 && mSimulationData.cellType[leftNeighbour] != CellType::eFIXED)
		{
			left = mSimulationData.gridDensityControl[leftNeighbour];
			++sumX;
		}
		if (rightNeighbour != -1 && mSimulationData.cellType[rightNeighbour] != CellType::eFIXED)
		{
			right = mSimulationData.gridDensityControl[rightNeighbour];
			++sumX;
		}
		if (bottomNeighbour != -1 && mSimulationData.cellType[bottomNeighbour] != CellType::eFIXED)
		{
			bottom = mSimulationData.gridDensityControl[bottomNeighbour];
			++sumY;
		}
		if (topNeighbour != -1 && mSimulationData.cellType[topNeighbour] != CellType::eFIXED)
		{
			top = mSimulationData.gridDensityControl[topNeighbour];
			++sumY;
		}

		double bumpX = scale * (right - left) / (sumX * mSimulationData.cellSize);
		double bumpY = scale * (top - bottom) / (sumY * mSimulationData.cellSize);

		mSimulationData.particlePositions[pIndex].x += scale * (right - left) / (sumX * mSimulationData.cellSize);
		mSimulationData.particlePositions[pIndex].y += scale * (top - bottom) / (sumY * mSimulationData.cellSize);
	}
}

void PICScene::CalculateCellDivergence()
{
	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		if (mSimulationData.cellType[cIndex] != CellType::eFLUID)
		{
			mSimulationData.gridDivergence[cIndex] = 0.0;
			continue;
		}

		double divergence = 0.0;

		int x, y;
		std::tie(x, y) = GetXYFromIndex(cIndex);

		if (x > 0)
		{
			int leftNeighbour = GetIndexFromXY(x - 1, y);

			if (mSimulationData.cellType[leftNeighbour] == CellType::eFIXED)
			{
				divergence += mSimulationData.gridVelX[cIndex];
			}
		}

		if (x < mSimulationData.gridResolutionWidth - 1)
		{
			int rightNeighbour = GetIndexFromXY(x + 1, y);

			divergence += mSimulationData.gridVelX[rightNeighbour] - mSimulationData.gridVelX[cIndex];

			if (mSimulationData.cellType[rightNeighbour] == CellType::eFIXED)
			{
				divergence -= mSimulationData.gridVelX[rightNeighbour];
			}
		}

		if (y > 0)
		{
			int lowerNeighbour = GetIndexFromXY(x, y - 1);

			if (mSimulationData.cellType[lowerNeighbour] == CellType::eFIXED)
			{
				divergence += mSimulationData.gridVelY[cIndex];
			}
		}

		if (y < mSimulationData.gridResolutionHeight - 1)
		{
			int upperNeighbour = GetIndexFromXY(x, y + 1);

			divergence += mSimulationData.gridVelY[upperNeighbour] - mSimulationData.gridVelY[cIndex];

			if (mSimulationData.cellType[upperNeighbour] == CellType::eFIXED)
			{
				divergence -= mSimulationData.gridVelY[upperNeighbour];
			}
		}

		mSimulationData.gridDivergence[cIndex] = divergence * -mSimulationData.inverseCellSize;
	}
}

void PICScene::CalculateCellDensityError()
{
	std::vector<double> cellMass(mSimulationData.numGridCells, 0.0);

	for (int pIndex = 0; pIndex < mSimulationData.numParticles; pIndex++)
	{
		int closestCell = GetClosestCellIndex(mSimulationData.particlePositions[pIndex]);

		cellMass[closestCell] += mSimulationData.particleMass;
	}


	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		if (mSimulationData.cellType[cIndex] != CellType::eFLUID)
		{
			mSimulationData.gridDensityError[cIndex] = 0.0;
			continue;
		}

		double density = cellMass[cIndex] * mSimulationData.inverseCellVolume;

		if (GetNumTypeNeighbours(cIndex, CellType::eFLUID) < 4)
		{
			density = density < mSimulationData.fluidDensity ? mSimulationData.fluidDensity : density;
		}

		double a = 1.0 - density / mSimulationData.fluidDensity;
		a = a < -mSimulationData.halfCellSize ? -mSimulationData.halfCellSize : a;
		a = a > mSimulationData.halfCellSize ? mSimulationData.halfCellSize : a;

		double error = a / mSimulationData.deltaTime;

		mSimulationData.gridDensityError[cIndex] = -error;
	}
}

void PICScene::SolvePressure(int maxIterations)
{
	Eigen::SparseMatrix<double> A(mSimulationData.numGridCells, mSimulationData.numGridCells);

	InitialiseLinearSystem(A);

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;

	solver.setMaxIterations(maxIterations);
	//solver.setTolerance(SIM_PRECISION);

	solver.compute(A);

	mSimulationData.gridPressure = solver.solve(mSimulationData.gridDivergence);

	std::cout << "---- NUM PRESSURE SOLVE ITERATIONS: " << solver.iterations() << std::endl;
	std::cout << "---- PRESSURE SOLVE ERROR: " << solver.error() << std::endl;
}

void PICScene::SolveDensity(int maxIterations)
{
	Eigen::SparseMatrix<double> A(mSimulationData.numGridCells, mSimulationData.numGridCells);

	InitialiseLinearSystem(A);

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;

	solver.setMaxIterations(maxIterations);

	solver.compute(A);

	mSimulationData.gridDensityControl = solver.solve(mSimulationData.gridDensityError);

	std::cout << "---- NUM DENSITY SOLVE ITERATIONS: " << solver.iterations() << std::endl;
	std::cout << "---- DENSITY SOLVE ERROR: " << solver.error() << std::endl;
}

void PICScene::InitialiseLinearSystem(Eigen::SparseMatrix<double>& A)
{
	double scale = mSimulationData.deltaTime * mSimulationData.inverseCellVolume / mSimulationData.fluidDensity;

	// Eigen triplet holds (i,j,value) for constructing a sparse matrix.
	std::vector<Eigen::Triplet<double>> coefficients; // TODO: reserve the memory for this?

	for (int cIndex = 0; cIndex < mSimulationData.numGridCells; cIndex++)
	{
		if (mSimulationData.cellType[cIndex] == CellType::eFLUID)
		{
			double sum = 0.0;

			int x, y;
			std::tie(x, y) = GetXYFromIndex(cIndex);

			if (x > 0)
			{
				int leftNeighbour = GetIndexFromXY(x - 1, y);
				if (mSimulationData.cellType[leftNeighbour] == CellType::eFLUID)
				{
					coefficients.push_back(Eigen::Triplet<double>(leftNeighbour, cIndex, -scale));
					sum += 1.0;
				}
				else if (mSimulationData.cellType[leftNeighbour] != CellType::eFIXED)
				{
					sum += 1.0;
				}
			}

			if (x < mSimulationData.gridResolutionWidth - 1)
			{
				int rightNeighbour = GetIndexFromXY(x + 1, y);
				if (mSimulationData.cellType[rightNeighbour] == CellType::eFLUID)
				{
					coefficients.push_back(Eigen::Triplet<double>(rightNeighbour, cIndex, -scale));
					sum += 1.0;
				}
				else if (mSimulationData.cellType[rightNeighbour] != CellType::eFIXED)
				{
					sum += 1.0;
				}
			}

			if (y > 0)
			{
				int lowerNeighbour = GetIndexFromXY(x, y - 1);
				if (mSimulationData.cellType[lowerNeighbour] == CellType::eFLUID)
				{
					coefficients.push_back(Eigen::Triplet<double>(lowerNeighbour, cIndex, -scale));
					sum += 1.0;
				}
				else if (mSimulationData.cellType[lowerNeighbour] != CellType::eFIXED)
				{
					sum += 1.0;
				}
			}

			if (y < mSimulationData.gridResolutionHeight - 1)
			{
				int upperNeighbour = GetIndexFromXY(x, y + 1);
				if (mSimulationData.cellType[upperNeighbour] == CellType::eFLUID)
				{
					coefficients.push_back(Eigen::Triplet<double>(upperNeighbour, cIndex, -scale));
					sum += 1.0;
				}
				else if (mSimulationData.cellType[upperNeighbour] != CellType::eFIXED)
				{
					sum += 1.0;
				}
			}

			if (sum != 0.0)
			{
				coefficients.push_back(Eigen::Triplet<double>(cIndex, cIndex, scale * sum));
			}
		}
	}

	A.setFromTriplets(coefficients.begin(), coefficients.end());
}

void PICScene::UpdateCellVelocity()
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