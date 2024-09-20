#pragma once

#include "SimulationScene.h"

// Single phase fluid simulation

class PICScene : public SimulationScene
{
public:
	PICScene();
	~PICScene() = default;

	void BumpParticlesDensity(); // Bump particles from density solve

	void CalculateCellDivergence();
	void CalculateCellDensityError();

	void SolvePressure(int maxIterations = 1000);
	void SolveDensity(int maxIterations = 1000);

	void UpdateCellVelocity() override;

private:

	void InitialiseLinearSystem(Eigen::SparseMatrix<double>& A);

	void UpdateParticleSimulationData() override;
	void UpdateGridSimulationData() override;
};

