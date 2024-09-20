#pragma once

#include "SimulationScene.h"

// MPM fluid simulation

class MPMScene : public SimulationScene
{
public:
	MPMScene();
	~MPMScene() = default;

	void CalculateGridForce();

	void UpdateDeformationGradient();

	void UpdateCellVelocity() override;

private:
	void UpdateParticleSimulationData() override;
	void UpdateGridSimulationData() override;
};
