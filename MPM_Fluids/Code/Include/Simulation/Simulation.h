#pragma once

#include "SimulationScene.h"
#include "PICScene.h"
#include "MPMScene.h"
#include "SimulationData.h"

#include "Timer.h"

enum SimType
{
	MPM = 0,
	PIC = 1,
	UNKNOWN = 2
};

class Simulation
{
public:
	Simulation(const SimType simType);
	~Simulation();

	void Update();

	bool IsComplete();

	SimulationData& GetSimulationData();
	void SetSimulationData(SimulationData inSimulationData);
	void UpdateSimulationDataValues();

private:

	void UpdatePIC();
	void UpdateMPM();

	void StepPIC();
	void StepMPM();

	PICScene mSimulationScene;
	MPMScene mMPMScene;

	Timer mTimer;
	int mIterationCount;
	float elapsedTime;

	SimType simType;
};