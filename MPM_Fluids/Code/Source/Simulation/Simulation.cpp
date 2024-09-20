
#include "Simulation.h"

#include <iostream>

Simulation::Simulation(const SimType simType)
	: mIterationCount(0), simType(simType), elapsedTime(0.0)
{
	std::cout << "CreateScene" << std::endl;
}

Simulation::~Simulation()
{
}

void Simulation::Update()
{
	switch (simType)
	{
		case(SimType::MPM):
			UpdateMPM();
			break;
		case(SimType::PIC):
			UpdatePIC();
			break;
		default:
			break;
	}
}

void Simulation::UpdatePIC()
{
	mSimulationScene.SetDeltaTime();
	int numSteps = mSimulationScene.GetNumSubsteps();

	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	std::cout << "Iteration: " << mIterationCount++ << std::endl;
	std::cout << "Total elapsed time: " << elapsedTime << " seconds, realtime: " << mIterationCount * mSimulationScene.GetTimeStep() << std::endl;
	std::cout << "Num substeps: " << numSteps << std::endl;
	std::cout << " -- Particles: " << mSimulationScene.GetSimulationData().numParticles << std::endl;

	for (int substep = 0; substep < numSteps; substep++)
	{
		StepPIC();
	}
}

void Simulation::StepPIC()
{
	Timer loopTimer;

	mTimer.SetStart(glfwGetTime());

	mSimulationScene.RebuildGrid();
	mTimer.PrintTimerReset("Rebuild grid around particles");

	mSimulationScene.CalculateSolidPhi();
	mTimer.PrintTimerReset("Calculate solid signed distance field");

	mSimulationScene.UpdateParticleCellMap();
	mTimer.PrintTimerReset("Update Particle-Cell Map");

	mSimulationScene.InterpolatePToG();
	mTimer.PrintTimerReset("Interpolate particle velocities to grid");

	mSimulationScene.AssignCellTypeTags();
	mTimer.PrintTimerReset("Assign cell tags");

	mSimulationScene.CalculateCellDensityError();
	mTimer.PrintTimerReset("Calculate cell density error");

	mSimulationScene.ApplyExternalForces();
	mTimer.PrintTimerReset("Apply external forces");

	mSimulationScene.CalculateCellDivergence();
	mTimer.PrintTimerReset("Calculate grid cell divergence");

	mSimulationScene.SolvePressure();
	mTimer.PrintTimerReset("Pressure solve");

	mSimulationScene.SolveDensity();
	mTimer.PrintTimerReset("Density solve");

	mSimulationScene.UpdateCellVelocity();
	mTimer.PrintTimerReset("Update cell velocities");

	// Extrapolate grid velocities so that all nodes have a valid, up-to-date velocity.
	mSimulationScene.ExtrapolateVelocityFields();
	mTimer.PrintTimerReset("Extrapolate interpolated velocity fields");

	mSimulationScene.InterpolateGToP();
	mTimer.PrintTimerReset("Interpolate grid velocities to particles");

	// step particles in time.
	mSimulationScene.AdvectParticles();
	mTimer.PrintTimerReset("Advect particles");

	// Delete particles outside of the simulation domain. // Or fluid particles with zero volume
	mSimulationScene.DeleteOutOfBoundsParticles();
	mTimer.PrintTimerReset("Delete particles outside domain");

	mSimulationScene.IncrementTime();
	loopTimer.PrintTimer("----- TOTAL ITERATION TIME");
	elapsedTime += loopTimer.Now();
}

void Simulation::UpdateMPM()
{
	mMPMScene.SetDeltaTime();
	int numSteps = mMPMScene.GetNumSubsteps();

	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	std::cout << "Iteration: " << mIterationCount++ << std::endl;
	std::cout << "Total elapsed time: " << elapsedTime << " seconds, realtime: " << mIterationCount * mMPMScene.GetTimeStep() << std::endl;
	std::cout << "Num substeps: " << numSteps << std::endl;
	std::cout << " -- Particles: " << mMPMScene.GetSimulationData().numParticles << std::endl;

	for (int substep = 0; substep < numSteps; substep++)
	{
		StepMPM();
	}
}

void Simulation::StepMPM()
{
	Timer loopTimer;

	mTimer.SetStart(glfwGetTime());

	mMPMScene.RebuildGrid();
	mTimer.PrintTimerReset("Rebuild grid around particles");

	mMPMScene.CalculateSolidPhi();
	mTimer.PrintTimerReset("Calculate solid signed distance field");

	mMPMScene.UpdateParticleCellMap();
	mTimer.PrintTimerReset("Update Particle-Cell Map");

	mMPMScene.InterpolatePToG();
	mTimer.PrintTimerReset("Interpolate particle velocities to grid");

	mMPMScene.AssignCellTypeTags();
	mTimer.PrintTimerReset("Assign cell tags");

	mMPMScene.ApplyExternalForces();
	mTimer.PrintTimerReset("Apply external forces");

	mMPMScene.CalculateGridForce();
	mTimer.PrintTimerReset("Calculating grid forces");

	mMPMScene.UpdateCellVelocity();
	mTimer.PrintTimerReset("Update cell velocities");

	// Extrapolate grid velocities so that all nodes have a valid, up-to-date velocity.
	mMPMScene.ExtrapolateVelocityFields();
	mTimer.PrintTimerReset("Extrapolate interpolated velocity fields");

	mMPMScene.UpdateDeformationGradient();
	mTimer.PrintTimerReset("Updating particle deformation gradients");

	mMPMScene.InterpolateGToP();
	mTimer.PrintTimerReset("Interpolate grid velocities to particles");

	// step particles in time.
	mMPMScene.AdvectParticles();
	mTimer.PrintTimerReset("Advect particles");

	// Delete particles outside of the simulation domain. // Or fluid particles with zero volume
	mMPMScene.DeleteOutOfBoundsParticles();
	mTimer.PrintTimerReset("Delete particles outside domain");

	mMPMScene.IncrementTime();
	loopTimer.PrintTimer("----- TOTAL ITERATION TIME");
	elapsedTime += loopTimer.Now();
}

bool Simulation::IsComplete()
{
	SimulationData simData = GetSimulationData();

	if (simData.elapsedSimulationTime >= simData.totalSimulationTime)
	{
		return true;
	}

	return false;
}

SimulationData& Simulation::GetSimulationData()
{
	switch (simType)
	{
	case(SimType::MPM):
		return mMPMScene.GetSimulationData();
		break;
	case(SimType::PIC):
		return mSimulationScene.GetSimulationData();
		break;
	default:
		break;
	}

	// TODO: how to resolve this?
	SimulationData simData;
	return simData;
}

void Simulation::SetSimulationData(SimulationData inSimulationData)
{
	switch (simType)
	{
	case(SimType::MPM):
		mMPMScene.SetSimulationData(inSimulationData);
		break;
	case(SimType::PIC):
		mSimulationScene.SetSimulationData(inSimulationData);
		break;
	default:
		break;
	}
}

void Simulation::UpdateSimulationDataValues()
{
	switch (simType)
	{
	case(SimType::MPM):
		mMPMScene.UpdateSimulationDataValues();
		break;
	case(SimType::PIC):
		mSimulationScene.UpdateSimulationDataValues();
		break;
	default:
		break;
	}
}