#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <tuple>

#include "SimulationData.h"

#include <Eigen/Sparse>

// Single phase fluid simulation

class SimulationScene
{
public:
	SimulationScene();
	~SimulationScene() = default;

	SimulationData& GetSimulationData() { return mSimulationData; }
	void SetSimulationData(SimulationData inSimulationData);
	virtual void UpdateSimulationDataValues();

	const int GetNumParticles() const { return mSimulationData.numParticles; }

	void IncrementTime() { mSimulationData.elapsedSimulationTime += mSimulationData.deltaTime; }

	void UpdateParticleCellMap();

	void InterpolatePToG();
	void InterpolateGToP();

	void ApplyExternalForces();

	void SaveVelocity();

	void AdvectParticles();

	virtual void UpdateCellVelocity();

	void ExtrapolateVelocityFields(const int maxSearchDepth = 20000); // Breadth first search.

	void AssignFixedCellTags();
	void AssignCellTypeTags();

	void CalculateSolidPhi();

	void RebuildGrid(); // Reconstructs the grid around the maximum extent of the particles (to within a maximum).
	void DeleteOutOfBoundsParticles();

	const double GetDeltaTime() const { return mSimulationData.deltaTime; }
	const double GetTimeStep() const { return mSimulationData.timeStep; }
	const int GetNumSubsteps() const { return static_cast<int>(ceil(mSimulationData.timeStep / mSimulationData.deltaTime)); }
	void SetDeltaTime();

protected:

	glm::dvec2 RandomPointInCell(const glm::dvec2& cellPosition);
	void CreateNewParticle(const glm::dvec2& newPosition);
	void DeleteParticle(const int particleIndex);

	void PICParticlesToGrid(std::vector<double>& nodeLiquidMomentumX, std::vector<double>& nodeLiquidMomentumY);
	void PICGridToParticles();

	void APICParticlesToGrid(std::vector<double>& nodeLiquidMomentumX, std::vector<double>& nodeLiquidMomentumY);
	void APICGridToParticles();

	void PolyPICParticlesToGrid(std::vector<double>& nodeLiquidMomentumX, std::vector<double>& nodeLiquidMomentumY);
	void PolyPICGridToParticles();
	void InitialisePolyPIC();

	void GetCellNeighbours(int cellIndex, std::array<int, 16>& indices);

	const int GetNumTypeNeighbours(const int cellIndex, const CellType cellType) const;

	int GetClosestCellIndex(const glm::dvec2& position) const;

	std::tuple<int, int> GetXYFromIndex(int index) const; // TODO: is this the best place for this function?
	int GetIndexFromXY(int inX, int inY, bool isCorner = false) const; // TODO: as above.

	virtual void UpdateParticleSimulationData();
	virtual void UpdateGridSimulationData();

	bool IsParticleOutOfBounds(const glm::dvec2& position) const;
	void ProjectOutSolid(const int pIndex);

	double Clamp(const double value, const double lower, const double higher) const { return value > higher ? higher : (value < lower ? lower : value); }

	SimulationData mSimulationData;

	int weightMask = 16;
};
