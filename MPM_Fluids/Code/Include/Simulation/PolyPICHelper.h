#pragma once

#include <vector>
#include <Eigen/Core>
#include <glm/glm.hpp>

namespace PolyPIC
{
	void Contribution(const std::vector<double>& scalarModes, const std::vector<double>& coefficients, std::vector<double>& contributions,
					  const int numScalarModes, const int numParticles, const int weightMask, const bool threeD);

	void CalculateNodeCoefficients2D(std::vector<double>& polypicCoefficients, const std::vector<double>& polypicCoefficientScales,
									 const std::vector<double>& x0, const std::vector<double>& x1,
									 const std::vector<double>& g0, const std::vector<double>& g1,
									 const std::vector<double>& velocity, const std::vector<int>& idx,
									 const double invDeltaX, const int numParticles, const int weightMask); // Coefficient for the single node, to be summed over all contributing nodes.
	void CalculateNodeCoefficients3D(std::vector<double>& polypicCoefficients, const std::vector<double>& polypicCoefficientScales,
									 const std::vector<double>& x0, const std::vector<double>& x1, const std::vector<double>& x2,
									 const std::vector<double>& g0, const std::vector<double>& g1, const std::vector<double>& g2,
									 const double velocity, const double invDeltaX, const int idx, const int numParticles); // Coefficient for the single node, to be summed over all contributing nodes.

	void CalculateScalarModes2D(std::vector<double>& scalarModes, const std::vector<double>& x0, const std::vector<double>& x1, const std::vector<double>& g0, const std::vector<double>& g1, const int numParticles, const int weightMask);
	void CalculateScalarModes3D(std::vector<double>& scalarModes, const std::vector<double>& x0, const std::vector<double>& x1, const std::vector<double>& x2,
																  const std::vector<double>& g0, const std::vector<double>& g1, const std::vector<double>& g2,
																  const int numParticles);

	void G(const std::vector<double>& input, std::vector<double>& output, const double deltaX);

	void CalculateCoefficientScales2D(std::vector<double>& polypicCoefficientScales, const double invDeltaX);
	void CalculateCoefficientScales3D(std::vector<double>& polypicCoefficientScales, const double invDeltaX);

	const int numModes2D = 9;
	const int numModes3D = 27;
}