
#include "PolyPICHelper.h"
#include "Interpolation.h"

void PolyPIC::CalculateScalarModes2D(std::vector<double>& scalarModes, const std::vector<double>& x0, const std::vector<double>& x1,
									 const std::vector<double>& g0, const std::vector<double>& g1, const int numParticles, const int weightMask)
{
	for (int pIndex = 0; pIndex < numParticles; pIndex++)
	{
		for (int cIndex = 0; cIndex < weightMask; cIndex++)
		{
			int modeIndex = pIndex * weightMask * numModes2D + cIndex * numModes2D;
			int mapIndex = pIndex * weightMask + cIndex;

 			scalarModes[modeIndex] = 1.0;
			scalarModes[modeIndex + 1] = x0[mapIndex];
			scalarModes[modeIndex + 2] = x1[mapIndex];
			scalarModes[modeIndex + 3] = x0[mapIndex] * x1[mapIndex];
			scalarModes[modeIndex + 4] = g0[mapIndex];
			scalarModes[modeIndex + 5] = g1[mapIndex];
			scalarModes[modeIndex + 6] = g0[mapIndex] * x1[mapIndex];
			scalarModes[modeIndex + 7] = g1[mapIndex] * x0[mapIndex];
			scalarModes[modeIndex + 8] = g0[mapIndex] * g1[mapIndex];
		}
	}
}

void PolyPIC::CalculateScalarModes3D(std::vector<double>& scalarModes, const std::vector<double>& x0, const std::vector<double>& x1, const std::vector<double>& x2,
									 const std::vector<double>& g0, const std::vector<double>& g1, const std::vector<double>& g2, const int numParticles)
{
	for (int pIndex = 0; pIndex < numParticles; pIndex++)
	{
		int modeIndex = pIndex * numModes3D;

		scalarModes[modeIndex] = 1.0;
		scalarModes[modeIndex + 1] = x0[pIndex];
		scalarModes[modeIndex + 2] = x1[pIndex];
		scalarModes[modeIndex + 3] = x2[pIndex];
		scalarModes[modeIndex + 4] = x0[pIndex] * x1[pIndex];
		scalarModes[modeIndex + 5] = x0[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 6] = x1[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 7] = x0[pIndex] * x1[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 8] = g0[pIndex];
		scalarModes[modeIndex + 9] = g1[pIndex];
		scalarModes[modeIndex + 10] = g2[pIndex];
		scalarModes[modeIndex + 11] = g0[pIndex] * g1[pIndex];
		scalarModes[modeIndex + 12] = g1[pIndex] * g2[pIndex];
		scalarModes[modeIndex + 13] = g0[pIndex] * g2[pIndex];
		scalarModes[modeIndex + 14] = g0[pIndex] * g1[pIndex] * g2[pIndex];
		scalarModes[modeIndex + 15] = g0[pIndex] * x1[pIndex];
		scalarModes[modeIndex + 16] = g0[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 17] = g0[pIndex] * x1[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 18] = g1[pIndex] * x0[pIndex];
		scalarModes[modeIndex + 19] = g1[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 20] = g1[pIndex] * x0[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 21] = g2[pIndex] * x0[pIndex];
		scalarModes[modeIndex + 22] = g2[pIndex] * x1[pIndex];
		scalarModes[modeIndex + 23] = g2[pIndex] * x0[pIndex] * x1[pIndex];
		scalarModes[modeIndex + 24] = g0[pIndex] * g1[pIndex] * x2[pIndex];
		scalarModes[modeIndex + 25] = g0[pIndex] * g2[pIndex] * x1[pIndex];
		scalarModes[modeIndex + 26] = g1[pIndex] * g2[pIndex] * x0[pIndex];
	}
}

void PolyPIC::CalculateCoefficientScales2D(std::vector<double>& polypicCoefficientScales, const double invDeltaX)
{
	polypicCoefficientScales.resize(numModes2D);
	
	// See techdoc for following constants.
	// Constants are calculated as 1 / presentedvalue.

	const double invDeltaXSqr = invDeltaX * invDeltaX;
	const double invDeltaXPow4 = invDeltaXSqr * invDeltaXSqr;

	polypicCoefficientScales[0] = 1;
	polypicCoefficientScales[1] = invDeltaXSqr * 4;
	polypicCoefficientScales[2] = invDeltaXSqr * 4;
	polypicCoefficientScales[3] = invDeltaXPow4 * 16;
	polypicCoefficientScales[4] = invDeltaXSqr * 0.5;
	polypicCoefficientScales[5] = invDeltaXSqr * 0.5;
	polypicCoefficientScales[6] = invDeltaXPow4;
	polypicCoefficientScales[7] = invDeltaXPow4;
	polypicCoefficientScales[8] = invDeltaXPow4 * 0.25;
}

void PolyPIC::CalculateCoefficientScales3D(std::vector<double>& polypicCoefficientScales, const double invDeltaX)
{
	polypicCoefficientScales.resize(numModes3D);

	// See techdoc for following constants.
	// Constants are calculated as 1 / presentedvalue.

	const double invDeltaXSqr = invDeltaX * invDeltaX;
	const double invDeltaXPow4 = invDeltaXSqr * invDeltaXSqr;
	const double invDeltaXPow6 = invDeltaXPow4 * invDeltaXSqr;

	polypicCoefficientScales[0] = 1;
	polypicCoefficientScales[1] = invDeltaXSqr * 4;
	polypicCoefficientScales[2] = invDeltaXSqr * 4;
	polypicCoefficientScales[3] = invDeltaXSqr * 4;
	polypicCoefficientScales[4] = invDeltaXPow4 * 16;
	polypicCoefficientScales[5] = invDeltaXPow4 * 16;
	polypicCoefficientScales[6] = invDeltaXPow4 * 16;
	polypicCoefficientScales[7] = invDeltaXPow6 * 64;
	polypicCoefficientScales[8] = invDeltaXSqr * 0.5;
	polypicCoefficientScales[9] = invDeltaXSqr * 0.5;
	polypicCoefficientScales[10] = invDeltaXSqr * 0.5;
	polypicCoefficientScales[11] = invDeltaXPow4 * 0.25;
	polypicCoefficientScales[12] = invDeltaXPow4 * 0.25;
	polypicCoefficientScales[13] = invDeltaXPow4 * 0.25;
	polypicCoefficientScales[14] = invDeltaXPow6 * 0.125;
	polypicCoefficientScales[15] = invDeltaXPow4 * 2;
	polypicCoefficientScales[16] = invDeltaXPow4 * 2;
	polypicCoefficientScales[17] = invDeltaXPow6 * 8;
	polypicCoefficientScales[18] = invDeltaXPow4 * 2;
	polypicCoefficientScales[19] = invDeltaXPow4 * 2;
	polypicCoefficientScales[20] = invDeltaXPow6 * 8;
	polypicCoefficientScales[21] = invDeltaXPow4 * 2;
	polypicCoefficientScales[22] = invDeltaXPow4 * 2;
	polypicCoefficientScales[23] = invDeltaXPow6 * 8;
	polypicCoefficientScales[24] = invDeltaXPow6;
	polypicCoefficientScales[25] = invDeltaXPow6;
	polypicCoefficientScales[26] = invDeltaXPow6;
}

void PolyPIC::Contribution(const std::vector<double>& scalarModes, const std::vector<double>& coefficients, std::vector<double>& contributions,
						   const int numScalarModes, const int numParticles, const int weightMask, const bool threeD)
{
	int maxModes = threeD ? numModes3D : numModes2D;

	if (numScalarModes <= 0 || numScalarModes > maxModes)
	{
		// TODO: error
		return;
	}

	std::fill(contributions.begin(), contributions.end(), 0.0);

	for (int pIndex = 0; pIndex < numParticles; pIndex++)
	{
		for (int wIndex = 0; wIndex < weightMask; wIndex++)
		{
			int contributionIndex = pIndex * weightMask + wIndex;

			for (int mode = 0; mode < numScalarModes; mode++)
			{
				int modeIndex = contributionIndex * maxModes + mode;
				int coefficientIndex = pIndex * maxModes + mode;

				contributions[contributionIndex] += coefficients[coefficientIndex] * scalarModes[modeIndex];
			}
		}
	}
}

void PolyPIC::CalculateNodeCoefficients2D(std::vector<double>& polypicCoefficients, const std::vector<double>& polypicCoefficientScales,
										  const std::vector<double>& x0, const std::vector<double>& x1,
										  const std::vector<double>& g0, const std::vector<double>& g1,
										  const std::vector<double>& velocity, const std::vector<int>& idx,
										  const double invDeltaX, const int numParticles, const int weightMask)
{
	std::fill(polypicCoefficients.begin(), polypicCoefficients.end(), 0.0);

	for (int pIndex = 0; pIndex < numParticles * weightMask; pIndex++)
	{
		int coefficientIndex = static_cast<int>(std::floor(pIndex / weightMask)) * numModes2D;

		const double weight0 = Interpolation::QuadBSpline(x0[pIndex] * invDeltaX); // weight for x axis.
		const double weight1 = Interpolation::QuadBSpline(x1[pIndex] * invDeltaX); // y axis.

		double weight = weight0 * weight1;
		
		const int idx0 = idx[pIndex] % 4;                                      // This 4.0 is based on a 4x4 weightMask being used.
		const int idx1 = static_cast<int>(std::floor(idx[pIndex] / 4.0)) % 4;  // This 4.0 is based on a 4x4 weightMask being used.

		const double exp0 = std::pow(-2.0, idx0 % 2); // TODO: is this right for a 4x4 mask, or should it output a 0,1,1,0 type mask? rather than 0,1,0,1?
		const double exp1 = std::pow(-2.0, idx1 % 2); // TODO: is this right for a 4x4 mask, or should it output a 0,1,1,0 type mask? rather than 0,1,0,1?

		polypicCoefficients[coefficientIndex] += polypicCoefficientScales[0] * velocity[pIndex] * weight;
		polypicCoefficients[coefficientIndex + 1] += polypicCoefficientScales[1] * velocity[pIndex] * weight * x0[pIndex];
		polypicCoefficients[coefficientIndex + 2] += polypicCoefficientScales[2] * velocity[pIndex] * weight * x1[pIndex];
		polypicCoefficients[coefficientIndex + 3] += polypicCoefficientScales[3] * velocity[pIndex] * weight * x0[pIndex] * x1[pIndex];
		polypicCoefficients[coefficientIndex + 4] += polypicCoefficientScales[4] * velocity[pIndex] * weight1 * exp0;
		polypicCoefficients[coefficientIndex + 5] += polypicCoefficientScales[5] * velocity[pIndex] * weight0 * exp1;
		polypicCoefficients[coefficientIndex + 6] += polypicCoefficientScales[6] * velocity[pIndex] * weight1 * x1[pIndex] * exp1;
		polypicCoefficients[coefficientIndex + 7] += polypicCoefficientScales[7] * velocity[pIndex] * weight0 * x0[pIndex] * exp0;
		polypicCoefficients[coefficientIndex + 8] += polypicCoefficientScales[8] * velocity[pIndex] * exp0 * exp1;
	}
}

void PolyPIC::CalculateNodeCoefficients3D(std::vector<double>& polypicCoefficients, const std::vector<double>& polypicCoefficientScales,
										  const std::vector<double>& x0, const std::vector<double>& x1, const std::vector<double>& x2,
										  const std::vector<double>& g0, const std::vector<double>& g1, const std::vector<double>& g2,
										  const double velocity, const double invDeltaX, const int idx, const int numParticles)
{
	for (int index = 0; index < numParticles; index++)
	{
		const double weight0 = Interpolation::QuadBSpline(x0[index] * invDeltaX); // weight for x axis.
		const double weight1 = Interpolation::QuadBSpline(x1[index] * invDeltaX); // y axis.
		const double weight2 = Interpolation::QuadBSpline(x2[index] * invDeltaX); // z axis.

		const double weight = weight0 * weight1 * weight2;

		const int idx0 = static_cast<int>(std::fmod(idx, 4.0));
		const int idx1 = static_cast<int>(std::fmod(static_cast<int>(std::floor(idx / 4.0)), 4.0));
		const int idx2 = static_cast<int>(std::fmod(static_cast<int>(std::floor(idx / 16.0)), 4.0));

		const double exp0 = std::pow(-2.0, idx0 % 2);
		const double exp1 = std::pow(-2.0, idx1 % 2);
		const double exp2 = std::pow(-2.0, idx2 % 2);

		polypicCoefficients[index * numModes3D] = polypicCoefficientScales[0] * velocity * weight;
		polypicCoefficients[index * numModes3D + 1] = polypicCoefficientScales[1] * velocity * weight * x0[index];
		polypicCoefficients[index * numModes3D + 2] = polypicCoefficientScales[2] * velocity * weight * x1[index];
		polypicCoefficients[index * numModes3D + 3] = polypicCoefficientScales[3] * velocity * weight * x2[index];
		polypicCoefficients[index * numModes3D + 4] = polypicCoefficientScales[4] * velocity * weight * x0[index] * x1[index];
		polypicCoefficients[index * numModes3D + 5] = polypicCoefficientScales[5] * velocity * weight * x1[index] * x2[index];
		polypicCoefficients[index * numModes3D + 6] = polypicCoefficientScales[6] * velocity * weight * x0[index] * x2[index];
		polypicCoefficients[index * numModes3D + 7] = polypicCoefficientScales[7] * velocity * weight * x0[index] * x1[index] * x2[index];
		polypicCoefficients[index * numModes3D + 8] = polypicCoefficientScales[8] * velocity * weight1 * weight2 * exp0;
		polypicCoefficients[index * numModes3D + 9] = polypicCoefficientScales[9] * velocity * weight0 * weight2 * exp1;
		polypicCoefficients[index * numModes3D + 10] = polypicCoefficientScales[10] * velocity * weight0 * weight1 * exp2;
		polypicCoefficients[index * numModes3D + 11] = polypicCoefficientScales[11] * velocity * weight2 * exp0 * exp1;
		polypicCoefficients[index * numModes3D + 12] = polypicCoefficientScales[12] * velocity * weight1 * exp0 * exp2;
		polypicCoefficients[index * numModes3D + 13] = polypicCoefficientScales[13] * velocity * weight0 * exp1 * exp2;
		polypicCoefficients[index * numModes3D + 14] = polypicCoefficientScales[14] * velocity * exp0 * exp1 * exp2;
		polypicCoefficients[index * numModes3D + 15] = polypicCoefficientScales[15] * velocity * weight1 * weight2 * x1[index] * exp0;
		polypicCoefficients[index * numModes3D + 16] = polypicCoefficientScales[16] * velocity * weight1 * weight2 * x2[index] * exp0;
		polypicCoefficients[index * numModes3D + 17] = polypicCoefficientScales[17] * velocity * weight1 * weight2 * x1[index] * x2[index] * exp0;
		polypicCoefficients[index * numModes3D + 18] = polypicCoefficientScales[18] * velocity * weight0 * weight2 * x0[index] * exp1;
		polypicCoefficients[index * numModes3D + 19] = polypicCoefficientScales[19] * velocity * weight0 * weight2 * x2[index] * exp1;
		polypicCoefficients[index * numModes3D + 20] = polypicCoefficientScales[20] * velocity * weight0 * weight2 * x0[index] * x2[index] * exp1;
		polypicCoefficients[index * numModes3D + 21] = polypicCoefficientScales[21] * velocity * weight0 * weight1 * x0[index] * exp2;
		polypicCoefficients[index * numModes3D + 22] = polypicCoefficientScales[22] * velocity * weight0 * weight1 * x1[index] * exp2;
		polypicCoefficients[index * numModes3D + 23] = polypicCoefficientScales[23] * velocity * weight0 * weight1 * x0[index] * x1[index] * exp2;
		polypicCoefficients[index * numModes3D + 24] = polypicCoefficientScales[24] * velocity * weight2 * x2[index] * exp0 * exp1;
		polypicCoefficients[index * numModes3D + 25] = polypicCoefficientScales[25] * velocity * weight1 * x1[index] * exp0 * exp2;
		polypicCoefficients[index * numModes3D + 26] = polypicCoefficientScales[26] * velocity * weight0 * x0[index] * exp1 * exp2;
	}
}

void PolyPIC::G(const std::vector<double>& input, std::vector<double>& output, const double deltaX)
{
	const double deltaXSqr(deltaX * deltaX);
	const double invDeltaX(1 / deltaX);
	const double invDeltaXSqr(invDeltaX * invDeltaX);

	const double c = deltaXSqr * 0.25;

	for (int index = 0; index < input.size(); index++)
	{
		const double weight = Interpolation::QuadBSpline(input[index] * invDeltaX);

		const double a = weight * weight;
		const double b = input[index] * (deltaXSqr - 4.0 * input[index] * input[index]) * weight * invDeltaXSqr;
		
		output[index] = a - b - c;
	}
}