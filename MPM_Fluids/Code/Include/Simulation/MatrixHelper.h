#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Sparse>

namespace MatrixHelper
{
	void QRDecomposition2(const glm::dmat2& input, glm::dmat2& Q, glm::dmat2& R);
	void QRDecomposition3(const glm::dmat3& input, glm::dmat3& Q, glm::dmat3& R);

	glm::dmat2 DiagonalFromVec2(const glm::dvec2& input);
	glm::dmat3 DiagonalFromVec3(const glm::dvec3& input);

	void EigenDiagonalFromVecX(const std::vector<double>& input, Eigen::SparseMatrix<double>& output);
}