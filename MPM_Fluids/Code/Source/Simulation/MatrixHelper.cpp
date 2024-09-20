#include "MatrixHelper.h"

void MatrixHelper::QRDecomposition2(const glm::dmat2& input, glm::dmat2& Q, glm::dmat2& R)
{
	const glm::dvec2 e0 = glm::normalize(input[0]);
	const glm::dvec2 e1 = glm::normalize(input[1] - glm::dot(input[1], e0) * e0);

	Q = glm::dmat2(e0, e1);
	R = glm::dmat2(glm::dot(input[0], e0), 0,                         // col 0
				   glm::dot(input[1], e0), glm::dot(input[1], e1));   // col 1
}

void MatrixHelper::QRDecomposition3(const glm::dmat3& input, glm::dmat3& Q, glm::dmat3& R)
{
	const glm::dvec3 e0 = glm::normalize(input[0]);
	const glm::dvec3 e1 = glm::normalize(input[1] - glm::dot(input[1], e0) * e0);
	const glm::dvec3 e2 = glm::normalize(input[2] - glm::dot(input[2], e0) * e0 - glm::dot(input[2], e1) * e1);

	Q = glm::dmat3(e0, e1, e2);
	R = glm::dmat3(glm::dot(input[0], e0), 0, 0,                                              // col 0
				   glm::dot(input[1], e0), glm::dot(input[1], e1), 0,                         // col 1
				   glm::dot(input[2], e0), glm::dot(input[2], e1), glm::dot(input[2], e2));   // col 2
}

glm::dmat2 MatrixHelper::DiagonalFromVec2(const glm::dvec2& input)
{
	return glm::dmat2(input.x, 0.0,   // column 1
					  0.0, input.y);  // column 2
}

glm::dmat3 MatrixHelper::DiagonalFromVec3(const glm::dvec3& input)
{
	return glm::dmat3(input.x, 0.0, 0.0,   // column 1
					  0.0, input.y, 0.0,   // column 2
					  0.0, 0.0, input.z);  // column 3
}

void MatrixHelper::EigenDiagonalFromVecX(const std::vector<double>& input, Eigen::SparseMatrix<double>& output)
{
	std::vector<Eigen::Triplet<double>> diagonals;

	for (int i = 0; i < input.size(); i++)
	{
		diagonals.push_back(Eigen::Triplet<double>(i, i, input[i]));
	}

	output.setFromTriplets(diagonals.begin(), diagonals.end());
}