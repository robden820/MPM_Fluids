
#include "Solid.h"

#include <algorithm>

Solid::Solid(const std::vector<float>& vertices, const std::vector<float>& normals)
	: mVertices(vertices), mNormals(normals)
{

	mIndices = {0, 2, 3, // left
				0, 3, 1,
				2, 4, 6, // bottom
				2, 0, 4,
				3, 6, 7, // back
				3, 2, 6,
				5, 7, 6, // right
				5, 6, 4, 
				0, 5, 4, // front
				0, 1, 5,
				1, 3, 7, // top
				1, 7, 5
			};

	UpdateBounds();
}

void Solid::UpdateBounds()
{
	int numVertices = static_cast<int>(mVertices.size() / 3);

	mLeftBound = mVertices[0];
	mRightBound = mVertices[0];

	mUpperBound = mVertices[1];
	mLowerBound = mVertices[1];

	for (int v = 1; v < numVertices; v++)
	{
		int vIndex = 3 * v;

		if (mVertices[vIndex] < mLeftBound) mLeftBound = mVertices[vIndex];
		else if (mVertices[vIndex] > mRightBound) mRightBound = mVertices[vIndex];

		if (mVertices[vIndex + 1] < mLowerBound) mLowerBound = mVertices[vIndex + 1];
		else if (mVertices[vIndex + 1] > mUpperBound) mUpperBound = mVertices[vIndex + 1];
	}
}

bool Solid::PointInBounds(const glm::dvec2& point) const
{
	if (point.x < mLeftBound) return false;
	if (point.x > mRightBound) return false;

	if (point.y < mLowerBound) return false;
	if (point.y > mUpperBound) return false;

	return true;
}

bool Solid::ContainsPoint(const glm::dvec2& point) const
{
	glm::dvec2 p0(mVertices[0], mVertices[1]);  // Top front right
	glm::dvec2 p1(mVertices[3], mVertices[4]);  // Bottom front right
	glm::dvec2 p2(mVertices[9], mVertices[10]); // Bottom front left
	glm::dvec2 p3(mVertices[6], mVertices[7]);  // Top front left

	glm::dvec2 n0(mNormals[15], mNormals[16]); // +ve x
	glm::dvec2 n1(mNormals[9], mNormals[10]);  // -ve y
	glm::dvec2 n2(mNormals[3], mNormals[4]);   // -ve x
	glm::dvec2 n3(mNormals[0], mNormals[1]); // +ve y

	if (glm::dot(point - p0, n0) > 0.0) return false;
	if (glm::dot(point - p1, n1) > 0.0) return false;
	if (glm::dot(point - p2, n2) > 0.0) return false;
	if (glm::dot(point - p3, n3) > 0.0) return false;

	return true;
}

void Solid::GetDistanceNormal(const glm::dvec2& point, double& distOut, glm::dvec2& normOut) const
{
	glm::dvec2 p0(mVertices[0], mVertices[1]);  // Top front right
	glm::dvec2 p1(mVertices[3], mVertices[4]);  // Bottom front right
	glm::dvec2 p2(mVertices[9], mVertices[10]); // Bottom front left
	glm::dvec2 p3(mVertices[6], mVertices[7]);  // Top front left

	glm::dvec2 diff0 = point - p0;
	glm::dvec2 diff1 = point - p1;
	glm::dvec2 diff2 = point - p2;
	glm::dvec2 diff3 = point - p3;

	if (point.x >= p0.x && point.y >= p0.y)
	{
		distOut = glm::length(diff0);
		normOut = diff0;
		return;
	}
	else if (point.x >= p1.x && point.y <= p1.y)
	{
		distOut = glm::length(diff1);
		normOut = diff1;
		return;
	}
	else if (point.x <= p2.x && point.y <= p2.y)
	{
		distOut = glm::length(diff2);
		normOut = diff2;
		return;
	}
	else if (point.x <= p3.x && point.y >= p3.y)
	{
		distOut = glm::length(diff3);
		normOut = diff3;
		return;
	}

	glm::dvec2 n0(mNormals[15], mNormals[16]); // +ve x
	glm::dvec2 n1(mNormals[9], mNormals[10]);  // -ve y
	glm::dvec2 n2(mNormals[3], mNormals[4]);   // -ve x
	glm::dvec2 n3(mNormals[0], mNormals[1]); // +ve y

	glm::dvec2 edge0 = p1 - p0;
	glm::dvec2 edge1 = p2 - p1;
	glm::dvec2 edge2 = p3 - p2;
	glm::dvec2 edge3 = p0 - p3;

	double dist0 = glm::length(diff0 - glm::dot(diff0, edge0) / glm::dot(edge0, edge0) * edge0);
	double dist1 = glm::length(diff1 - glm::dot(diff1, edge1) / glm::dot(edge1, edge1) * edge1);
	double dist2 = glm::length(diff2 - glm::dot(diff2, edge2) / glm::dot(edge2, edge2) * edge2);
	double dist3 = glm::length(diff3 - glm::dot(diff3, edge3) / glm::dot(edge3, edge3) * edge3);
	
	if (point.x >= mRightBound)
	{
		distOut = dist0;
		normOut = n0;
	}
	else if (point.y <= mLowerBound)
	{
		distOut = dist1;
		normOut = n1;
	}
	else if (point.x <= mLeftBound)
	{
		distOut = dist2;
		normOut = n2;
	}
	else if (point.y >= mUpperBound)
	{
		distOut = dist3;
		normOut = n3;
	}
	else if (dist0 < dist1 && dist0 < dist2 && dist0 < dist3)
	{
		distOut = dist0;
		normOut = n0;
	}
	else if (dist1 < dist0 && dist1 < dist2 && dist1 < dist3)
	{
		distOut = dist1;
		normOut = n1;
	}
	else if (dist2 < dist0 && dist2 < dist1 && dist2 < dist3)
	{
		distOut = dist2;
		normOut = n2;
	}
	else
	{
		distOut = dist3;
		normOut = n3;
	}
}

double Solid::GetClosestDistance(const glm::dvec2& point) const
{
	glm::dvec2 p0(mVertices[0], mVertices[1]);     // Top front right
	glm::dvec2 p1(mVertices[3], mVertices[4]);     // Bottom front right
	glm::dvec2 p2(mVertices[6], mVertices[7]);     // Top front left
	glm::dvec2 p3(mVertices[9], mVertices[10]);   // Bottom front left

	// -If point is outside
	// --If point is closest to a corner
	if (point.x >= p0.x && point.y >= p0.y) return glm::length(point - p0);
	else if (point.x >= p1.x && point.y <= p1.y) return glm::length(point - p1);
	else if (point.x <= p2.x && point.y >= p2.y) return glm::length(point - p2);
	else if (point.x <= p3.x && point.y <= p3.y) return glm::length(point - p3);

	// --If point is closest to an edge
	if (point.x > mRightBound && point.y > mUpperBound) return glm::length(glm::dvec2(point.x - p0.x, point.y - p0.y));
	else if (point.x > mRightBound && point.y < mLowerBound) return glm::length(glm::dvec2(point.x - p0.x, point.y - p3.y));
	else if (point.x < mLeftBound && point.y < mLowerBound) return glm::length(glm::dvec2(point.x - p3.x, point.y - p3.y));
	else if (point.x < mLeftBound && point.y > mUpperBound) return glm::length(glm::dvec2(point.x - p3.x, point.y - p0.y));

	// --If point is closest to a face
	if (point.x > mRightBound) return abs(point.x - p0.x);
	else if (point.x < mLeftBound) return abs(point.x - p3.x);
	else if (point.y > mUpperBound) return abs(point.y - p0.y);
	else if (point.y < mLowerBound) return abs(point.y - p3.y);

	// iIf point is inside the solid
	double distRight = abs(point.x - p0.x);
	double distLeft = abs(point.x - p3.x);
	double distTop = abs(point.y - p0.y);
	double distBottom = abs(point.y - p3.y);

	return std::min({ distRight, distLeft, distTop, distBottom});
}

void Solid::ProjectOutPoint(glm::dvec2& point, glm::dvec2& velocity) const
{
	double dist;
	glm::dvec2 norm;

	GetDistanceNormal(point, dist, norm);
	// Now we have the edge distances, project out in direction.

	point += norm * dist;
	velocity += norm * dist;

}