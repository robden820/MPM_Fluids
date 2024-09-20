#pragma once

#include <vector>
#include <glm/glm.hpp>

// TODO: initially we will assume the solid can only be an axis aligned cube.
// TODO: initially all solid objects are stationary.
class Solid
{
public:
	Solid() = default;
	Solid(const std::vector<float>& vertices, const std::vector<float>& normals); // TODO; when loading solids from obj this should also take normals and faces.
	~Solid() = default;

	const std::vector<float>& GetVertices() const { return mVertices; }
	const std::vector<float>& GetNormals() const { return mNormals; }
	const std::vector<int>& GetIndices() const { return mIndices; }

	const size_t GetNumIndices() const { return mIndices.size(); }

	// Functions useful for rendering
	const size_t GetSizeOfVertices() const { return sizeof(float) * mVertices.size(); }
	float* GetVerticesAsFloatArray() { return mVertices.data(); }
	const size_t GetSizeOfIndices() const { return sizeof(int) * mIndices.size(); }
	int* GetIndiciesAsIntArray() { return mIndices.data(); }

	void UpdateBounds();

	bool PointInBounds(const glm::dvec2& point) const; // Check if a point lies inside the bounding box
	bool ContainsPoint(const glm::dvec2& point) const;
	void ProjectOutPoint(glm::dvec2& point, glm::dvec2& velocity) const; // Updates the input position to give the nearest point outside the bounding box. Also sets the velocity in the projection direction to zero

	void GetDistanceNormal(const glm::dvec2& point, double& distOut, glm::dvec2& normOut) const;
	double GetClosestDistance(const glm::dvec2& point) const; // returns the closest distance from the point to the solid

	const double GetLeftBound() const { return mLeftBound; }
	const double GetRightBound() const { return mRightBound; }
	const double GetUpperBound() const { return mUpperBound; }
	const double GetLowerBound() const { return mLowerBound; }

private:

	// Follow same format as .obj files
	std::vector<float> mVertices;
	std::vector<float> mNormals;
	std::vector<int> mIndices;

	// Bounding box - axis aligned cube.
	double mLeftBound;
	double mRightBound;
	double mUpperBound;
	double mLowerBound;
};