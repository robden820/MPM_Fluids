
#include "SceneParser.h"

#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>

SceneParser::SceneParser(const char* scenePath, SimulationData& simData)
{
	/// Open scene file
	FILE* filePointer;
	errno_t error = fopen_s(&filePointer, scenePath, "rb");
	
	if (error)
	{
		std::cout << "ERROR:: UNABLE TO OPEN SCENE FILE FOR READING:\n" << std::endl;
		return;
		// TODO: descruct
	}

	// Read file to buffer
	std::array<char, 1024> sceneBuffer;
	rapidjson::FileReadStream inputStream(filePointer, sceneBuffer.data(), sizeof(sceneBuffer));

	// Parse JSON
	rapidjson::Document scene;
	scene.ParseStream(inputStream);
	fclose(filePointer);

	// Check document has been parsed successfully
	if (scene.HasParseError())
	{
		std::cout << "ERROR:: SCENE JSON NOT SUCCESSFULLY PARSED:\n" << std::endl;
		return;
		// TODO: descruct
	}

	if (scene.HasMember("numGridCells"))
	{
		int numGridCells = scene["numGridCells"].GetInt();

		simData.numGridCells = numGridCells;
		simData.gridCellPositions.reserve(numGridCells);
	}
	else
	{
		std::cout << "SCENE ERROR:: Number of grid cells not found in scene.json - must be specified with an integer tagged 'numGridCells'. \n" << std::endl;
	}

	if (scene.HasMember("numCellsWidth"))
	{
		int numCellsWidth = scene["numCellsWidth"].GetInt();

		simData.gridResolutionWidth = numCellsWidth;
	}
	else
	{
		std::cout << "SCENE ERROR:: Number of grid cells wide not found in scene.json - must be specified with an integer tagged 'numCellsWidth'. \n" << std::endl;
	}

	if (scene.HasMember("numCellsHeight"))
	{
		int numCellsHeight = scene["numCellsHeight"].GetInt();

		simData.gridResolutionHeight = numCellsHeight;
	}
	else
	{
		std::cout << "SCENE ERROR:: Number of grid cells high not found in scene.json - must be specified with an integer tagged 'numCellsHeight'. \n" << std::endl;
	}

	if (scene.HasMember("gridCellPositions"))
	{
		const rapidjson::Value& positions = scene["gridCellPositions"];

		for (rapidjson::SizeType i = 0; i < positions.Size(); i++)
		{
			simData.gridCellPositions.push_back(glm::vec2(positions[i][0].GetFloat(), positions[i][1].GetFloat()));
		}
	}
	else
	{
		std::cout << "SCENE ERROR:: Grid Cells positions not found in scene.json - must be specified with a 2D float array tagged 'gridCellPositions'. \n" << std::endl;
	}

	if (scene.HasMember("cellSize"))
	{
		float cellSize = scene["cellSize"].GetFloat();

		simData.cellSize = cellSize;
	}
	else
	{
		std::cout << "SCENE ERROR:: Cell Size not found in scene.json - must be specified with a float tagged 'cellSize'. \n" << std::endl;
	}

	if (scene.HasMember("domainMinX"))
	{
		float minX = scene["domainMinX"].GetFloat();

		simData.domainMinX = minX;
	}
	else
	{
		std::cout << "SCENE ERROR:: Minimum X not found in scene.json - must be specified with a float tagged 'domainMinX'. \n" << std::endl;
	}

	if (scene.HasMember("domainMaxX"))
	{
		float maxX = scene["domainMaxX"].GetFloat();

		simData.domainMaxX = maxX;
	}
	else
	{
		std::cout << "SCENE ERROR:: Maximum X not found in scene.json - must be specified with a float tagged 'domainMaxX'. \n" << std::endl;
	}

	if (scene.HasMember("domainMinY"))
	{
		float minY = scene["domainMinY"].GetFloat();

		simData.domainMinY = minY;
	}
	else
	{
		std::cout << "SCENE ERROR:: Minimum Y not found in scene.json - must be specified with a float tagged 'domainMinY'. \n" << std::endl;
	}

	if (scene.HasMember("domainMaxY"))
	{
		float maxY = scene["domainMaxY"].GetFloat();

		simData.domainMaxY = maxY;
	}
	else
	{
		std::cout << "SCENE ERROR:: Maximum Y not found in scene.json - must be specified with a float tagged 'domainMaxY'. \n" << std::endl;
	}

	if (scene.HasMember("numParticles"))
	{
		int numParticles = scene["numParticles"].GetInt();

		simData.numParticles = numParticles;
		simData.particlePositions.reserve(numParticles);
	}
	else
	{
		std::cout << "SCENE ERROR:: Number of particles not found in scene.json - must be specified with an integer tagged 'numParticles'. \n" << std::endl;
	}

	if (scene.HasMember("particlePositions"))
	{
		const rapidjson::Value& positions = scene["particlePositions"];

		for (rapidjson::SizeType i = 0; i < positions.Size(); i++)
		{
			simData.particlePositions.push_back(glm::vec2(positions[i][0].GetFloat(), positions[i][1].GetFloat()));
		}
	}
	else
	{
		std::cout << "SCENE ERROR:: Particle positions not found in scene.json - must be specified with a 2D float array tagged 'particlePositions'. \n" << std::endl;
	}

	if (scene.HasMember("particleVelocities"))
	{
		const rapidjson::Value& velocities = scene["particleVelocities"];

		for (rapidjson::SizeType i = 0; i < velocities.Size(); i++)
		{
			simData.particleVelocityX.push_back(velocities[i][0].GetFloat());
			simData.particleVelocityY.push_back(velocities[i][1].GetFloat());
		}
	}
	else
	{
		std::cout << "SCENE ERROR:: Particle velocities not found in scene.json - must be specified with a 2D float array tagged 'particleVelocities'. \n" << std::endl;
	}

	if (scene.HasMember("particleMass"))
	{
		const float mass = scene["particleMass"].GetFloat();

		simData.particleMass = mass;
	}
	else
	{
		std::cout << "SCENE ERROR:: Particle mass not found in scene.json - must be specified with a float array tagged 'particleMass'. \n" << std::endl;
	}

	if (scene.HasMember("solids"))
	{
		const rapidjson::Value& solids = scene["solids"];

		for (rapidjson::SizeType sIndex = 0; sIndex < solids.Size(); sIndex++)
		{
			std::vector<float> vertices(solids[sIndex].GetArray()[0].Size());
			std::vector<float> normals(solids[sIndex].GetArray()[1].Size());

			for (rapidjson::SizeType index = 0; index < solids[sIndex].GetArray()[0].Size(); index++)
			{
				vertices[index] = solids[sIndex][0][index].GetFloat();
			}
			for (rapidjson::SizeType index = 0; index < solids[sIndex].GetArray()[1].Size(); index++)
			{
				normals[index] = solids[sIndex][1][index].GetFloat();
			}

			Solid solid(vertices, normals);
			simData.solidObjects.push_back(solid);
		}
	}
	else
	{
		std::cout << "SCENE ERROR:: Solids not found in scene.json - must be specified with tag 'solids'. \n" << std::endl;
	}

	if (scene.HasMember("fluidDensity"))
	{
		float density = scene["fluidDensity"].GetFloat();

		simData.fluidDensity = density;
	}
	else
	{
		std::cout << "SCENE ERROR:: fluid density not found in scene.json - must be specified with an integer tagged 'fluidDensity'. \n" << std::endl;
	}

	if (scene.HasMember("simulationDuration"))
	{
		float duration = scene["simulationDuration"].GetFloat();

		simData.totalSimulationTime = duration;
	}
	else
	{
		std::cout << "SCENE ERROR:: total simulation time not found in scene.json - must be specified with an integer tagged 'simulationDuration'. \n" << std::endl;
	}

	if (scene.HasMember("deltaTime"))
	{
		float dt = scene["deltaTime"].GetFloat();

		simData.timeStep = dt;
	}
	else
	{
		std::cout << "SCENE ERROR:: deltaTime not found in scene.json - must be specified with an integer tagged 'deltaTime'. \n" << std::endl;
	}
}
