#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

// App includes
#include "Shader.h"

// Simulation includes
#include "Simulation.h"

class OpenGLApp
{
public:
	OpenGLApp(int width, int height);
	~OpenGLApp();

	void LoadScene(const char* scenePath);

	void Run();
	void Render();

private:

	void ProcessInput(GLFWwindow* window);

	void OutputSimulationState(int iteration);

	void RenderParticles();
	void RenderGrid();
	void RenderSolids();

	void SetColorCellType(Shader& shader, const SimulationData& simData);
	void SetColorVelocity(Shader& shader, const SimulationData& simData);
	void SetColorDivergence(Shader& shader, const SimulationData& simData);
	void SetColorPressure(Shader& shader, const SimulationData& simData);
	void SetColorSolidPhi(Shader& shader, const SimulationData& simData);
	void SetColorDensityError(Shader& shader, const SimulationData& simData);

	GLFWwindow* mWindow;
	Shader mShader;

	int mWidth;
	int mHeight;

	Simulation mSimulation;
};