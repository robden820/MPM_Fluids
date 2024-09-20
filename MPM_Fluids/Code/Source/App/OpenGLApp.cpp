
#include <iostream>
#include <fstream>
#include <sstream>

#include "OpenGLApp.h"
#include "SceneParser.h"
#include "StateWriter.h"
#include "StateLoader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <onetbb/oneapi/tbb.h>

#include "Solid.h"

#define RENDER_GRID 0
#define RENDER_CELL_TYPES 1
#define RENDER_CELL_VELOCITY 0
#define RENDER_DIVERGENCE 1
#define RENDER_PRESSURE 0
#define RENDER_CURVATURE 0
#define RENDER_PHI 0
#define RENDER_PHI_GRAD 0
#define RENDER_SOLID_PHI 0
#define RENDER_DENSITY 0
#define RENDER_DENSITY_ERROR 0
#define RENDER_LIQUID_FRACTION 0

#define USE_MPM 0

#define LOAD_FROM_FILE 0
#define SAVE_TO_FILE 0
#define WRITE_SIMULATION 0
#define WRITE_RENDER 0
#define WRITE_META 0


OpenGLApp::OpenGLApp(int width, int height)
    : mWidth(width), mHeight(height),
#if USE_MPM
    mSimulation(Simulation(SimType::MPM))
#else
    mSimulation(Simulation(SimType::PIC))
#endif
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(mWidth, mHeight, "MPM Fluids", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        // TODO: fail gracefully.
    }
    mWindow = window;
    glfwMakeContextCurrent(mWindow);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        // TODO: fail gracefully.
    }

    glViewport(0, 0, mWidth, mHeight);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    mShader.LinkShaders("Code/shaders/vertex.vs", "Code/shaders/fragment.fs");
    mShader.Use();

    LoadScene("filepath");
}

OpenGLApp::~OpenGLApp()
{
    glfwTerminate();
}

void OpenGLApp::LoadScene(const char* scenePath)
{
    std::cout << "Loading scene..." << std::endl;
    std::cout << "Setting simulation data..." << std::endl;
    // TO DO: this filepath from command line arguments
#if USE_MPM
    SceneParser scene("Scenes/MPM_scene.json", mSimulation.GetSimulationData());
    mSimulation.UpdateSimulationDataValues();
#else 
    SceneParser scene("Scenes/PIC_scene.json", mSimulation.GetSimulationData());
    mSimulation.UpdateSimulationDataValues();
#endif
}

void OpenGLApp::Run()
{
    //InitShaderTransforms();
    // create transformations
    glm::mat4 view = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    glm::mat4 projection = glm::mat4(1.0f);
    projection = glm::perspective(glm::radians(45.0f), (float)mWidth / (float)mHeight, 0.1f, 200.0f);
    view = glm::translate(view, glm::vec3(0.0f, 0.0f, -100.0f));
    // pass transformation matrices to the shader
    mShader.SetMatrix4("projection", projection);
    mShader.SetMatrix4("view", view);

    std::cout << "Beginning simulation" << std::endl;
    int iterations = 0;
    int itrCount = 5;

    // TODO: have command line options to load from a file.
#if LOAD_FROM_FILE
    std::array<char, 256> scenePath;
    sprintf_s(scenePath.data(), 256, "Output/test_small/scene_000047.txt");
    StateLoader loader(scenePath, mSimulation2D.GetSimulationData(), iterations);
    iterations *= itrCount;
    iterations++; //: this is to avoid immediately overwriting the input file.
#endif

    bool complete = false;

    // rendering loop
    while (!glfwWindowShouldClose(mWindow) && !complete)
    {
        // Deal with our inputs
        ProcessInput(mWindow);

        // Update simulation
        mSimulation.Update();
        complete = mSimulation.IsComplete();


#if SAVE_TO_FILE
        if (iterations % itrCount == 0) // TODO number of steps for output from command line?
        {
            OutputSimulationState(iterations / itrCount);
        }
#endif

        // Rendering
        Render();

        // Check and call events, swap buffers.
        glfwSwapBuffers(mWindow);
        glfwPollEvents();

        ++iterations;
    }
}

void OpenGLApp::Render()
{
    std::vector<float> vertices = {
            -0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f,  0.5f, -0.5f,
             0.5f,  0.5f, -0.5f,
            -0.5f,  0.5f, -0.5f,
            -0.5f, -0.5f, -0.5f,

            -0.5f, -0.5f,  0.5f,
             0.5f, -0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f,  0.5f,
            -0.5f, -0.5f,  0.5f,

            -0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f, -0.5f,
            -0.5f, -0.5f, -0.5f,
            -0.5f, -0.5f, -0.5f,
            -0.5f, -0.5f,  0.5f,
            -0.5f,  0.5f,  0.5f,

             0.5f,  0.5f,  0.5f,
             0.5f,  0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,

            -0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f,  0.5f,
             0.5f, -0.5f,  0.5f,
            -0.5f, -0.5f,  0.5f,
            -0.5f, -0.5f, -0.5f,

            -0.5f,  0.5f, -0.5f,
             0.5f,  0.5f, -0.5f,
             0.5f,  0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f, -0.5f
    };

    glClearColor(0.99f, 0.99f, 0.99f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // TODO: probably don't need to do this one every time.
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    
    // render boxes
    glBindVertexArray(VAO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    
#if RENDER_GRID
    RenderGrid();
#else
    RenderParticles();
    RenderSolids();
#endif
    

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void OpenGLApp::ProcessInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

void OpenGLApp::RenderParticles()
{
    SimulationData simData;

    simData = mSimulation.GetSimulationData();

    mShader.SetFloat("alpha", 1.0f);

    float maxVel = 100.0f;
    float minVel = 0.0f;

    for (int pIndex = 0; pIndex < simData.numParticles; pIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        glm::dvec3 pos = glm::dvec3(simData.particlePositions[pIndex], 0.0);
        model = glm::translate(model, pos);

        float vel = simData.particleVelocityX[pIndex] * simData.particleVelocityX[pIndex] + simData.particleVelocityY[pIndex] * simData.particleVelocityY[pIndex];
        float color = vel / maxVel;

        mShader.SetFloat("alpha", 0.8f);
        mShader.SetVector("color", glm::vec3(0.2f, 0.2f, color));
        glm::dvec3 particleScale = glm::dvec3(simData.cellSize * 0.35f);

        model = glm::scale(model, particleScale);
        mShader.SetMatrix4("model", model);

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::RenderGrid()
{
    SimulationData simData;

    simData = mSimulation.GetSimulationData();

    mShader.SetFloat("alpha", 0.5f);

#if RENDER_CELL_TYPES
    SetColorCellType(mShader, simData);
#elif RENDER_CELL_VELOCITY
    SetColorVelocity(mShader, simData);
#elif RENDER_DIVERGENCE
    SetColorDivergence(mShader, simData);
#elif RENDER_PRESSURE
    SetColorPressure(mShader, simData);
#elif RENDER_SOLID_PHI
    SetColorSolidPhi(mShader, simData);
#elif RENDER_DENSITY
    SetColorDensity(mShader, simData);
#elif RENDER_DENSITY_ERROR
    SetColorDensityError(mShader, simData);
#endif
}

void OpenGLApp::SetColorCellType(Shader& shader, const SimulationData& simData)
{
    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        model = glm::translate(model, glm::dvec3(simData.gridCellPositions[cIndex], 0.0));
        model = glm::scale(model, glm::dvec3(simData.cellSize));

        mShader.SetMatrix4("model", model);

        CellType cellType = simData.cellType[cIndex];

        if (cellType == CellType::eFIXED)
        {
            mShader.SetVector("color", glm::vec3(0.0f, 0.0f, 0.0f));
        }
        else if (cellType == CellType::eAIR)
        {
            mShader.SetVector("color", glm::vec3(0.0f, 1.0f, 0.0f));
        }
        else if (cellType == CellType::eFLUID)
        {
            mShader.SetVector("color", glm::vec3(0.0f, 0.0f, 1.0f));
        }
        else
        {
            mShader.SetVector("color", glm::vec3(1.0f, 0.0f, 1.0f));
        }

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::SetColorVelocity(Shader& shader, const SimulationData& simData)
{
    double minVel = 1000.0;
    double maxVel = -1000.0;

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        double vel = glm::length(glm::dvec2(simData.gridVelX[cIndex], simData.gridVelY[cIndex]));

        if (vel < minVel) minVel = vel;
        else if (vel > maxVel) maxVel = vel;
    }

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        model = glm::translate(model, glm::dvec3(simData.gridCellPositions[cIndex], 0.0));
        model = glm::scale(model, glm::dvec3(simData.cellSize));

        mShader.SetMatrix4("model", model);

        double vel = glm::length(glm::dvec2(simData.gridVelX[cIndex], simData.gridVelY[cIndex]));

        double v = (vel - minVel) / (maxVel - minVel);
        mShader.SetVector("color", glm::vec3(v, v, v));

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::SetColorDivergence(Shader& shader, const SimulationData& simData)
{
    double minDiv = 1000.0;
    double maxDiv = -1000.0;

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        if (simData.gridDivergence[cIndex] < minDiv) minDiv = simData.gridDivergence[cIndex];
        else if (simData.gridDivergence[cIndex] > maxDiv) maxDiv = simData.gridDivergence[cIndex];
    }

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        model = glm::translate(model, glm::dvec3(simData.gridCellPositions[cIndex], 0.0));
        model = glm::scale(model, glm::dvec3(simData.cellSize));

        mShader.SetMatrix4("model", model);
    
        double d = (simData.gridDivergence[cIndex] - minDiv) / (maxDiv - minDiv);
        mShader.SetVector("color", glm::vec3(d, d, d));

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::SetColorPressure(Shader& shader, const SimulationData& simData)
{
    double minP = 1000.0;
    double maxP = -1000.0;

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        if (simData.cellType[cIndex] == CellType::eFIXED) continue;
        if (simData.gridPressure[cIndex] < minP) minP = simData.gridPressure[cIndex];
        else if (simData.gridPressure[cIndex] > maxP) maxP = simData.gridPressure[cIndex];
    }

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        model = glm::translate(model, glm::dvec3(simData.gridCellPositions[cIndex], 0.0));
        model = glm::scale(model, glm::dvec3(simData.cellSize));

        mShader.SetMatrix4("model", model);

        double d = (simData.gridPressure[cIndex] - minP) / (maxP - minP);
        mShader.SetVector("color", glm::vec3(d, d, d));

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::SetColorSolidPhi(Shader& shader, const SimulationData& simData)
{
    double minDistance = 1000000.0;
    double maxDistance = -1000000.0;

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        if (simData.gridSolidPhiX[cIndex] < minDistance) minDistance = simData.gridSolidPhiX[cIndex];
        if (simData.gridSolidPhiX[cIndex] > maxDistance) maxDistance = simData.gridSolidPhiX[cIndex];
    }

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        model = glm::translate(model, glm::dvec3(simData.gridCellPositions[cIndex], 0.0));
        model = glm::scale(model, glm::dvec3(simData.cellSize));

        mShader.SetMatrix4("model", model);

        double d = (simData.gridSolidPhiX[cIndex] - minDistance) / (maxDistance - minDistance);

        if (simData.gridSolidPhiX[cIndex] < 0.0)
        {
            mShader.SetVector("color", glm::vec3(1.0, d, 1.0));
        }
        else
        {
            mShader.SetVector("color", glm::vec3(d, d, 1.0));
        }

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::SetColorDensityError(Shader& shader, const SimulationData& simData)
{
    double minError = 10000000.0;
    double maxError = -10000000.0;

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        if (simData.gridDensityError[cIndex] < minError) minError = simData.gridDensityError[cIndex];
        if (simData.gridDensityError[cIndex] > maxError) maxError = simData.gridDensityError[cIndex];
    }

    for (int cIndex = 0; cIndex < simData.numGridCells; cIndex++)
    {
        glm::dmat4 model = glm::dmat4(1.0f);
        model = glm::translate(model, glm::dvec3(simData.gridCellPositions[cIndex], 0.0));
        model = glm::scale(model, glm::dvec3(simData.cellSize));

        mShader.SetMatrix4("model", model);

        double d = (simData.gridDensityError[cIndex] - minError) / (maxError - minError);
        if (simData.cellType[cIndex] == CellType::eFIXED)
        {
            mShader.SetVector("color", glm::vec3(d, 0.0f, d));
        }
        {
            mShader.SetVector("color", glm::vec3(d, d, d));
        }
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

void OpenGLApp::RenderSolids()
{
    SimulationData simData;

    simData = mSimulation.GetSimulationData();

    mShader.SetVector("color", glm::vec3(1.0f, 0.2f, 0.4f));
    mShader.SetFloat("alpha", 0.1f);

    // TODO: probably don't need to do this one every time.
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    unsigned int EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    // render boxes
    glBindVertexArray(VAO);

    for (int sIndex = 0; sIndex < simData.solidObjects.size(); sIndex++)
    {
        glBufferData(GL_ARRAY_BUFFER, simData.solidObjects[sIndex].GetSizeOfVertices(), simData.solidObjects[sIndex].GetVerticesAsFloatArray(), GL_STATIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, simData.solidObjects[sIndex].GetSizeOfIndices(), simData.solidObjects[sIndex].GetIndiciesAsIntArray(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glm::mat4 model = glm::mat4(1.0f);
        mShader.SetMatrix4("model", model);

        glDrawElements(GL_TRIANGLES, simData.solidObjects[sIndex].GetNumIndices(), GL_UNSIGNED_INT, 0);
    }
}

void OpenGLApp::OutputSimulationState(int iterations)
{
    std::cout << "--- Outputting to file ---" << std::endl;
    // TODO: don't create a separate statewriter for each of these, combine them into one
#if WRITE_SIMULATION
    StateWriter sceneSimulation(iterations, "2D_scene_sim", mSimulation2D.GetSimulationData(), WriteType::eSIMULATION);
#endif
#if WRITE_RENDER
    StateWriter sceneRender(iterations, "2D_scene_render", mSimulation2D.GetSimulationData(), WriteType::eRENDER);
#endif
#if WRITE_META
    StateWriter sceneMeta(iterations, "2D_scene_meta", mSimulation2D.GetSimulationData(), WriteType::eMETA);
#endif

}