/*
* Copyright (c) Joey De Vries https://twitter.com/JoeyDeVriez
* https://creativecommons.org/licenses/by/4.0/
*
* https://learnopengl.com/
* https://github.com/JoeyDeVries/LearnOpenGL/blob/master/includes/learnopengl/camera.h
*/

#pragma once

#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader
{
public:
	/**
	*	Constructor will read and build the shader.
	*	These are the filepaths to the source code for each shader.
	*/
	Shader();
	~Shader();

	void LinkShaders(const char* vertexPath, const char* fragmentPath);
	/* Use and activate the shader */
	void Use();

	/* Uniform utility functions */
	void SetBool(const std::string& name, bool value) const;
	void SetInt(const std::string& name, int value) const;
	void SetFloat(const std::string& name, float value) const;
	void SetVector(const std::string& name, glm::vec3 value) const;
	void SetMatrix4(const std::string& name, glm::mat4 value) const;

private:
	// program ID
	unsigned int mProgramID;
};