#pragma once

#include <GLFW/glfw3.h>

class Timer
{
public:
	Timer();
	~Timer() = default;

	double Now() const { return glfwGetTime() - mStartTime; }
	void SetStart(double newStart) { mStartTime = newStart; }

	void PrintTimer(const char* logStatement) const;
	void PrintTimerReset(const char* logStatement);

private:

	double mStartTime;
};