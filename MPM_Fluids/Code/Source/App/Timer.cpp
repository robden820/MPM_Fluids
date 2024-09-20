
#include "Timer.h"

#include <iostream>

Timer::Timer()
	: mStartTime(glfwGetTime())
{
}

void Timer::PrintTimer(const char* logStatement) const
{
	std::cout << logStatement << ": " << Now() << " seconds." << std::endl;
}

void Timer::PrintTimerReset(const char* logStatement)
{
	PrintTimer(logStatement);
	SetStart(glfwGetTime());
}