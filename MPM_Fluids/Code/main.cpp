// Entry point for MPM fluids simulation

#include <iostream>

#include "OpenGlApp.h"

int main()
{
    std::cout << "Running MPM simulation...\n";

    OpenGLApp app(800, 600);

    app.Run();

    return 0;
}