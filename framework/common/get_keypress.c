/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

// Function to check keyboard without blocking
#include <unistd.h>  // For STDIN_FILENO and read()
#include <fcntl.h>   // For fcntl(), F_GETFL, F_SETFL, and O_NONBLOCK
#include <termios.h> // For struct termios, tcgetattr(), etc.

char get_keypress() {
    struct termios oldt, newt;
    char ch = 0;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    read(STDIN_FILENO, &ch, 1);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Reset terminal
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    return ch;
}