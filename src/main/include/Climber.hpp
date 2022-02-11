#pragma once

namespace Climber
{
    enum POSITION {
        DOWN = 50,
        UP   = 802,
        ZERO = 0
    };
    //Function Declarations
    void init();
    void set(POSITION position);
    void buttonManager();
    void joystickControl(double);
    void printStatus();
} // namespace Climber