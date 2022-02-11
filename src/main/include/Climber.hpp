#pragma once

#include <wpi/array.h>
namespace Climber
{
    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/
    enum POSITION
    {
        DOWN = 50,
        UP = 802,
        ZERO = 0
    };
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();
    void set(POSITION position);
    void buttonManager();
    void joystickControl(double);
    void printStatus();

    [[nodiscard]] wpi::array<double, 2> getTemps();
} // namespace Climber