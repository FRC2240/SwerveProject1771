#pragma once

namespace Intake
{
    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/
    enum class DIRECTION
    {
        OUT,
        OFF,
        IN
    };
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();
    void drive(DIRECTION const &mode);
    void deploy(bool const &val);

    [[nodiscard]] bool isIntakeDown();

    [[nodiscard]] double getWheelTemp();
}