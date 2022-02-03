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
    void drive(DIRECTION mode);
    void deploy(bool val);

    [[nodiscard]] bool isIntakeDown();
}