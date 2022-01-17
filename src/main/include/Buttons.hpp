#pragma once

#include <frc/GenericHID.h>
#include <frc/Joystick.h>

/******************************************************************/
/*                  Public Function Declarations                  */
/******************************************************************/
class JoystickButton
{
    frc::GenericHID &stick_;
    int const button_;

public:
    JoystickButton(frc::GenericHID &, int const &button);
    [[nodiscard]] operator bool() const { return stick_.GetRawButton(button_); }
    [[nodiscard]] bool getRawButton() const;
    [[nodiscard]] bool getRawButtonPressed();
    [[nodiscard]] bool getRawButtonReleased();
};

/******************************************************************/
/*                        Public Constants                        */
/******************************************************************/

namespace BUTTON
{
    inline frc::Joystick ps5{0};
    namespace DRIVETRAIN
    {
        inline JoystickButton ROTATE_FRONT{BUTTON::ps5, 7};
        inline JoystickButton ROTATE_BACK{BUTTON::ps5, 8};
        inline JoystickButton ROTATE_TO_CLOSEST{BUTTON::ps5, 9};
        inline JoystickButton TURN_45{BUTTON::ps5, 9};
        inline JoystickButton TURN_neg45{BUTTON::ps5, 9};
        inline JoystickButton TURN_90{BUTTON::ps5, 9};
        inline JoystickButton TURN_neg90{BUTTON::ps5, 9};
        inline JoystickButton ROTATION_MODE{BUTTON::ps5, 9};
    }
}