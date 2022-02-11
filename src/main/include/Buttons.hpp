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
    inline frc::Joystick PS5{0}, oStick{1};

    namespace INTAKE
    {
        inline JoystickButton DEPLOY{BUTTON::oStick, 3};
        inline JoystickButton RETRACT{BUTTON::oStick, 4};
        inline JoystickButton INTAKE{BUTTON::oStick, 5};
    } // namespace INTAKE
    namespace SHOOTER
    {
        inline JoystickButton AIM_FRONT{BUTTON::oStick, 8};
        inline JoystickButton AIM_BACK{BUTTON::oStick, 10};
        inline JoystickButton AIM_SIDE{BUTTON::oStick, 2};
        inline JoystickButton BATTERSHOT{BUTTON::oStick, 6};
        inline JoystickButton SHOOT{BUTTON::oStick, 1};
        inline JoystickButton ADJUST_SHOOTER_UP{BUTTON::oStick, 12};
        inline JoystickButton ADJUST_SHOOTER_DOWN{BUTTON::oStick, 11};

    } // namespace SHOOTER

    namespace CLIMBER
    {
        inline JoystickButton RAISE{BUTTON::oStick, 11};
    }
    namespace DRIVETRAIN
    {
        inline JoystickButton ROTATE_FRONT{BUTTON::PS5, 4};
        inline JoystickButton ROTATE_BACK{BUTTON::PS5, 2};
        inline JoystickButton ROTATE_TO_CLOSEST{BUTTON::PS5, 1};
        inline JoystickButton TURN_45{BUTTON::PS5, 4};
        inline JoystickButton TURN_neg45{BUTTON::PS5, 2};
        inline JoystickButton TURN_90{BUTTON::PS5, 3};
        inline JoystickButton TURN_neg90{BUTTON::PS5, 1};
        inline JoystickButton ROTATION_MODE{BUTTON::PS5, 9};
        inline JoystickButton FIELD_CENTRIC{BUTTON::PS5, 10};
    }
}