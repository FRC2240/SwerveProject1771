#include "Buttons.hpp"
#include <iostream>

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

JoystickButton::JoystickButton(frc::GenericHID& stick, int const button)
    : stick_ { stick }
    , button_ { button }
{
    if(button > stick.GetButtonCount() || button < 1)
        std::cerr << "Invalid Button Assignment: " << button << '\n';
}

bool JoystickButton::getRawButton() const
{
    return stick_.GetRawButton(button_);
}

bool JoystickButton::getRawButtonPressed()
{
    return stick_.GetRawButtonPressed(button_);
}

bool JoystickButton::getRawButtonReleased()
{
    return stick_.GetRawButtonReleased(button_);
}
