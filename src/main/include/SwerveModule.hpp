#pragma once

#include <frc/kinematics/SwerveModuleState.h>
// #include <frc/motorcontrol/PWMSparkMax.h>
// #include <units/angular_velocity.h>
// #include <units/time.h>
// #include <units/velocity.h>
// #include <units/voltage.h>
// #include <wpi/numbers>
#include <ctre\Phoenix.h>
using can_adr = int;

class SwerveModule
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    SwerveModule(can_adr drive_motor_adr, can_adr turning_motor_adr, can_adr cancoder_adr, frc::Translation2d wheel_position);
    frc::SwerveModuleState getState() const;
    void setDesiredState(const frc::SwerveModuleState &state);

    // Allows SwerveModule to be placed into Kinematics
    constexpr operator frc::Translation2d() const { return wheel_pos; }

    //No copies/moves should be occuring
    SwerveModule(Wheel const&) = delete;
    SwerveModule(Wheel&&)      = delete;
private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    TalonFX drive_motor, turning_motor;
    CANCoder direction;
    frc::Translation2d wheel_pos;
};
