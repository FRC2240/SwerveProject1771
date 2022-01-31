#pragma once

#include "ngr.hpp"

#include <wpi/numbers>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>
#include <units/acceleration.h>

namespace Drivetrain
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();

    // Returns values with 0 being front and positive angles going CW
    [[nodiscard]] units::degree_t getAngle();

    [[nodiscard]] frc::Rotation2d getCCWHeading();

    [[nodiscard]] frc::Rotation2d getCWHeading();

    void drive(units::meters_per_second_t const &xSpeed,
               units::meters_per_second_t const &ySpeed,
               units::radians_per_second_t const &rot,
               bool const &fieldRelative);

    void drive(frc::ChassisSpeeds const &speeds);

    void drive(wpi::array<frc::SwerveModuleState, 4> states);

    // For theta, positive is CCW
    void faceDirection(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, units::degree_t const &theta, bool const &field_relative);

    void faceClosest(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, bool const &field_relative);

    void setAngleForTuning(units::degree_t const &desired_angle);

    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/

/*
Thereotical max speed considering pi radians/second max angular speed
ROBOT SAT: 9.535f/s
*/

    //Formula for determing ROBOT_MAX_SPEED is Wheel Max Speed = Robot Max Speed + Omega max speed * distance of module from center
    //Or Robot Max Speed = Max wheel speed - Omega max speed * distance from center
    //Distance of module from center is 1.294ft

    //These are all very high and shouldn't normally be used
    local constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 5.47_fps;
    local constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{2 * wpi::numbers::pi};
    local constexpr units::meters_per_second_t MODULE_MAX_SPEED = 13.9_fps;

    local constexpr units::meters_per_second_t TELEOP_MAX_SPEED = 5_fps;
    local constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{wpi::numbers::pi};

    local constexpr units::meters_per_second_t TRAJ_MAX_SPEED = 1_fps;
    local constexpr units::acceleration::meters_per_second_squared_t TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 1_s;

}