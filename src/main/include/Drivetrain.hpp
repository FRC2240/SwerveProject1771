#pragma once

#include "ngr.hpp"

#include <wpi/numbers>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>

namespace Drivetrain
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();

    void resetGyro();

    // Returns values with 0 being front and positive angles going CCW
    units::degree_t getAngle();

    // Returns values with 0 being front and positive angles going CCW
    frc::Rotation2d getHeading();

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
    local constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 5_fps;                    // Implicit conversion
    local constexpr units::meters_per_second_t MODULE_MAX_SPEED = 7_fps;                    // Implicit conversion
    local constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{wpi::numbers::pi}; // 1/2 rotation per second
}