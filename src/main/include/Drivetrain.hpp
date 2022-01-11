#pragma once

#include "SwerveModule.hpp"
#include <wpi/numbers>
#include <units/angular_velocity.h>
#include <frc/geometry/Pose2d.h>

/**
 * Represents a swerve drive style drivetrain.
 */
namespace Drivetrain
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();

    void resetGyro();

    units::degree_t getAngle();

    frc::Rotation2d getHeading();

    frc::Pose2d getOdometryPose();

    void drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot, bool fieldRelative);

    void updateOdometry();

    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/
    static constexpr units::meters_per_second_t K_MAX_SPEED = 3.0_mps;                  // 3 meters per second
    static constexpr units::radians_per_second_t K_MAX_ANGULAR_SPEED{wpi::numbers::pi}; // 1/2 rotation per second

}