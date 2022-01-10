#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>

#include "SwerveModule.hpp"

/**
 * Represents a swerve drive style drivetrain.
 */
namespace Drivetrain
{
    Drivetrain() { m_gyro.Reset(); }

    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative);
    void UpdateOdometry();

    static constexpr units::meters_per_second_t kMaxSpeed = 3.0_mps;                 // 3 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{wpi::numbers::pi}; // 1/2 rotation per second

};
