#pragma once

#include "SwerveModule.hpp"

#include <wpi/numbers>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>

#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner; //PathPlanner keeps everything hidden behind 2 sets of namespaces so it is safe to remove the first layer

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

    void printOdometryPose();

    frc::SwerveDriveKinematics<4> const &getKinematics();

    void updateOdometry();

    void drive(units::meters_per_second_t const &xSpeed,
               units::meters_per_second_t const &ySpeed,
               units::radians_per_second_t const &rot,
               bool const &fieldRelative);

    void drive(frc::ChassisSpeeds const &speeds);

    void drive(wpi::array<frc::SwerveModuleState, 4> states);

    void faceDirection(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, units::degree_t const &theta);

    void faceClosest(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy);

    void trajectoryDrive(frc::Trajectory::State const &state, frc::Rotation2d const &rotation);

    void trajectoryDrive(PathPlannerTrajectory::PathPlannerState const &state);

    void trajectoryAutonDrive(frc::Trajectory const &traj, frc::Rotation2d const &faceAngle);

    void trajectoryAutonDrive(PathPlannerTrajectory traj);

    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/
    static constexpr units::meters_per_second_t K_MAX_SPEED = 2_fps;                   // Implicit conversion
    static constexpr units::radians_per_second_t K_MAX_ANGULAR_SPEED{wpi::numbers::pi}; // 1/2 rotation per second
}