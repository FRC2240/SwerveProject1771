#pragma once

#include "Drivetrain.hpp"

#include <pathplanner/lib/PathPlanner.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <functional>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

namespace Trajectory
{
    /******************************************************************/
    /*                   Public Variable Definitions                  */
    /******************************************************************/
    void putField2d();

    [[nodiscard]] frc::Pose2d getOdometryPose();

    void updateOdometry();

    [[nodiscard]] frc::ChassisSpeeds const getEstimatedSpeeds();

    [[nodiscard]] frc::ChassisSpeeds const getRealSpeeds();

    void printEstimatedSpeeds();

    void printRealSpeeds();

    void driveToState(PathPlannerTrajectory::PathPlannerState const &state);

    void follow(std::string const &traj_dir,
                std::function<void(units::second_t time)> const &periodic = nullptr,
                units::meters_per_second_t const &max_vel = Drivetrain::TRAJ_MAX_SPEED,
                units::meters_per_second_squared_t const &max_accl = Drivetrain::TRAJ_MAX_ACCELERATION);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);

    inline bool reverse_trajectory = false;
}