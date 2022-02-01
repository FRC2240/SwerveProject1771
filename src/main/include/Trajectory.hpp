#pragma once

#include <pathplanner/lib/PathPlanner.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <functional>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it is safe to remove the first layer

namespace Trajectory
{
    void putField2d();

    [[nodiscard]] frc::Pose2d getOdometryPose();

    void updateOdometry();

    [[nodiscard]] frc::ChassisSpeeds const getEstimatedSpeeds();

    [[nodiscard]] frc::ChassisSpeeds const getRealSpeeds();

    void printEstimatedSpeeds();

    void printRealSpeeds();

    void driveToState(PathPlannerTrajectory::PathPlannerState const &state);

    void follow(pathplanner::PathPlannerTrajectory traj, std::function<void(units::second_t time)> extra_control);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);
}