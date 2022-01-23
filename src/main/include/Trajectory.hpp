#include <pathplanner/lib/PathPlanner.h>

#include <frc/kinematics/ChassisSpeeds.h>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it is safe to remove the first layer

namespace Trajectory
{
    frc::Pose2d getOdometryPose();

    void updateOdometry();

    void printOdometryPose();

    frc::ChassisSpeeds const getSpeeds();

    void driveToState(PathPlannerTrajectory::PathPlannerState const &state);

    void follow(PathPlannerTrajectory traj);

    void testHolonomic(units::degree_t const &desired_angle);
}