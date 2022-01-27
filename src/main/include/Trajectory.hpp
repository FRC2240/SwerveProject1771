#include <pathplanner/lib/PathPlanner.h>

#include <frc/kinematics/ChassisSpeeds.h>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it is safe to remove the first layer

namespace Trajectory
{
    void init();

    frc::Pose2d getOdometryPose();

    void updateOdometry();

    frc::ChassisSpeeds const getEstimatedSpeeds();

    frc::ChassisSpeeds const getRealSpeeds();

    void printEsimatedSpeeds();

    void printRealSpeeds();

    void driveToState(PathPlannerTrajectory::PathPlannerState const &state);

    void follow(PathPlannerTrajectory traj);

    void testHolonomic(units::degree_t const &desired_angle);
}