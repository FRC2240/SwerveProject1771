#include <pathplanner/lib/PathPlanner.h>

#include <frc/kinematics/ChassisSpeeds.h>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it is safe to remove the first layer

namespace Trajectory
{
    void init();

    [[nodiscard]] frc::Pose2d getOdometryPose();

    void updateOdometry();

    [[nodiscard]] frc::ChassisSpeeds const getEstimatedSpeeds();

    [[nodiscard]] frc::ChassisSpeeds const getRealSpeeds();

    void printEstimatedSpeeds();

    void printRealSpeeds();

    void driveToState(PathPlannerTrajectory::PathPlannerState const &state);

    void follow(PathPlannerTrajectory traj);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);

}