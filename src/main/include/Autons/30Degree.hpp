#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    constexpr auto thirtyDegree = []()
    {
        Trajectory::follow(PathPlanner::loadPath("30 degree turn", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION, Trajectory::reverse_trajectory));
    };
}