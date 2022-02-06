#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    constexpr auto straight = []()
    {
        Trajectory::follow(PathPlanner::loadPath("Straight Line", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION, Trajectory::reverse_trajectory));
    };
}