#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"
namespace Autons
{
    constexpr auto tShape = []()
    {
        Trajectory::follow(PathPlanner::loadPath("T shape", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
    };
}