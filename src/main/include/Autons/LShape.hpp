#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    constexpr auto l_shape_auton = []()
    {
        Trajectory::follow(PathPlanner::loadPath("L with Rotate", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
    };
}