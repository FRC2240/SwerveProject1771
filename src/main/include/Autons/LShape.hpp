#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    inline static auto const l_shape_auton = []()
    {
                Trajectory::follow(PathPlanner::loadPath("L with Rotate", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION),
                           [](units::second_t current_time) {});
    };
}