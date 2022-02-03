#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

inline static auto const l_shape_auton = []()
{
    using namespace pathplanner;
    Trajectory::follow(PathPlanner::loadPath("L with Rotate", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION),
                       [](units::second_t current_time) {});
};