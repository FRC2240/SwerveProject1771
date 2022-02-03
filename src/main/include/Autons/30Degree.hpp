#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

using namespace pathplanner;

;
namespace Autons
{
    inline static auto const thirty_degree_auton = []()
    {
        using namespace pathplanner;
        Trajectory::follow(PathPlanner::loadPath("30 degree turn", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION),
                           [](units::second_t current_time) {});
    };
}