#pragma once

#include "../Trajectory.hpp"
#include "../Drivetrain.hpp"

;
namespace Autons
{
    inline static constexpr auto t_shape_auton = []()
    {
        using namespace pathplanner;

        Trajectory::follow(PathPlanner::loadPath("T shape", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION),
                           [](units::second_t current_time) {});
    };
}