#pragma once
#include "Trajectory.hpp"
#include "Drivetrain.hpp"
#include "Intake.hpp"
#include "Drivetrain.hpp"
#include "Intake.hpp"

namespace Autons
{
    constexpr auto complex_auton = []()
    { Trajectory::follow(PathPlanner::loadPath("Complex", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), [](units::second_t current_time)
                         {if (current_time > 1.5_s && current_time < 1.6_s) Intake::deploy(false); if (current_time > 2.5_s && current_time < 2.6_s) Intake::deploy(true); }); };
}