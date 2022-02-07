#pragma once
#include "Trajectory.hpp"
#include "Drivetrain.hpp"
#include "Intake.hpp"
#include "Drivetrain.hpp"
#include "Intake.hpp"
#include "Hopper.hpp"

namespace Autons
{
    constexpr auto complex = []()
    {
        Intake::deploy(true);
        Intake::drive(Intake::DIRECTION::IN);
        Trajectory::follow("Complex", [](units::second_t current_time)
                           { Hopper::index(false); });
        Intake::deploy(false);
    };
}