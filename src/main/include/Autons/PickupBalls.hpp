#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"
#include "Hopper.hpp"
#include "Intake.hpp"

namespace Autons
{
    constexpr auto pickupBalls = []()
    {
        Intake::deploy(true);
        Intake::drive(Intake::DIRECTION::IN);
        Trajectory::follow("Pickup Balls",
                           [](units::second_t current_time)
                           {
            Hopper::index(false); });
        Intake::deploy(false);
        Intake::drive(Intake::DIRECTION::OFF);
    };
}