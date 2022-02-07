#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    constexpr auto straight = []()
    {
        Trajectory::follow("Straight Line");
    };
}