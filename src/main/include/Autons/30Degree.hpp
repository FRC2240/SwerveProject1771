#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    constexpr auto thirtyDegree = []()
    {
        Trajectory::follow("30 degree turn");
    };
}