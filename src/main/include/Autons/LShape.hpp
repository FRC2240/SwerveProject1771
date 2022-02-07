#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"

namespace Autons
{
    constexpr auto lShape = []()
    {
        Trajectory::follow("L with Rotate");
    };
}