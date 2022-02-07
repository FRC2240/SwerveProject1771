#pragma once

#include "Trajectory.hpp"
#include "Drivetrain.hpp"
namespace Autons
{
    constexpr auto tShape = []()
    {
        Trajectory::follow("T shape");
    };
}