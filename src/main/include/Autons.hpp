#pragma once

#include "Autons/30Degree.hpp"
#include "Autons/Complex.hpp"
#include "Autons/LShape.hpp"
#include "Autons/PickupBalls.hpp"
#include "Autons/TShape.hpp"

#include <map>
#include <functional>

// All autonomous programs are stored as lambdas to be called
using namespace Autons;
inline std::map<std::string, std::function<void()>> autons{
    {"Default - LShape", l_shape_auton},
    {"30 Degree", thirty_degree_auton},
    {"TShape", t_shape_auton},
    {"Pickup Balls", pickup_balls_auton},
    {"Complex", complex_auton}};