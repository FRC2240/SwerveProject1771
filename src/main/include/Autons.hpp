#pragma once

#include "ngr.hpp"
#include "Drivetrain.hpp"
#include "Trajectory.hpp"

#include <map>

using namespace pathplanner;
local std::map<std::string, std::function<void()>> autons{
    {"Default - LShape", []()
     {
         Trajectory::follow(PathPlanner::loadPath("L with Rotate", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
     }},
    {"30 Degree", []()
     {
         Trajectory::follow(PathPlanner::loadPath("30 degree turn", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
     }},

    {"Straight", []()
     {
         Trajectory::follow(PathPlanner::loadPath("Straight Line", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
     }},

    {"TShape", []()
     {
         Trajectory::follow(PathPlanner::loadPath("T shape", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
     }}};