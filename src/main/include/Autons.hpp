#pragma once

#include "ngr.hpp"
#include "Drivetrain.hpp"
#include "Trajectory.hpp"
#include "Intake.hpp"
#include "Hopper.hpp"
#include "Hood.hpp"
#include "ShooterWheel.hpp"

#include <map>

using namespace pathplanner;
local std::map<std::string, std::function<void()>> autons{
    {"Default - LShape", []()
     {
         Trajectory::follow(PathPlanner::loadPath("L with Rotate", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), {});
     }},
    {"30 Degree", []()
     {
         Trajectory::follow(PathPlanner::loadPath("30 degree turn", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), {});
     }},

    {"Straight", []()
     {
         Trajectory::follow(PathPlanner::loadPath("Straight Line", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), {});
     }},

    {"TShape", []()
     {
         Trajectory::follow(PathPlanner::loadPath("T shape", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), {});
     }},
    {"Pickup Balls", []()
     {
         Intake::deploy(true);
         Intake::drive(Intake::DIRECTION::IN);
         Trajectory::follow(PathPlanner::loadPath("Pickup Balls", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), [](units::second_t current_time)
                            {
            ShooterWheel::bangbang();
            Hopper::index(false); });
         Intake::deploy(false);
         Intake::drive(Intake::DIRECTION::OFF);
     }},
    {"Complex", []()
     {
         Trajectory::follow(PathPlanner::loadPath("Complex", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION), [](units::second_t current_time)
                            {
            if (current_time > 1.5_s && current_time < 1.6_s)
                Intake::deploy(false);
            if (current_time > 2.5_s && current_time < 2.6_s)
                Intake::deploy(true); });
     }}};