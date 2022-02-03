#pragma once

#include "Drivetrain.hpp"

inline static auto const t_shape_auton = [](){
    using namespace pathplanner;
    
    Trajectory::follow(PathPlanner::loadPath("T shape", Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION),
                       [](units::second_t current_time) {});
};