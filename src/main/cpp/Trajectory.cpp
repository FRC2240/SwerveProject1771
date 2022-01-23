#include <Trajectory.hpp>
#include <SwerveModule.hpp>
#include <Drivetrain.hpp>
#include "RobotState.hpp"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <AHRS.h>

#include <thread>
#include <chrono>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/
namespace Module
{
    extern SwerveModule front_left;
    extern SwerveModule front_right;
    extern SwerveModule back_left;
    extern SwerveModule back_right;
}

extern frc::SwerveDriveKinematics<4> const kinematics;

extern std::unique_ptr<AHRS> navx;

local frc::SwerveDriveOdometry<4> odometry{kinematics, frc::Rotation2d{0_deg}};

local frc::HolonomicDriveController controller{
    frc2::PIDController{.405, 0, 2},
    frc2::PIDController{.405, 0, 2},
    frc::ProfiledPIDController<units::radian>{
        8, 0, 2,
        frc::TrapezoidProfile<units::radian>::Constraints{
            Drivetrain::ROBOT_MAX_ANGULAR_SPEED,
            Drivetrain::ROBOT_MAX_ANGULAR_SPEED / 0.5_s}}};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
void Trajectory::updateOdometry()
{
    odometry.Update(Drivetrain::getHeading(),
                    Module::front_left.getState(),
                    Module::front_right.getState(),
                    Module::back_left.getState(),
                    Module::back_right.getState());
}

frc::Pose2d Trajectory::getOdometryPose() { return odometry.GetPose(); }

void Trajectory::printOdometryPose()
{
    auto const pose = odometry.GetPose();
    frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::ChassisSpeeds const Trajectory::getSpeeds()
{
    // Init for first time
    local frc::Timer speedTimer;
    speedTimer.Start();
    local frc::Pose2d previousPose{};

    frc::Pose2d const currentPose = odometry.GetPose();

    frc::Transform2d const deltaPose{previousPose, currentPose};

    auto const timeElapsed = speedTimer.Get();
    units::meters_per_second_t const X = deltaPose.X() / timeElapsed;

    units::meters_per_second_t const Y = deltaPose.Y() / timeElapsed;

    units::degrees_per_second_t const rot = deltaPose.Rotation().Degrees() / timeElapsed;

    previousPose = odometry.GetPose();

    speedTimer.Reset();

    return frc::ChassisSpeeds{X, Y, rot};
}

/******************************************************************/
/*                     Trajectory Functions                       */
/******************************************************************/

void Trajectory::driveToState(PathPlannerTrajectory::PathPlannerState const &state)
{
    // It is necessary to take the frc::Pose2d object from the state, extract its X & Y components, and then take the holonomicRotation
    //  to construct a new Pose2d as the original Pose2d's Z (rotation) value uses non-holonomic math
    frc::Pose2d const targetPose = {state.pose.X(), state.pose.Y(), state.holonomicRotation};

    frc::ChassisSpeeds const correction = controller.Calculate(odometry.GetPose(), targetPose, state.velocity, state.holonomicRotation);
    Drivetrain::drive(correction);

    // Put out the error numbers so we can tune?
    frc::Transform2d const holonomic_error = {odometry.GetPose(), targetPose};
    frc::ChassisSpeeds const current_speeds = Trajectory::getSpeeds();

    frc::SmartDashboard::PutNumber("Holonomic x error", holonomic_error.X().value());
    frc::SmartDashboard::PutNumber("Holonomic y error", holonomic_error.Y().value());
    frc::SmartDashboard::PutNumber("Holonomic z error", holonomic_error.Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber("Correction VX Speed", correction.vx.value());
    frc::SmartDashboard::PutNumber("Correction VY Speed", correction.vy.value());
    frc::SmartDashboard::PutNumber("Correction Omega Speed", units::degrees_per_second_t{correction.omega}.value());

    frc::SmartDashboard::PutNumber("Current VX Speed", current_speeds.vx.value());
    frc::SmartDashboard::PutNumber("Current VY Speed", current_speeds.vy.value());
    frc::SmartDashboard::PutNumber("Current Omega Speed", current_speeds.omega.value());

    frc::SmartDashboard::PutNumber("Speed VX Change", (correction.vx - current_speeds.vx).value());
    frc::SmartDashboard::PutNumber("Speed VY Change", (correction.vy - current_speeds.vy).value());
    frc::SmartDashboard::PutNumber("Speed Omega Change", (correction.omega - current_speeds.omega).value());
}

using namespace std::chrono_literals;

void Trajectory::follow(pathplanner::PathPlannerTrajectory traj)
{
    //How much "error" is tolerated
    controller.SetTolerance(frc::Pose2d{{0.05_m, 0.05_m}, {3_deg}});

    fmt::print("Interpreting PathPlanner trajectory\n");

    auto const inital_state = *traj.getInitialState();
    auto const inital_pose = inital_state.pose;

    //Just for debugging
    fmt::print("Got initial state: X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value());
    frc::SmartDashboard::PutString("Inital State: ", fmt::format("X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value()));

    // It is necessary to take the frc::Pose2d object from the state, extract its X & Y components, and then take the holonomicRotation
    // to construct a new Pose2d as the original Pose2d's Z (rotation) value uses non-holonomic math
    odometry.ResetPosition({inital_pose.X(), inital_pose.Y(), inital_state.holonomicRotation}, Drivetrain::getHeading());

    frc::Timer trajTimer;
    trajTimer.Start();

    int trajectory_samples = 0;

    fmt::print("Successfully set odometry\n");

    while (RobotState::IsAutonomousEnabled() && trajTimer.Get() <= traj.getTotalTime())
    {
        auto current_time = trajTimer.Get();
        auto sample = traj.sample(current_time);

        //Just for debugging/tuning
        frc::SmartDashboard::PutString("Sample:",
                                       fmt::format("Current trajectory sample value: {}, Pose X: {}, Pose Y: {}, Pose Z: {}\nHolonomic Rotation: {}, Timer: {}\n",
                                                   ++trajectory_samples, sample.pose.X().value(), sample.pose.Y().value(), sample.pose.Rotation().Degrees().value(),
                                                   sample.holonomicRotation.Degrees().value(), current_time.value()));

        driveToState(sample);
        std::this_thread::sleep_for(20ms); // This is the refresh rate of the HolonomicDriveController's PID controllers (can be tweaked if needed)
    }
    Drivetrain::drive(0_mps, 0_mps, units::radians_per_second_t{0}, false);
}

void Trajectory::testHolonomic(units::degree_t const &desired_angle)
{
    auto pose = odometry.GetPose();
    Drivetrain::drive(controller.Calculate(pose, frc::Pose2d{{pose.X() + .1_m, pose.Y() + .1_m}, pose.Rotation()}, 1_mps, {desired_angle}));
}