#include <Trajectory.hpp>
#include <SwerveModule.hpp>
#include <Drivetrain.hpp>
#include "RobotState.hpp"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

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
    []()
    {
        frc::ProfiledPIDController<units::radian> theta_controller{
            8, 0, 2,
            frc::TrapezoidProfile<units::radian>::Constraints{
                Drivetrain::ROBOT_MAX_ANGULAR_SPEED,
                Drivetrain::ROBOT_MAX_ANGULAR_SPEED / 0.5_s}};
        theta_controller.EnableContinuousInput(units::radian_t{-wpi::numbers::pi}, units::radian_t{wpi::numbers::pi});
        return theta_controller;
    }()};

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
    static frc::Timer speed_timer;
    speed_timer.Start();
    static frc::Pose2d previous_pose{};

    frc::Pose2d const current_pose = odometry.GetPose();

    frc::Transform2d const delta_pose{previous_pose, current_pose};

    auto const time_elapsed = speed_timer.Get();
    units::meters_per_second_t const X = delta_pose.X() / time_elapsed;

    units::meters_per_second_t const Y = delta_pose.Y() / time_elapsed;

    units::degrees_per_second_t const rot = delta_pose.Rotation().Degrees() / time_elapsed;

    previous_pose = odometry.GetPose();

    speed_timer.Reset();

    return frc::ChassisSpeeds{X, Y, rot};

    /*
    Alternative,
    "auto FL_speed = FL.state.speed
    auto FL_dir = FL.state.direction
    auto FL_DX = FL_speed*cos(FL_dir)
    auto FL_DY = FL_speed*sin(FL_dir)"

    probably an easier way to write that but you get the idea

    rinse and repeat

    average out vertex components (dx and dy) (edited)

    and then you get the final robot vector?
    */
}

/******************************************************************/
/*                     Trajectory Functions                       */
/******************************************************************/

void Trajectory::driveToState(PathPlannerTrajectory::PathPlannerState const &state)
{
    //Correction to help the robot follow trajectory (combination of original trajectory speeds & error correction)
    frc::ChassisSpeeds const correction = controller.Calculate(odometry.GetPose(), state.pose, state.velocity, state.holonomicRotation);
    Drivetrain::drive(correction);

    // Put out the error numbers so we can tune?
    frc::Transform2d const holonomic_error = {odometry.GetPose(), state.pose};
    frc::ChassisSpeeds const current_speeds = Trajectory::getSpeeds();

    frc::SmartDashboard::PutNumber("Holonomic x error", holonomic_error.X().value());
    frc::SmartDashboard::PutNumber("Holonomic y error", holonomic_error.Y().value());
    frc::SmartDashboard::PutNumber("Holonomic z error", holonomic_error.Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber("Correction
     VX Speed", correction.vx.value());
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
    // How much "error" is tolerated
    controller.SetTolerance(frc::Pose2d{{0.05_m, 0.05_m}, {3_deg}});

    fmt::print("Interpreting PathPlanner trajectory\n");

    auto const inital_state = *traj.getInitialState();
    auto const inital_pose = inital_state.pose;

    // Just for debugging
    fmt::print("Got initial state: X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value());
    frc::SmartDashboard::PutString("Inital State: ", fmt::format("X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value()));

    // It is necessary to take the frc::Pose2d object from the state, extract its X & Y components, and then take the holonomicRotation
    // to construct a new Pose2d as the original Pose2d's Z (rotation) value uses non-holonomic math
    odometry.ResetPosition({inital_pose.X(), inital_pose.Y(), inital_state.holonomicRotation}, Drivetrain::getHeading());

    frc::Timer trajTimer;
    trajTimer.Start();

    int trajectory_samples = 0;

    // For debugging, we can disable the "error correction"
    controller.SetEnabled(false);

    frc::Field2d traj_field;
    frc::SmartDashboard::PutData("Trajectory Field", &traj_field);

    fmt::print("Successfully set odometry\n");

    while (RobotState::IsAutonomousEnabled() && trajTimer.Get() <= traj.getTotalTime())
    {
        auto current_time = trajTimer.Get();
        auto sample = traj.sample(current_time);

        // Just for debugging/tuning
        frc::SmartDashboard::PutString("Sample:",
                                       fmt::format("Current trajectory sample value: {}, Pose X: {}, Pose Y: {}, Pose Z: {}\nHolonomic Rotation: {}, Timer: {}\n",
                                                   ++trajectory_samples, sample.pose.X().value(), sample.pose.Y().value(), sample.pose.Rotation().Degrees().value(),
                                                   sample.holonomicRotation.Degrees().value(), current_time.value()));

        traj_field.SetRobotPose(odometry.GetPose());

        traj_field.GetObject("Traj")->SetPose({sample.pose.X(), sample.pose.Y(), sample.holonomicRotation});

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