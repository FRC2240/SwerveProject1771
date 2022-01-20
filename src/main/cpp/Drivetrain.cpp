#include "Drivetrain.hpp"
#include "RobotState.hpp"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <AHRS.h>

#include <thread>
#include <chrono>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

// faceDirection && faceClosest constants
double constexpr ROTATE_P = 1.75; // Modifier for rotational speed -> (degree * ROTATE_P)

units::degrees_per_second_t constexpr MAX_FACE_DIRECTION_SPEED = 150_deg / 1_s; // only used for faceDirection

units::degree_t constexpr FRONT{0}, BACK{180};

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib

namespace Module
{
  local SwerveModule front_left{40, 41, 12, {11_in, 11_in}, -344.53125};
  local SwerveModule front_right{30, 31, 11, {11_in, -11_in}, -264.726563};
  local SwerveModule back_left{50, 51, 13, {-11_in, 11_in}, -91.54203};
  local SwerveModule back_right{60, 61, 14, {-11_in, -11_in}, -285.0293};
}

local_c frc::SwerveDriveKinematics<4> kinematics{Module::front_left,
                                                 Module::front_right,
                                                 Module::back_left,
                                                 Module::back_right};

local std::unique_ptr<AHRS> navx = nullptr;

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

void Drivetrain::init()
{
  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

  resetGyro();

  Module::front_left.init();
  Module::front_right.init();
  Module::back_left.init();
  Module::back_right.init();

  controller.SetTolerance(frc::Pose2d{{0.2_m, 0.2_m}, {3_deg}});
}

void Drivetrain::resetGyro() { navx->ZeroYaw(); }

// Returns values in CW direction with 0 being equal to front (navx is normally CCW)
units::degree_t Drivetrain::getAngle() { return units::degree_t{navx->GetAngle()}; }

frc::Pose2d Drivetrain::getOdometryPose() { return odometry.GetPose(); }

void Drivetrain::printOdometryPose()
{
  auto const pose = odometry.GetPose();
  frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::Rotation2d Drivetrain::getHeading() { return {getAngle()}; }

frc::SwerveDriveKinematics<4> const &Drivetrain::getKinematics() { return kinematics; }

void Drivetrain::updateOdometry()
{
  odometry.Update(getHeading(),
                  Module::front_left.getState(),
                  Module::front_right.getState(),
                  Module::back_left.getState(),
                  Module::back_right.getState());
}

/******************************************************************/
/*                       Driving Functions                        */
/******************************************************************/

// Converts inputted speeds into a frc::ChassisSpeeds object
void Drivetrain::drive(units::meters_per_second_t const &xSpeed,
                       units::meters_per_second_t const &ySpeed,
                       units::radians_per_second_t const &rot,
                       bool const &fieldRelative)
{
  auto const speeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  drive(speeds);
}

// Takes the speed & direction the robot should be going and figures out the states for each indivdual module
void Drivetrain::drive(frc::ChassisSpeeds const &speeds)
{
  drive(kinematics.ToSwerveModuleStates(speeds));
}

// Sets each module to the desired state
void Drivetrain::drive(wpi::array<frc::SwerveModuleState, 4> states)
{

  kinematics.DesaturateWheelSpeeds(&states, ROBOT_MAX_SPEED);

  auto const [fl, fr, bl, br] = states;

  Module::front_left.setDesiredState(fl);
  Module::front_right.setDesiredState(fr);
  Module::back_left.setDesiredState(bl);
  Module::back_right.setDesiredState(br);
}

/******************************************************************/
/*                        Facing Functions                        */
/******************************************************************/

void Drivetrain::setAngleForTuning(units::degree_t const &desired_angle)
{
  Module::front_left.setTurnerAngle(desired_angle);
}
void Drivetrain::faceDirection(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, units::degree_t const &theta, bool const &field_relative)
{
  frc::SmartDashboard::PutNumber("Current angle", getAngle().value());
  int error_theta = (theta - getAngle()).to<int>() % 360; // Get difference between old and new angle; gets the equivalent value between -360 and 360
  frc::SmartDashboard::PutNumber("Original error_theta", error_theta);
  if (error_theta < -180)
    error_theta += 360; // Ensure angle is between 0 and 360
  if (error_theta > 180)
    error_theta -= 360; // Optimizes angle if over 180
  frc::SmartDashboard::PutNumber("Optimized error_theta", error_theta);
  if (abs(error_theta) < 10)
    error_theta = 0;
  frc::SmartDashboard::PutNumber("Dead-zoned error_theta", error_theta);
  double p_rotation = error_theta * ROTATE_P; // Modifies error_theta in order to get a faster turning speed
  if (abs(p_rotation) > MAX_FACE_DIRECTION_SPEED.value())
    p_rotation = MAX_FACE_DIRECTION_SPEED.value() * ((p_rotation > 0) ? 1 : -1); // Constrains turn speed
  frc::SmartDashboard::PutNumber("Constrained p_rotation", p_rotation);
  drive(dx, dy, units::degrees_per_second_t{p_rotation}, field_relative);
}

void Drivetrain::faceClosest(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, bool const &field_relative)
{
  int current_rotation = getAngle().to<int>() % 360; // Ensure angle is between -360 and 360
  if (current_rotation < 0)
    current_rotation += 360; // Ensure angle is between 0 and 360
  if (current_rotation <= 90 || current_rotation >= 270)
    faceDirection(dx, dy, 0_deg, field_relative);
  else
    faceDirection(dx, dy, 180_deg, field_relative);
}

/******************************************************************/
/*                     Trajectory Functions                       */
/******************************************************************/

void Drivetrain::trajectoryDrive(PathPlannerTrajectory::PathPlannerState const &state)
{
  auto correction = controller.Calculate(odometry.GetPose(), state.pose, state.velocity, state.holonomicRotation);
  drive(correction);
  // Put out the error numbers so we can tune?
  auto holonomic_error = odometry.GetPose() - state.pose;
  frc::SmartDashboard::PutNumber("Holonomic x error", holonomic_error.X().value());
  frc::SmartDashboard::PutNumber("Holonomic y error", holonomic_error.Y().value());
  frc::SmartDashboard::PutNumber("Holonomic z error", (odometry.GetPose().Rotation().Degrees() - state.holonomicRotation.Degrees()).value());

  frc::SmartDashboard::PutNumber("Chassis Speeds: VX", correction.vx.value());
  frc::SmartDashboard::PutNumber("Chassis Speeds: VY", correction.vy.value());
  frc::SmartDashboard::PutNumber("Chassis Speeds: Omega", units::degrees_per_second_t{correction.omega}.value());
}

static std::thread trajectory_thread;

bool trajectory_stop_flag = false;

void Drivetrain::trajectoryAutonDrive(pathplanner::PathPlannerTrajectory const &traj)
{
  using namespace std::chrono_literals;
  fmt::print("Interpreting PathPlanner trajectory\n");
  trajectory_stop_flag = true; // stop previous thread (this is only here as a safety feature in case method gets called twice)
  if (trajectory_thread.joinable())
    trajectory_thread.join();
  trajectory_stop_flag = false;
  trajectory_thread = std::thread{[&traj]()
                                  {
                                    PathPlannerTrajectory copied_traj(traj);
                                    fmt::print("Beginning trajectory sampling\n");
                                    auto const inital_state = *copied_traj.getInitialState();
                                    auto const inital_pose = inital_state.pose;
                                    fmt::print("Got initial state: X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value());
                                    frc::SmartDashboard::PutString("Inital State: ", fmt::format("X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value()));
                                    odometry.ResetPosition({inital_pose.X(), inital_pose.Y(), inital_state.holonomicRotation}, getHeading());
                                    frc::Timer trajTimer;
                                    trajTimer.Start();
                                    int trajectory_samples = 0;
                                    fmt::print("Successfully set odometry\n");
                                    while (!trajectory_stop_flag && RobotState::IsAutonomousEnabled() && trajTimer.Get() <= copied_traj.getTotalTime())
                                    {
                                      auto current_time = trajTimer.Get();
                                      auto sample = copied_traj.sample(current_time);
                                      frc::SmartDashboard::PutString("Sample:",
                                                                     fmt::format("Current trajectory sample value: {}, Pose X: {}, Pose Y: {}, Pose Z: {}\nHolonomic Rotation: {}, Timer: {}\n",
                                                                                 ++trajectory_samples, sample.pose.X().value(), sample.pose.Y().value(), sample.pose.Rotation().Degrees().value(),
                                                                                 sample.holonomicRotation.Degrees().value(), current_time.value()));
                                      trajectoryDrive(sample);
                                      std::this_thread::sleep_for(20ms); // This is the refresh rate of the PID controllers
                                    }
                                    drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);
                                  }};
  trajectory_thread.join();
}

std::thread test_holonomic_thread;
bool stop_testing = true;
void Drivetrain::testHolonomicRotation(units::degree_t const &desired_angle)
{
  // if (stop_testing)
  // {
  //   stop_testing = false;
  //   fmt::print("Driving to an angle of %d\n", desired_angle.value());
  //   test_holonomic_thread = std::thread{[&desired_angle]()
  //                                       {
  //                                         auto new_angle(desired_angle);
  //                                         auto i = 0;
  //                                         using namespace std::chrono_literals;
  //                                         while (!stop_testing)
  //                                         {
  //                                           drive(controller.Calculate(odometry.GetPose(), odometry.GetPose(), 0_mps, {new_angle}));
  //                                           // if (i++ % 10 == 0)
  //                                           //   new_angle += 3_deg;
  //                                           std::this_thread::sleep_for(20ms);
  //                                         }
  //                                       }};
  // }
  auto pose = odometry.GetPose();
  drive(controller.Calculate(pose, frc::Pose2d{{pose.X() + .1_m, pose.Y() + .1_m}, pose.Rotation()}, 1_mps, {desired_angle}));
}

void Drivetrain::stopHolonomicTesting()
{
  stop_testing = true;
  if (test_holonomic_thread.joinable())
    test_holonomic_thread.join();
}