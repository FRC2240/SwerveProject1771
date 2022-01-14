#include "Drivetrain.hpp"
#include "RobotState.hpp"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/controller/HolonomicDriveController.h>

#include <AHRS.h>

#include <thread>
#include <chrono>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

double constexpr ROTATE_P = 1.25; // Modifier for rotational speed -> (degree * ROTATE_P) per second

units::degrees_per_second_t constexpr MAX_FACE_DIRECTION_SPEED = 35_deg / 1_s; // only used for faceDirection

units::degree_t constexpr FRONT{0}, BACK{180};

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib
namespace Module
{
  inline static SwerveModule front_left{40, 41, 12, {11_in, 11_in}};
  inline static SwerveModule front_right{30, 31, 11, {11_in, -11_in}};
  inline static SwerveModule back_left{50, 51, 13, {-11_in, 11_in}};
  inline static SwerveModule back_right{60, 61, 14, {-11_in, -11_in}};
}

inline static const frc::SwerveDriveKinematics<4> kinematics{Module::front_left,
                                                               Module::front_right,
                                                               Module::back_left,
                                                               Module::back_right};

inline static std::unique_ptr<AHRS> navx = nullptr;

inline static frc::SwerveDriveOdometry<4> odometry{kinematics, frc::Rotation2d{0_deg}};

static frc::HolonomicDriveController controller{
    frc2::PIDController{1, 0, 0},
    frc2::PIDController{1, 0, 0},
    frc::ProfiledPIDController<units::radian>{
        1, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            Drivetrain::K_MAX_ANGULAR_SPEED,
            Drivetrain::K_MAX_ANGULAR_SPEED / 1_s}}};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

  resetGyro();
}

void Drivetrain::resetGyro() { navx->ZeroYaw(); }

units::degree_t Drivetrain::getAngle() { return units::degree_t{-navx->GetAngle() - 90}; }

frc::Pose2d Drivetrain::getOdometryPose() { return odometry.GetPose(); }

void Drivetrain::printOdometryPose()
{
  auto const pose = odometry.GetPose();
  printf("Pose X: %f, Y: %f, Z (Degrees): %f", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
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

  kinematics.DesaturateWheelSpeeds(&states, K_MAX_SPEED);

  auto const [fl, fr, bl, br] = states;

  Module::front_left.setDesiredState(fl);
  Module::front_right.setDesiredState(fr);
  Module::back_left.setDesiredState(bl);
  Module::back_right.setDesiredState(br);
}

/******************************************************************/
/*                        Facing Functions                        */
/******************************************************************/

void Drivetrain::faceDirection(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, units::degree_t const &theta)
{

  int const error_theta = (theta - getAngle()).to<int>() % 360;          // Get difference between old and new angle; gets the equivalent value between -360 and 360
  auto p_rotation = units::degrees_per_second_t{error_theta * ROTATE_P}; // Modifies error_theta in order to get a faster turning speed
  if (p_rotation > MAX_FACE_DIRECTION_SPEED)                             // Max rotational speed
    p_rotation = MAX_FACE_DIRECTION_SPEED * ((p_rotation.value() > 0) ? 1 : -1);
  drive(frc::ChassisSpeeds{dx, dy, p_rotation});
}

void Drivetrain::faceClosest(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy)
{
  int current_rotation = getAngle().to<int>() % 360; // Ensure angle is between -360 and 360
  if (current_rotation < 0)
  {
    current_rotation += 360; // Ensure angle is between 0 and 360
  }
  units::degree_t const new_rotation = (current_rotation <= 90 || current_rotation >= 270) ? FRONT : BACK;
  faceDirection(dx, dy, new_rotation);
}

/******************************************************************/
/*                     Trajectory Functions                       */
/******************************************************************/
void Drivetrain::trajectoryDrive(frc::Trajectory::State const &state, frc::Rotation2d const &rotation)
{
  printf("Driving based on inputted trajectory state\n");
  drive(controller.Calculate(odometry.GetPose(), state, rotation));
}

void Drivetrain::trajectoryDrive(PathPlannerTrajectory::PathPlannerState const &state)
{
  printf("Driving based on inputted PathPlanner state\n");
  drive(controller.Calculate(odometry.GetPose(), state.pose, state.velocity, state.holonomicRotation));
}

static std::thread trajectory_thread;

bool trajectory_stop_flag = false;

void Drivetrain::trajectoryAutonDrive(frc::Trajectory const &traj, frc::Rotation2d const &faceAngle)
{
  printf("Interpreting trajectory\n");
  trajectory_stop_flag = true; // stop previous thread (this is only here as a safety feature in case method gets called twice)
  if (trajectory_thread.joinable())
    trajectory_thread.join();
  trajectory_stop_flag = false;
  trajectory_thread = std::thread{[&traj, &faceAngle]()
                                  {
                                    printf("Beginning trajectory sampling\n");
                                    odometry.ResetPosition(traj.Sample(0_s).pose, getHeading());
                                    frc::Timer trajTimer;
                                    trajTimer.Start();
                                    int trajectory_samples = 0;
                                    while (!trajectory_stop_flag && RobotState::IsAutonomousEnabled() && trajTimer.Get() <= traj.TotalTime())
                                    {
                                      using namespace std::literals::chrono_literals;
                                      printf("Current trajectory sample value: %d\n", ++trajectory_samples);
                                      trajectoryDrive(traj.Sample(trajTimer.Get()), faceAngle);
                                      std::this_thread::sleep_for(10ms); // Don't kill CPU
                                    }
                                    faceDirection(0_mps, 0_mps, faceAngle.Degrees());
                                  }};
}

void Drivetrain::trajectoryAutonDrive(pathplanner::PathPlannerTrajectory traj)
{
  printf("Interpreting PathPlanner trajectory\n");
  trajectory_stop_flag = true; // stop previous thread (this is only here as a safety feature in case method gets called twice)
  if (trajectory_thread.joinable())
    trajectory_thread.join();
  trajectory_stop_flag = false;
  trajectory_thread = std::thread{[&traj]()
                                  {
                                    printf("Beginning trajectory sampling\n");
                                    auto const inital_state = traj.getInitialState();
                                    odometry.ResetPosition(inital_state->pose, inital_state->holonomicRotation);
                                    frc::Timer trajTimer;
                                    trajTimer.Start();
                                    int trajectory_samples = 0;
                                    while (!trajectory_stop_flag && RobotState::IsAutonomousEnabled() && trajTimer.Get() <= traj.getTotalTime())
                                    {
                                      using namespace std::literals::chrono_literals;
                                      printf("Current trajectory sample value: %d\n", ++trajectory_samples);
                                      trajectoryDrive(traj.sample(trajTimer.Get()));
                                      std::this_thread::sleep_for(10ms); // Don't kill CPU
                                    }
                                    faceDirection(0_mps, 0_mps, traj.getEndState()->holonomicRotation.Degrees());
                                  }};
}