#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"
#include "Autons.hpp"
#include "Intake.hpp"
#include "ShooterWheel.hpp"
#include "Hood.hpp"
#include "Turret.hpp"
#include "Hopper.hpp"
#include "Limelight.hpp"
#include "ngr.hpp"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static auto rotation_joystick = false;

static frc::SendableChooser<std::function<void()>> traj_selector;

static auto field_centric = true;

/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

LimeLight camera{};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

Robot::Robot()
{
  // setup RobotStates
  RobotState::IsEnabled = [this]()
  { return IsEnabled(); };

  RobotState::IsDisabled = [this]()
  { return IsDisabled(); };

  RobotState::IsAutonomous = [this]()
  { return IsAutonomous(); };

  RobotState::IsAutonomousEnabled = [this]()
  { return IsAutonomousEnabled(); };

  RobotState::IsTeleop = [this]()
  { return IsTeleop(); };

  RobotState::IsTeleopEnabled = [this]()
  { return IsTeleopEnabled(); };

  RobotState::IsTest = [this]()
  { return IsTest(); };

  // Call the inits for all subsystems here
  Drivetrain::init();
  Trajectory::putField2d();

  // Add all paths here

  for (auto &[name, auton] : autons)
  {
    if (name.find("Default") != std::string::npos)
      traj_selector.SetDefaultOption(name, auton);
    else
      traj_selector.AddOption(name, auton);
  }

  frc::SmartDashboard::PutData("Traj Selector", &traj_selector);

  frc::SmartDashboard::PutBoolean("Traj Reversed", Trajectory::reverse_trajectory);

  // This is the second joystick's Y axis
  BUTTON::PS5.SetTwistChannel(5);
}

void Robot::RobotInit()
{
  Trajectory::putField2d();
}

void Robot::RobotPeriodic()
{
  Trajectory::reverse_trajectory = frc::SmartDashboard::GetBoolean("Traj Reversed", Trajectory::reverse_trajectory);
}

void Robot::AutonomousInit()
{
  // Start aiming

  traj_selector.GetSelected()();

  Drivetrain::drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);

  // If driving after "stop" is called is a problem, I will add a "stop" method
  //  which runs a few times to ensure all modules are stopped

  // Will only finish after trajectory is done, so we can add additional trajectories and timers to intake & shoot
}

void Robot::AutonomousPeriodic()
{
  // This is what gets called after Init()
  Drivetrain::drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
  if (BUTTON::DRIVETRAIN::ROTATION_MODE.getRawButtonPressed())
    rotation_joystick = !rotation_joystick;

  if (BUTTON::DRIVETRAIN::FIELD_CENTRIC.getRawButtonPressed())
    field_centric = !field_centric;

  driveWithJoystick(field_centric);

  Trajectory::updateOdometry();

  if constexpr (debugging)
  {
    Trajectory::printEstimatedSpeeds();
    Trajectory::printRealSpeeds();
  }
}

void Robot::TestInit()
{
  /*
  Intake::deploy(true);
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1s);
  Hood::goToPosition(Hood::POSITION::MIDPOINT);
  Turret::goToPosition(Turret::POSITION::FRONT);
  */
}

void Robot::TestPeriodic()
{
  /*
  ShooterWheel::bangbang();
  Hopper::index(false);
    if(BUTTON::DRIVETRAIN::TURN_90.getRawButtonReleased())
        Hopper::stop();
    if(BUTTON::DRIVETRAIN::TURN_90)
        Hopper::shoot();
*/
  if (BUTTON::DRIVETRAIN::TURN_45)
    Drivetrain::manualVelocity(7500);
  else if (BUTTON::DRIVETRAIN::TURN_90)
    Drivetrain::manualVelocity(10000);
  else if (BUTTON::DRIVETRAIN::TURN_neg45)
    Drivetrain::manualVelocity(2500);
  else if (BUTTON::DRIVETRAIN::TURN_neg90)
    Drivetrain::manualVelocity(5000);
  else
    Drivetrain::manualVelocity(0);
}

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/

void Robot::tunePID()
{
  if (BUTTON::DRIVETRAIN::TURN_45)
  {
    Drivetrain::tuneTurner(45_deg);
  }
  else if (BUTTON::DRIVETRAIN::TURN_neg45)
  {
    Drivetrain::tuneTurner(-45_deg);
  }
  else if (BUTTON::DRIVETRAIN::TURN_90)
  {
    Drivetrain::tuneTurner(90_deg);
  }
  else if (BUTTON::DRIVETRAIN::TURN_neg90)
  {
    Drivetrain::tuneTurner(-90_deg);
  }
  else
  {
    Drivetrain::tuneTurner(0_deg);
  }
}

void Robot::driveWithJoystick(bool const &field_relative)
{
  auto const left_right = -frc::ApplyDeadband(BUTTON::PS5.GetX(), 0.08) * Drivetrain::TELEOP_MAX_SPEED;
  auto const front_back = -frc::ApplyDeadband(BUTTON::PS5.GetY(), 0.08) * Drivetrain::TELEOP_MAX_SPEED;
  if (BUTTON::DRIVETRAIN::ROTATE_FRONT)
    Drivetrain::faceDirection(front_back, left_right, 0_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_BACK)
    Drivetrain::faceDirection(front_back, left_right, 180_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    Drivetrain::faceClosest(front_back, left_right, field_relative);
  else if (rotation_joystick)
  {
    // Multiplied by 10 to avoid rounding to 0 by the atan2() method
    double const rotate_joy_x = BUTTON::PS5.GetZ() * 10;
    double const rotate_joy_y = -BUTTON::PS5.GetTwist() * 10;

    // If we aren't actually pressing the joystick, leave rotation at previous
    if (abs(rotate_joy_x) > 0.1 || abs(rotate_joy_y) > 0.1)
    {
      // Get degree using arctan, then convert from unit circle to front-centered values with positive being CW
      Drivetrain::faceDirection(front_back, left_right, -units::radian_t{atan2(rotate_joy_y, rotate_joy_x)} + 90_deg, field_relative);
    }
    else
      Drivetrain::drive(front_back, left_right, units::radians_per_second_t{0}, field_relative);
  }
  else
  {
    auto const rot = -frc::ApplyDeadband(BUTTON::PS5.GetZ(), 0.04) * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, left_right, rot, field_relative);
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif