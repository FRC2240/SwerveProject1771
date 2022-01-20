#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"

#include <frc/MathUtil.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

local bool rotation_joystick = false;
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

  Drivetrain::init();

  BUTTON::PS5.SetTwistChannel(5);
}

void Robot::AutonomousInit()
{
  testPathPlanner();

  Drivetrain::updateOdometry();
}

void Robot::AutonomousPeriodic()
{
  Drivetrain::updateOdometry();
}

void Robot::TeleopPeriodic()
{
  if (BUTTON::DRIVETRAIN::ROTATION_MODE.getRawButtonPressed())
  {
    if (rotation_joystick)
      rotation_joystick = false;
    else
      rotation_joystick = true;
  }

  driveWithJoystick(false);

  Drivetrain::updateOdometry();
}

void Robot::TestPeriodic()
{

  if (BUTTON::DRIVETRAIN::TURN_90)
    Drivetrain::testHolonomicRotation(90_deg);
  else
    driveWithJoystick(true);

  Drivetrain::updateOdometry();
  Drivetrain::printOdometryPose();
}

void Robot::tunePID()
{
  if (BUTTON::DRIVETRAIN::TURN_45)
  {
    Drivetrain::setAngleForTuning(45_deg);
  }
  else if (BUTTON::DRIVETRAIN::TURN_neg45)
  {
    Drivetrain::setAngleForTuning(-45_deg);
  }
  else if (BUTTON::DRIVETRAIN::TURN_90)
  {
    Drivetrain::setAngleForTuning(90_deg);
  }
  else if (BUTTON::DRIVETRAIN::TURN_neg90)
  {
    Drivetrain::setAngleForTuning(-90_deg);
  }
  else
  {
    Drivetrain::setAngleForTuning(0_deg);
  }
}

void Robot::testPathPlanner()
{
  using namespace pathplanner;
  Drivetrain::trajectoryAutonDrive(PathPlanner::loadPath("30 degree turn", 5_fps, 5_fps_sq));
}

void Robot::driveWithJoystick(bool const &field_relative)
{
  /*
  fmt::print("Printing PS5 Inputs: X: {}, Y: {}, Z: {}, Twist: {}\n", BUTTON::PS5.GetX(), BUTTON::PS5.GetY(), BUTTON::PS5.GetZ(), BUTTON::PS5.GetTwist());
  fmt::print("Printing PS5 Channels: X: {}, Y: {}, Z: {}, Twist: {}\n", BUTTON::PS5.GetXChannel(), BUTTON::PS5.GetYChannel(), BUTTON::PS5.GetZChannel(), BUTTON::PS5.GetTwistChannel());
  fmt::print("Printing Direction (degrees): {}\n", BUTTON::PS5.GetDirectionDegrees());
  */

  auto const left_right = frc::ApplyDeadband(BUTTON::PS5.GetX(), 0.04) * Drivetrain::ROBOT_MAX_SPEED;
  auto const front_back = frc::ApplyDeadband(BUTTON::PS5.GetY(), 0.04) * Drivetrain::ROBOT_MAX_SPEED;
  if (BUTTON::DRIVETRAIN::ROTATE_FRONT)
    Drivetrain::faceDirection(front_back, left_right, 0_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_BACK)
    Drivetrain::faceDirection(front_back, left_right, 180_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    Drivetrain::faceClosest(front_back, left_right, field_relative);
  else if (rotation_joystick)
  {
    double const rotate_joy_x = frc::ApplyDeadband(BUTTON::PS5.GetZ(), 0.04);
    double const rotate_joy_y = -frc::ApplyDeadband(BUTTON::PS5.GetTwist(), 0.04);

    // If we aren't actually pressing the joystick, leave rotation at previous
    if (abs(rotate_joy_x) > 0.2 || abs(rotate_joy_y) > 0.2)
    {
      // Get degree using arctan, then convert from unit circle to normal CW values
      units::degree_t const direction = -units::radian_t{atan2(rotate_joy_y, rotate_joy_x)} + 90_deg;

      Drivetrain::faceDirection(front_back, left_right, direction, field_relative);
    }
    else
      Drivetrain::drive(front_back, left_right, units::radians_per_second_t{0}, field_relative);
  }
  else
  {
    auto const rot = frc::ApplyDeadband(BUTTON::PS5.GetZ(), 0.04) * Drivetrain::ROBOT_MAX_ANGULAR_SPEED; // Might need to be inverted in the future

    Drivetrain::drive(front_back, left_right, rot, field_relative);
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif