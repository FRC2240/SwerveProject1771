#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"

#include <frc/MathUtil.h>
#include <frc/filter/SlewRateLimiter.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
// to 1.
frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

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
}

void Robot::AutonomousPeriodic()
{
  // DriveWithJoystick(false);
  Drivetrain::updateOdometry();
}

void Robot::TeleopPeriodic() { DriveWithJoystick(true); }

void Robot::DriveWithJoystick(bool fieldRelative)
{
  // Get the x speed.
  auto const xSpeed = m_xspeedLimiter.Calculate(
                          frc::ApplyDeadband(BUTTON::ps5.GetX(), 0.04)) *
                      Drivetrain::K_MAX_SPEED;
  auto const ySpeed = -m_yspeedLimiter.Calculate(
                          frc::ApplyDeadband(BUTTON::ps5.GetY(), 0.04)) *
                      Drivetrain::K_MAX_SPEED;
  auto const rot = m_rotLimiter.Calculate( // Might need to be inverted in the future
                       frc::ApplyDeadband(BUTTON::ps5.GetZ(), 0.04)) *
                   Drivetrain::K_MAX_ANGULAR_SPEED;
  Drivetrain::drive(xSpeed, ySpeed, rot, fieldRelative);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
