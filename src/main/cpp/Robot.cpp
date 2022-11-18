#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"
#include "ngr.hpp"
#include "TempMonitoring.hpp"
#include "Odometry.hpp"
#include "Trajectory.hpp"


#include <frc/MathUtil.h>
#include <iostream>
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

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/


// Needed to flash on/off temp warnings on SmartDashboard/ShuffleBoard
//static bool drivers_flashing_red = false;
//static bool turners_flashing_red = false;

void monitorTemps()
{
  //TempMonitoring::monitorTemps(Drivetrain::getDriverTemps(), 70, "Driver Temps", "Drivers Overheating", drivers_flashing_red);
  //TempMonitoring::monitorTemps(Drivetrain::getTurnerTemps(), 60, "Turner Temps", "Turners Overheating", turners_flashing_red);
}

void buttonManager()
{
    if (BUTTON::DRIVETRAIN::ROTATION_MODE.getRawButtonPressed())
      rotation_joystick = !rotation_joystick;

    if (BUTTON::DRIVETRAIN::FIELD_CENTRIC.getRawButtonPressed())
      field_centric = !field_centric;
}

void tankDrive()
{
  auto const l_speed = -frc::ApplyDeadband(BUTTON::PS5.GetY(), 0.08);
  auto const r_speed = -frc::ApplyDeadband(BUTTON::PS5.GetTwist(), 0.08);

  Drivetrain::tankDrive(l_speed, r_speed);
}

void tunePID()
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

void tuneFF()
{
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

void swerveDrive(bool const &field_relative)
{
  auto const left_right = -frc::ApplyDeadband(BUTTON::PS5.GetX(), 0.08) * Drivetrain::TELEOP_MAX_SPEED;
  auto const front_back = -frc::ApplyDeadband(BUTTON::PS5.GetY(), 0.08) * Drivetrain::TELEOP_MAX_SPEED;
  //std::cout << "left_right = " << BUTTON::PS5.GetX() << "front_back = " << BUTTON::PS5.GetY() << std::endl;
  
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
    if (std::abs(rotate_joy_x) > 0.1 || std::abs(rotate_joy_y) > 0.1)
    {
      // Get degree using arctan, then convert from unit circle to front-centered values with positive being CW
      Drivetrain::faceDirection(front_back, left_right, -units::radian_t{atan2(rotate_joy_y, rotate_joy_x)} + 90_deg, field_relative);
    }
    else
      Drivetrain::drive(front_back, left_right, units::radians_per_second_t{0}, field_relative);
  }
  else
  {
    auto const rot = frc::ApplyDeadband(BUTTON::PS5.GetZ(), 0.04) * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, -left_right, rot, field_relative);
  }
}

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
  Odometry::putField2d();

  frc::SmartDashboard::PutData("Traj Selector", &traj_selector);

  frc::SmartDashboard::PutBoolean("Traj Reversed", Trajectory::reverse_trajectory);

  // This is the second joystick's Y axis
  BUTTON::PS5.SetTwistChannel(5);

  // Erik
  BUTTON::PS5.SetZChannel(4);
}

void Robot::RobotInit()
{
  Odometry::putField2d();
}

void Robot::RobotPeriodic()
{
  Trajectory::reverse_trajectory = frc::SmartDashboard::GetBoolean("Traj Reversed", Trajectory::reverse_trajectory);
  monitorTemps();
}

void Robot::AutonomousInit()
{
  // Start aiming

  traj_selector.GetSelected()();

  Drivetrain::stop();

  // If driving after "stop" is called is a problem, I will add a "stop" method
  //  which runs a few times to ensure all modules are stopped

  // Will only finish after trajectory is done, so we can add additional trajectories and timers to intake & shoot
}

void Robot::AutonomousPeriodic()
{
  // This is what gets called after Init()
  Drivetrain::stop();
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
  std::cout << "here0" << "\n";
  buttonManager();

  swerveDrive(field_centric);

  Odometry::update();

  if constexpr (debugging)
  {
    Trajectory::printRobotRelativeSpeeds();
    Trajectory::printFieldRelativeSpeeds();
  }
}

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif