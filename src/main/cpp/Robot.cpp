#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"
#include <Trajectory.hpp>

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

local bool rotation_joystick = false;

local frc::SendableChooser<std::string> traj_chooser;

local bool field_centric = true;

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

  /*
     for(auto const& dir_entry : std::filesystem::directory_iterator{std::filesystem::path{frc::filesystem::GetDeployDirectory() + "/pathplanner/"}})
     {
       traj_chooser.AddOption(dir_entry.path().filename().string());
     }
  */

  //Add all paths here
  traj_chooser.AddOption("30 Degree Turn", "30 degree turn");
  traj_chooser.AddOption("Straight Line", "Straight Line");
  traj_chooser.AddOption("T-Shape", "T shape");
  traj_chooser.SetDefaultOption("L with Rotate", "L with Rotate");

  frc::SmartDashboard::PutData("Traj Selector", &traj_chooser);

  // This is the second joystick's Y axis
  BUTTON::PS5.SetTwistChannel(5);
}

void Robot::RobotInit()
{
  Trajectory::putField2d();
}

void Robot::AutonomousInit()
{
  //Start aiming
  //Deploy intake
  using namespace pathplanner;
  Trajectory::follow(PathPlanner::loadPath(traj_chooser.GetSelected(), Drivetrain::TRAJ_MAX_SPEED, Drivetrain::TRAJ_MAX_ACCELERATION));
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

void Robot::TestPeriodic()
{

  if (BUTTON::DRIVETRAIN::TURN_90)
    Trajectory::testHolonomic({1_m, 1_m, 30_deg}, 1_fps, {30_deg});
  else
  {
    driveWithJoystick(field_centric);
  }

  Trajectory::printEstimatedSpeeds();
  Trajectory::printRealSpeeds();
  Trajectory::updateOdometry();
}

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/

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

void Robot::driveWithJoystick(bool const &field_relative)
{
  if (debugging)
  {
    fmt::print("Printing PS5 Inputs: X: {}, Y: {}, Z: {}, Twist: {}\n", BUTTON::PS5.GetX(), BUTTON::PS5.GetY(), BUTTON::PS5.GetZ(), BUTTON::PS5.GetTwist());
    fmt::print("Printing PS5 Channels: X: {}, Y: {}, Z: {}, Twist: {}\n", BUTTON::PS5.GetXChannel(), BUTTON::PS5.GetYChannel(), BUTTON::PS5.GetZChannel(), BUTTON::PS5.GetTwistChannel());
    fmt::print("Printing Direction (degrees): {}\n", BUTTON::PS5.GetDirectionDegrees());
  }

  auto const left_right = -frc::ApplyDeadband(BUTTON::PS5.GetX(), 0.08) * Drivetrain::ROBOT_MAX_SPEED;
  auto const front_back = -frc::ApplyDeadband(BUTTON::PS5.GetY(), 0.08) * Drivetrain::ROBOT_MAX_SPEED;
  if (BUTTON::DRIVETRAIN::ROTATE_FRONT)
    Drivetrain::faceDirection(front_back, left_right, 0_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_BACK)
    Drivetrain::faceDirection(front_back, left_right, 90_deg, field_relative);
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
    auto const rot = -frc::ApplyDeadband(BUTTON::PS5.GetZ(), 0.04) * Drivetrain::ROBOT_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, left_right, rot, field_relative);
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif