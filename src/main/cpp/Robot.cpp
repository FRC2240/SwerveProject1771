#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"
#include "Autons.hpp"
#include "Intake.hpp"
#include "ShooterWheel.hpp"
#include "Hood.hpp"
// #include "Turret.hpp"
#include "Hopper.hpp"
#include "Limelight.hpp"
#include "ngr.hpp"
#include "Climber.hpp"

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
  shooterTempUpdate();

  buttonManager();

  swerveDrive(field_centric);

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
  /*
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
      */
}

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/

void Robot::buttonManager()
{
  if (BUTTON::oStick.GetThrottle() > 0)
    ShooterWheel::bangbang();
  else
    ShooterWheel::stop();

  bool target_locked = false;
  bool deploy_intake = false;

  if (BUTTON::SHOOTER::AIM_FRONT)
  {
    deploy_intake = true;
    target_locked = aim(Turret::POSITION::FRONT);
  }
  else if (BUTTON::SHOOTER::AIM_BACK)
  {
    deploy_intake = true;
    target_locked = aim(Turret::POSITION::BACK);
  }
  else if (BUTTON::SHOOTER::BATTERSHOT)
  {
    deploy_intake = true;

    // turret_in_pos is true when it's safe to deploy hood
    bool const turret_in_pos = Turret::goToPosition(Turret::POSITION::FRONT,
                                                    std::abs(Turret::POSITION::FRONT - Turret::POSITION::SAFE_TO_DEPLOY_HOOD_FRONT));
    if (turret_in_pos)
      target_locked = Hood::goToPosition(Hood::POSITION::BATTER);
    else
      Hood::goToPosition(Hood::POSITION::TRAVERSE);
  }
  else if (BUTTON::SHOOTER::AIM_SIDE)
  {
    deploy_intake = true;
    target_locked = Hood::goToPosition(Hood::POSITION::MIDPOINT);
  }
  else
  {
    deploy_intake = false;
    if (Hood::goToPosition(Hood::POSITION::BOTTOM, std::abs(Hood::POSITION::SAFE_TO_TURN)))
      Turret::goToPosition(Turret::POSITION::ZERO);

    if (BUTTON::DRIVETRAIN::ROTATION_MODE.getRawButtonPressed())
      rotation_joystick = !rotation_joystick;

    if (BUTTON::DRIVETRAIN::FIELD_CENTRIC.getRawButtonPressed())
      field_centric = !field_centric;
  }

  Intake::deploy(BUTTON::INTAKE::DEPLOY || deploy_intake);

  if (BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
    Hopper::stop();
  if (target_locked && BUTTON::SHOOTER::SHOOT)
    Hopper::shoot();
  else if (!BUTTON::SHOOTER::SHOOT)
    Hopper::index();

  if (BUTTON::INTAKE::INTAKE)
    Intake::drive(Intake::DIRECTION::IN);
  else if (BUTTON::INTAKE::RETRACT)
    Intake::drive(Intake::DIRECTION::OUT);
  else
    Intake::drive(Intake::DIRECTION::OFF);

  Climber::buttonManager();
}

bool Robot::aim(Turret::POSITION direction)
{
  if (auto [is_tracking, readyToShoot] = Turret::visionTrack(direction); is_tracking)
    return Hood::visionTrack() && readyToShoot;
  Hood::goToPosition(Hood::POSITION::TRAVERSE);
  return false;
}

void Robot::tankDrive()
{
  auto const l_speed = -frc::ApplyDeadband(BUTTON::PS5.GetY(), 0.08);
  auto const r_speed = -frc::ApplyDeadband(BUTTON::PS5.GetTwist(), 0.08);

  Drivetrain::tankDrive(l_speed, r_speed);
}

static bool overheating_flash_red = true;

bool Robot::shooterTempUpdate()
{
  frc::SmartDashboard::PutNumber("Shooter Temp", ShooterWheel::getTemp());
  // printf("\n Shooter Temp: %f", ShooterWheel::getTemp());
  if (ShooterWheel::getTemp() > 70)
  {
    // oscillating between green & red to grab attention
    if (overheating_flash_red)
    {
      frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", false);
      overheating_flash_red = false;
    }
    else
    {
      frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", true);
      overheating_flash_red = true;
    }
    // BUTTON::PS5.SetRumble(BUTTON::PS5.kLeftRumble, .7);
    // BUTTON::PS5.SetRumble(BUTTON::PS5.kRightRumble, .7);
  }
  else
  {
    frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", true);
    // BUTTON::PS5.SetRumble(BUTTON::PS5.kLeftRumble, 0);
    // BUTTON::PS5.SetRumble(BUTTON::PS5.kRightRumble, 0);
  }
  return ShooterWheel::getTemp() > 70;
}

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

void Robot::swerveDrive(bool const &field_relative)
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