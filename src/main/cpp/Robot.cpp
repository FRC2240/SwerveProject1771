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

bool rotation_directly_controlled_by_joystick = false;
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

void Robot::AutonomousInit()
{
  Drivetrain::updateOdometry();
  testTrajectory();
  // testPathPlanner();
}

void Robot::AutonomousPeriodic()
{
  fmt::print("Testing Modulo Operator: -500 % 360 should equal -140: {}", -500 % 360);
  // driveWithJoystick(false);
  Drivetrain::updateOdometry();
}

void Robot::TeleopPeriodic()
{
  driveWithJoystick(true);

  if (BUTTON::DRIVETRAIN::ROTATION_MODE.getRawButtonPressed())
  {
    if (rotation_directly_controlled_by_joystick)
      rotation_directly_controlled_by_joystick = false;
    else
      rotation_directly_controlled_by_joystick = true;
  }
}

  void Robot::TestPeriodic()
  {
    if (BUTTON::DRIVETRAIN::TURN_45)
    {
      Drivetrain::setAngleForTesting(45_deg);
    }
    else if (BUTTON::DRIVETRAIN::TURN_neg45)
    {
      Drivetrain::setAngleForTesting(-45_deg);
    }
    else if (BUTTON::DRIVETRAIN::TURN_90)
    {
      Drivetrain::setAngleForTesting(90_deg);
    }
    else if (BUTTON::DRIVETRAIN::TURN_neg90)
    {
      Drivetrain::setAngleForTesting(-90_deg);
    }
    else
    {
      Drivetrain::setAngleForTesting(0_deg);
    }
  }

  void Robot::testTrajectory()
  {
    Drivetrain::printOdometryPose();
    printf("Creating trajectory");
    auto config = frc::TrajectoryConfig(0.35_mps, 0.5_mps_sq);
    config.SetKinematics<4>(const_cast<frc::SwerveDriveKinematics<4> &>(Drivetrain::getKinematics()));
    // auto startPos = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0));
    frc::Pose2d const startPos;
    frc::Pose2d const endPos{1_m, 1_m, 0_deg};
    std::vector<frc::Translation2d> interiorPos{
        //     frc::Translation2d{.5_m, .25_m},
        //     frc::Translation2d{.7_m, .5_m}
    };

    auto traj = frc::TrajectoryGenerator::GenerateTrajectory(startPos, interiorPos, endPos, config);
    printf("Passing trajectory to auton drive");
    Drivetrain::trajectoryAutonDrive(traj, frc::Rotation2d{0_deg});
  }

  void Robot::testPathPlanner()
  {
    using namespace pathplanner;
    Drivetrain::trajectoryAutonDrive(PathPlanner::loadPath("New Path", 0.35_mps, 0.5_mps_sq));
  }

  void Robot::driveWithJoystick(bool const &field_relative)
  {
    // printf("Printing PS5 Inputs: X: %f, Y: %f, Z: %f\n", BUTTON::ps5.GetX(), BUTTON::ps5.GetY(), BUTTON::ps5.GetZ());

    auto const x_speed = frc::ApplyDeadband(BUTTON::ps5.GetX(), 0.04) * Drivetrain::ROBOT_MAX_SPEED;
    auto const y_speed = -frc::ApplyDeadband(BUTTON::ps5.GetY(), 0.04) * Drivetrain::ROBOT_MAX_SPEED;
  if (BUTTON::DRIVETRAIN::ROTATE_FRONT)
    Drivetrain::faceDirection(x_speed, y_speed, 0_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_BACK)
    Drivetrain::faceDirection(x_speed, y_speed, 180_deg, field_relative);
  else if (BUTTON::DRIVETRAIN::ROTATE_FRONT)
    Drivetrain::faceClosest(x_speed, y_speed, field_relative);
  else if (rotation_directly_controlled_by_joystick)
  {
    units::degree_t rot{0};
    double const rotate_joy_x = frc::ApplyDeadband(BUTTON::ps5.GetZ(), 0.2);
    double const rotate_joy_y = frc::ApplyDeadband(BUTTON::ps5.GetTwist(), 0.2);

    auto const direction = units::degree_t(units::radian_t{rotate_joy_y, rotate_joy_x}) - 90_deg;

    Drivetrain::faceDirection(x_speed, y_speed, direction, field_relative);
  }
  else
  {
    auto const rot = frc::ApplyDeadband(BUTTON::ps5.GetZ(), 0.04) * Drivetrain::ROBOT_MAX_ANGULAR_SPEED; // Might need to be inverted in the future

    Drivetrain::drive(x_speed, y_speed, rot, field_relative);
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
