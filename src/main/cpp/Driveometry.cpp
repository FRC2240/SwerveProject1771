#include "Driveometry.hpp"

// Odometry.cpp

frc::Field2d field2d;

// The numbers in pose2d have no bearing in reality and are stolen from docs.
// YOLO
    static frc::SwerveDriveOdometry<4> odometry{
        Odometry::kinematics,
        Drivetrain::getCCWHeading(),
        Drivetrain::get_module_pos(),
        frc::Pose2d{5_m, 13.5_m, 0_rad}
        };

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Odometry::putField2d()
{
    frc::SmartDashboard::PutData("Odometry Field", &field2d);
}

void Odometry::update()
{
    frc::Pose2d pose = odometry.Update(Drivetrain::getCCWHeading(),
                                       Drivetrain::get_module_pos());
    if constexpr (debugging)
        frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::Pose2d Odometry::getPose() { return odometry.GetPose(); }

frc::ChassisSpeeds const Odometry::getFieldRelativeSpeeds()
{
    // Init for first time
    static frc::Timer speed_timer;
    speed_timer.Start();
    static frc::Pose2d previous_pose{};

    frc::Pose2d const current_pose = odometry.GetPose();

    frc::Pose2d const delta_pose = current_pose.RelativeTo(previous_pose);

    auto const time_elapsed = speed_timer.Get();
    units::meters_per_second_t const X = delta_pose.X() / time_elapsed;

    units::meters_per_second_t const Y = delta_pose.Y() / time_elapsed;

    units::degrees_per_second_t const rot{delta_pose.Rotation().Degrees() / time_elapsed};

    previous_pose = odometry.GetPose(); // Set the previous_pose for the next time this loop is run

    speed_timer.Reset(); // Time how long until next call

    return frc::ChassisSpeeds{X, Y, rot};
}

void Odometry::resetPosition(const frc::Pose2d &pose, const frc::Rotation2d &gyro_angle)
{
    odometry.ResetPosition(gyro_angle, Drivetrain::get_module_pos(), pose);
}

frc::FieldObject2d *Odometry::getField2dObject(std::string_view name)
{
    return field2d.GetObject(name);
}

// Drivetrain.cpp
#include "ngr.hpp"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <AHRS.h>
#include <iostream>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static std::unique_ptr<AHRS> navx;
void Drivetrain::print_angle()
  {
    std::cout << "ANGLE: " << navx->GetAngle() << "\n";
  }
/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib

// These are "public" (not static) bc they are accessed by the Trajectory namespace

// This is not how it should be but doing it "correctly" (++,+-,-+,--) causes
// the wheels to form an "X" instead of diamond while turning.
// It's wrong but it works, no touchy.
// EDIT: Me touchy. In Odomentry now. Warning still applies.

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

}

// Returns values with 0 being front and positive angles going CW
units::degree_t Drivetrain::getAngle()
{
  static bool first_time_getting_angle = true;

  if (first_time_getting_angle)
  {
    navx->ZeroYaw(); // This can't be called in init() since the gyro will still be calibrating
    first_time_getting_angle = false;
  }
  return units::degree_t{navx->GetAngle()};
}
// IMPORTANT: CCW (counterclockwise) must not be inverted and CW (clockwise)
// must be. If CCW is negative and CW is positive, a 90 degree turn will
// cause feild centric inputs to be inverted.
// It's weird but the inversion as it stands is good and works, even though
// it seems odd.
frc::Rotation2d Drivetrain::getCCWHeading() { return {getAngle()}; }
// or navx->GetRotation()

frc::Rotation2d Drivetrain::getCWHeading() { return {-getAngle()}; }

//Names are hard. This is to fix type isssues.
frc::Rotation2d Drivetrain::test_heading() { return navx->GetRotation2d(); }

wpi::array<frc::SwerveModulePosition, 4> Drivetrain::get_module_pos()
  {
    return
      {
        Module::front_left.get_position(),
        Module::front_right.get_position(),
        Module::back_left.get_position(),
        Module::back_right.get_position()
      };
  }
wpi::array<double, 4> Drivetrain::getDriverTemps()
{
  using namespace Module;
  return {front_left.getDriverTemp(),
          front_right.getDriverTemp(),
          back_left.getDriverTemp(),
          back_right.getDriverTemp()};
}

wpi::array<double, 4> Drivetrain::getTurnerTemps()
{
  using namespace Module;
  return {front_left.getTurnerTemp(),
          front_right.getTurnerTemp(),
          back_left.getTurnerTemp(),
          back_right.getTurnerTemp()};
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds()
{
  return Odometry::kinematics.ToChassisSpeeds(Module::front_left.getState(),
                                    Module::front_right.getState(),
                                    Module::back_left.getState(),
                                    Module::back_right.getState());
}

const wpi::array<frc::SwerveModuleState, 4> Drivetrain::getModuleStates()
{
  return {Module::front_left.getState(),
          Module::front_right.getState(),
          Module::back_left.getState(),
          Module::back_right.getState()};
}
/******************************************************************/
/*                       Driving Functions                        */
/******************************************************************/

void Drivetrain::tankDrive(double const &l_speed, double const &r_speed)
{
  using namespace Module;
  front_left.percentOutputControl(l_speed);
  front_right.percentOutputControl(-r_speed);
  back_left.percentOutputControl(l_speed);
  back_right.percentOutputControl(-r_speed);
}
// Converts inputted speeds into a frc::ChassisSpeeds object
void Drivetrain::drive(units::meters_per_second_t const &xSpeed,
                       units::meters_per_second_t const &ySpeed,
                       units::radians_per_second_t const &rot,
                       bool const &fieldRelative)
{
  auto const speeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getCCWHeading())
                                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  drive(speeds);
}

// Takes the speed & direction the robot should be going and figures out the states for each indivdual module
void Drivetrain::drive(frc::ChassisSpeeds const &speeds)
{
  drive(Odometry::kinematics.ToSwerveModuleStates(speeds));

  if constexpr (debugging)
  {
    frc::SmartDashboard::PutNumber("Target VX Speed", speeds.vx.value());
    frc::SmartDashboard::PutNumber("Target VY Speed", speeds.vy.value());
    frc::SmartDashboard::PutNumber("Target Omega Speed (CCW is +)", units::degrees_per_second_t{speeds.omega}.value() / 720);
  }
}

// Sets each module to the desired state
void Drivetrain::drive(wpi::array<frc::SwerveModuleState, 4> states)
{
  Odometry::kinematics.DesaturateWheelSpeeds(&states, MODULE_MAX_SPEED);

  auto const [fl, fr, bl, br] = states;


  using namespace Module;
  front_left.setDesiredState(fl);
  front_right.setDesiredState(fr);
  back_left.setDesiredState(bl);
  back_right.setDesiredState(br);

  /*std::cout << "FL: " << front_left->getAngle().value()
            << "FR: " << front_right->getAngle().value()
            << "BL: " << back_left->getAngle().value()
            << "BR: " << back_right->getAngle().value()
            << "\n";*/


  if constexpr (debugging)
  {
    frc::SmartDashboard::PutString("Target Front Left Module", fmt::format("Speed (mps): {}, Direction: {}", fl.speed.value(), fl.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Front Right Module", fmt::format("Speed (mps): {}, Direction: {}", fr.speed.value(), fr.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Back Left Module", fmt::format("Speed (mps): {}, Direction: {}", bl.speed.value(), bl.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Back Right Module", fmt::format("Speed (mps): {}, Direction: {}", br.speed.value(), br.angle.Degrees().value()));

    auto const fl_old = front_left.getState();
    auto const fr_old = front_left.getState();
    auto const bl_old = back_left.getState();
    auto const br_old = back_right.getState();
    frc::SmartDashboard::PutString("Actual Front Left Module", fmt::format("Speed (mps): {}, Direction: {}", fl_old.speed, fl_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Front Right Module", fmt::format("Speed (mps): {}, Direction: {}", fr_old.speed.value(), fr_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Back Left Module", fmt::format("Speed (mps): {}, Direction: {}", bl_old.speed.value(), bl_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Back Right Module", fmt::format("Speed (mps): {}, Direction: {}", br_old.speed.value(), br_old.angle.Degrees().value()));
  }
}

void Drivetrain::stop()
{
  constexpr frc::SwerveModuleState stopped{0_mps, {}};

  using namespace Module;
  front_left.setDesiredState(stopped);
  front_right.setDesiredState(stopped);
  back_left.setDesiredState(stopped);
  back_right.setDesiredState(stopped);
}

/******************************************************************/
/*                        Facing Functions                        */
/******************************************************************/

void Drivetrain::faceDirection(units::meters_per_second_t const &dx,
                               units::meters_per_second_t const &dy,
                               units::degree_t const &theta,
                               bool const &field_relative,
                               double const &rot_p,
                               units::degrees_per_second_t const &max_rot_speed)
{
  int error_theta = (theta - getAngle()).to<int>() % 360; // Get difference between old and new angle;
                                                          // gets the equivalent value between -360 and 360

  if (error_theta < -180)
    error_theta += 360; // Ensure angle is between -180 and 360
  if (error_theta > 180)
    error_theta -= 360; // Optimizes angle if over 180
  if (std::abs(error_theta) < 5)
    error_theta = 0; // Dead-zone to prevent oscillation

  double p_rotation = error_theta * rot_p; // Modifies error_theta in order to get a faster turning speed

  if (std::abs(p_rotation) > max_rot_speed.value())
    p_rotation = max_rot_speed.value() * ((p_rotation > 0) ? 1 : -1); // Constrains turn speed

  // p_rotation is negated since the robot actually turns ccw, not cw
  drive(dx, dy, units::degrees_per_second_t{-p_rotation}, field_relative);
}

void Drivetrain::faceClosest(units::meters_per_second_t const &dx,
                             units::meters_per_second_t const &dy,
                             bool const &field_relative,
                             double const &rot_p,
                             units::degrees_per_second_t const &max_rot_speed)
{
  int current_rotation = getAngle().to<int>() % 360; // Ensure angle is between -360 and 360

  if (current_rotation < 0)
    current_rotation += 360; // Ensure angle is between 0 and 360

  if (current_rotation <= 90 || current_rotation >= 270)
    faceDirection(dx, dy, 0_deg, field_relative, rot_p, max_rot_speed);
  else
    faceDirection(dx, dy, 180_deg, field_relative, rot_p, max_rot_speed);
}

void Drivetrain::tuneTurner(units::degree_t const &desired_angle)
{
  using namespace Module;
  front_left.setDesiredState({0_mps, desired_angle});
  front_right.setDesiredState({0_mps, desired_angle});
  back_left.setDesiredState({0_mps, desired_angle});
  back_right.setDesiredState({0_mps, desired_angle});
}

void Drivetrain::manualPercentOutput(double const &percent_output)
{
  using namespace Module;
  front_left.percentOutputControl(percent_output);
  front_right.percentOutputControl(percent_output);
  back_left.percentOutputControl(percent_output);
  back_right.percentOutputControl(percent_output);
}

void Drivetrain::manualVelocity(double const &velocity_ticks_per_100ms)
{
  using namespace Module;
  front_left.manualVelocityContol(velocity_ticks_per_100ms);
  front_right.manualVelocityContol(velocity_ticks_per_100ms);
  back_left.manualVelocityContol(velocity_ticks_per_100ms);
  back_right.manualVelocityContol(velocity_ticks_per_100ms);
}

// You finished reading this...
// You poor soul
// Have some tea
/*
 *   /       \
 *     \
 *     /    /
 *    \
 *     \   \
 *  _____________  _
 * |=============|/A\
 * |             | U/
 * |_____________|_/
 *  \           /
 *   \_________/
 */