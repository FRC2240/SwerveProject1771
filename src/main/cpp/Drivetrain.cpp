#include "Drivetrain.hpp"
#include "SwerveModule.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

#include <AHRS.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

// faceDirection && faceClosest constants
constexpr auto ROTATE_P = 1.75; // Modifier for rotational speed -> (degree * ROTATE_P)

constexpr units::degrees_per_second_t
    MAX_FACE_DIRECTION_SPEED = 150_deg / 1_s; // only used for faceDirection

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

auto first_time_getting_angle = true;

/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib

namespace Module
{
  std::unique_ptr<SwerveModule> front_left;
  std::unique_ptr<SwerveModule> front_right;
  std::unique_ptr<SwerveModule> back_left;
  std::unique_ptr<SwerveModule> back_right;
}

frc::SwerveDriveKinematics<4> kinematics{frc::Translation2d{11_in, 11_in},
                                         frc::Translation2d{11_in, -11_in},
                                         frc::Translation2d{-11_in, 11_in},
                                         frc::Translation2d{-11_in, -11_in}};

local std::unique_ptr<AHRS> navx;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

  using namespace Module;
  front_left = std::make_unique<SwerveModule>(60, 61, 14, -285.0293);
  front_right = std::make_unique<SwerveModule>(50, 51, 13, -91.54203);
  back_left = std::make_unique<SwerveModule>(30, 31, 11, -264.726563);
  back_right = std::make_unique<SwerveModule>(40, 41, 12, -344.53125);
}

// Returns values with 0 being front and positive angles going CW
units::degree_t Drivetrain::getAngle()
{
  if (first_time_getting_angle)
  {
    navx->ZeroYaw(); // This can't be called in init() since the gyro will still be calibrating
    first_time_getting_angle = false;
  }
  return units::degree_t{navx->GetAngle()};
}

frc::Rotation2d Drivetrain::getCCWHeading() { return {-getAngle()}; }

frc::Rotation2d Drivetrain::getCWHeading() { return {getAngle()}; }

/******************************************************************/
/*                       Driving Functions                        */
/******************************************************************/

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
  drive(kinematics.ToSwerveModuleStates(speeds));

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

  kinematics.DesaturateWheelSpeeds(&states, MODULE_MAX_SPEED);

  auto const [fl, fr, bl, br] = states;

  Module::front_left->setDesiredState(fl);
  Module::front_right->setDesiredState(fr);
  Module::back_left->setDesiredState(bl);
  Module::back_right->setDesiredState(br);

  if constexpr (debugging)
  {
    frc::SmartDashboard::PutString("Target Front Left Module", fmt::format("Speed (mps): {}, Direction: {}", fl.speed.value(), fl.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Front Right Module", fmt::format("Speed (mps): {}, Direction: {}", fr.speed.value(), fr.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Back Left Module", fmt::format("Speed (mps): {}, Direction: {}", bl.speed.value(), bl.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Target Back Right Module", fmt::format("Speed (mps): {}, Direction: {}", br.speed.value(), br.angle.Degrees().value()));

    auto const fl_old = Module::front_left->getState();
    auto const fr_old = Module::front_left->getState();
    auto const bl_old = Module::back_left->getState();
    auto const br_old = Module::back_right->getState();
    frc::SmartDashboard::PutString("Actual Front Left Module", fmt::format("Speed (mps): {}, Direction: {}", fl_old.speed, fl_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Front Right Module", fmt::format("Speed (mps): {}, Direction: {}", fr_old.speed.value(), fr_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Back Left Module", fmt::format("Speed (mps): {}, Direction: {}", bl_old.speed.value(), bl_old.angle.Degrees().value()));
    frc::SmartDashboard::PutString("Actual Back Right Module", fmt::format("Speed (mps): {}, Direction: {}", br_old.speed.value(), br_old.angle.Degrees().value()));
  }
}

/******************************************************************/
/*                        Facing Functions                        */
/******************************************************************/

void Drivetrain::faceDirection(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, units::degree_t const &theta, bool const &field_relative)
{
  int error_theta = (theta - getAngle()).to<int>() % 360; // Get difference between old and new angle; gets the equivalent value between -360 and 360

  if (error_theta < -180)
    error_theta += 360; // Ensure angle is between -180 and 360
  if (error_theta > 180)
    error_theta -= 360; // Optimizes angle if over 180
  if (abs(error_theta) < 10)
    error_theta = 0; // Dead-zone to prevent oscillation

  double p_rotation = error_theta * ROTATE_P; // Modifies error_theta in order to get a faster turning speed

  if (abs(p_rotation) > MAX_FACE_DIRECTION_SPEED.value())
    p_rotation = MAX_FACE_DIRECTION_SPEED.value() * ((p_rotation > 0) ? 1 : -1); // Constrains turn speed

  // p_rotation is negated since the robot actually turns ccw, not cw
  drive(dx, dy, units::degrees_per_second_t{-p_rotation}, field_relative);
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

void Drivetrain::setAngleForTuning(units::degree_t const &desired_angle)
{
  Module::front_left->setDesiredState({0_mps, desired_angle});
  Module::front_right->setDesiredState({0_mps, desired_angle});
  Module::back_left->setDesiredState({0_mps, desired_angle});
  Module::back_right->setDesiredState({0_mps, desired_angle});
}