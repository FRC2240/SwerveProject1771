#include "Drivetrain.hpp"
#include "SwerveModule.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

#include <AHRS.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

// faceDirection && faceClosest constants
local constexpr double ROTATE_P = 1.75; // Modifier for rotational speed -> (degree * ROTATE_P)

local constexpr units::degrees_per_second_t MAX_FACE_DIRECTION_SPEED = 150_deg / 1_s; // only used for faceDirection

/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib

namespace Module
{
  SwerveModule front_left{40, 41, 12, -344.53125};
  SwerveModule front_right{30, 31, 11, -264.726563};
  SwerveModule back_left{50, 51, 13, -91.54203};
  SwerveModule back_right{60, 61, 14, -285.0293};
}

frc::SwerveDriveKinematics<4> kinematics{frc::Translation2d{11_in, 11_in},
                                         frc::Translation2d{11_in, -11_in},
                                         frc::Translation2d{-11_in, 11_in},
                                         frc::Translation2d{-11_in, -11_in}};

std::unique_ptr<AHRS> navx = nullptr;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

  resetGyro();

  Module::front_left.init();
  Module::front_right.init();
  Module::back_left.init();
  Module::back_right.init();
}

void Drivetrain::resetGyro() { navx->ZeroYaw(); }

// Returns values with 0 being front and positive angles going CCW
units::degree_t Drivetrain::getAngle()
{
  double navx_angle = navx->GetAngle();
  if (navx_angle < -180)
    navx_angle += 360; // Ensure angle is between 0 and 360
  if (navx_angle > 180)
    navx_angle -= 360; // Optimizes angle if over 180
  return units::degree_t{navx_angle};
}

// Returns values with 0 being front and positive angles going CCW
frc::Rotation2d Drivetrain::getHeading() { return {getAngle()}; }

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
  frc::SmartDashboard::PutNumber("Target VX Speed", speeds.vx.value());
  frc::SmartDashboard::PutNumber("Target VY Speed", speeds.vy.value());
  frc::SmartDashboard::PutNumber("Target Omega Speed (CCW is +)", units::degrees_per_second_t{speeds.omega}.value());
  drive(kinematics.ToSwerveModuleStates(speeds));
}

// Sets each module to the desired state
void Drivetrain::drive(wpi::array<frc::SwerveModuleState, 4> states)
{

  kinematics.DesaturateWheelSpeeds(&states, MODULE_MAX_SPEED);

  auto const [fl, fr, bl, br] = states;

  Module::front_left.setDesiredState(fl);
  Module::front_right.setDesiredState(fr);
  Module::back_left.setDesiredState(bl);
  Module::back_right.setDesiredState(br);
}

/******************************************************************/
/*                        Facing Functions                        */
/******************************************************************/

void Drivetrain::setAngleForTuning(units::degree_t const &desired_angle)
{
  Module::front_left.setTurnerAngle(desired_angle);
}

// For theta, positive is CCW
void Drivetrain::faceDirection(units::meters_per_second_t const &dx, units::meters_per_second_t const &dy, units::degree_t const &theta, bool const &field_relative)
{
  int error_theta = (theta - getAngle()).to<int>() % 360; // Get difference between old and new angle; gets the equivalent value between -360 and 360
  if (error_theta < -180)
    error_theta += 360; // Ensure angle is between -180 and 360
  if (error_theta > 180)
    error_theta -= 360; // Optimizes angle if over 180
  if (abs(error_theta) < 10)
    error_theta = 0;
  double p_rotation = error_theta * ROTATE_P; // Modifies error_theta in order to get a faster turning speed
  if (abs(p_rotation) > MAX_FACE_DIRECTION_SPEED.value())
    p_rotation = MAX_FACE_DIRECTION_SPEED.value() * ((p_rotation > 0) ? 1 : -1); // Constrains turn speed
  drive(dx, dy, units::degrees_per_second_t{p_rotation}, field_relative);
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