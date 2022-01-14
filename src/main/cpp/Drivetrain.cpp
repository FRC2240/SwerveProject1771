#include "Drivetrain.hpp"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// All variables interacting with hardware need initalized in init()
// to avoid issues with initalizing before wpilib
namespace Module
{
  inline static SwerveModule front_left{40, 41, 12, {11_in, 11_in}};
  inline static SwerveModule front_right{30, 31, 11, {11_in, -11_in}};
  inline static SwerveModule back_left{50, 51, 13, {-11_in, 11_in}};
  inline static SwerveModule back_right{60, 61, 14, {-11_in, -11_in}};
}

inline static const frc::SwerveDriveKinematics<4> kinematics{Module::front_left,
                                                               Module::front_right,
                                                               Module::back_left,
                                                               Module::back_right};

inline static std::unique_ptr<AHRS> navx = nullptr;

inline static frc::SwerveDriveOdometry<4> odometry{kinematics, frc::Rotation2d{0_deg}};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

  resetGyro();
}

void Drivetrain::resetGyro() { navx->ZeroYaw(); }

units::degree_t Drivetrain::getAngle() { return units::degree_t{-navx->GetAngle() - 90}; }

frc::Pose2d Drivetrain::getOdometryPose() { return odometry.GetPose(); }

frc::Rotation2d Drivetrain::getHeading() { return {getAngle()}; }

void Drivetrain::drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  auto states = kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kinematics.DesaturateWheelSpeeds(&states, K_MAX_SPEED);

  auto [fl, fr, bl, br] = states;

  Module::front_left.setDesiredState(fl);
  Module::front_right.setDesiredState(fr);
  Module::back_left.setDesiredState(bl);
  Module::back_right.setDesiredState(br);
}

void Drivetrain::updateOdometry()
{
  odometry.Update(getHeading(),
                   Module::front_left.getState(),
                   Module::front_right.getState(),
                   Module::back_left.getState(),
                   Module::back_right.getState());
}