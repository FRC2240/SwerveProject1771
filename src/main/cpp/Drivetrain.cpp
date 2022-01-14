#include "Drivetrain.hpp"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

namespace MODULEINFO
{
  inline static SwerveModuleInfo const FRONT_LEFT{40, 41, 12, {11_in, 11_in}};
  inline static SwerveModuleInfo const FRONT_RIGHT{30, 31, 11, {11_in, -11_in}};
  inline static SwerveModuleInfo const BACK_LEFT{50, 51, 13, {-11_in, 11_in}};
  inline static SwerveModuleInfo const BACK_RIGHT{60, 61, 14, {-11_in, -11_in}};
}

namespace Module
{
  inline static std::unique_ptr<SwerveModule> front_left = nullptr;
  inline static std::unique_ptr<SwerveModule> front_right = nullptr;
  inline static std::unique_ptr<SwerveModule> back_left = nullptr;
  inline static std::unique_ptr<SwerveModule> back_right = nullptr;
}

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

inline static std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics = nullptr;

inline static std::unique_ptr<AHRS> navx = nullptr;

inline static std::unique_ptr<frc::SwerveDriveOdometry<4>> odometry = nullptr;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init()
{
  resetGyro();

  navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

  Module::front_left = std::make_unique<SwerveModule>(MODULEINFO::FRONT_LEFT);
  Module::front_right = std::make_unique<SwerveModule>(MODULEINFO::FRONT_RIGHT);
  Module::back_left = std::make_unique<SwerveModule>(MODULEINFO::BACK_LEFT);
  Module::back_right = std::make_unique<SwerveModule>(MODULEINFO::BACK_RIGHT);

  kinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(*Module::front_left,
                                                               *Module::front_right,
                                                               *Module::back_left,
                                                               *Module::back_right);

  odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(*kinematics, Drivetrain::getHeading());
}

void Drivetrain::resetGyro() { navx->ZeroYaw(); }

units::degree_t Drivetrain::getAngle() { return units::degree_t{-navx->GetAngle() - 90}; }

frc::Pose2d Drivetrain::getOdometryPose() { return odometry->GetPose(); }

frc::Rotation2d Drivetrain::getHeading() { return {getAngle()}; }

void Drivetrain::drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  auto states = kinematics->ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kinematics->DesaturateWheelSpeeds(&states, K_MAX_SPEED);

  auto [fl, fr, bl, br] = states;

  Module::front_left->setDesiredState(fl);
  Module::front_right->setDesiredState(fr);
  Module::back_left->setDesiredState(bl);
  Module::back_right->setDesiredState(br);
}

void Drivetrain::updateOdometry()
{
  odometry->Update(getHeading(),
                  Module::front_left->getState(),
                  Module::front_right->getState(),
                  Module::back_left->getState(),
                  Module::back_right->getState());
}