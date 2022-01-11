#include "Drivetrain.hpp"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

namespace MODULES
{
  SwerveModule FRONT_LEFT{40, 41, 12, {11_in, 11_in}};
  SwerveModule FRONT_RIGHT{30, 31, 11, {11_in, -11_in}};
  SwerveModule BACK_LEFT{50, 51, 13, {-11_in, 11_in}};
  SwerveModule BACK_RIGHT{60, 61, 14, {-11_in, -11_in}};
}

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

frc::SwerveDriveKinematics<4> m_kinematics{
    MODULES::FRONT_LEFT,
    MODULES::FRONT_RIGHT,
    MODULES::BACK_LEFT,
    MODULES::BACK_RIGHT};

inline static std::unique_ptr<AHRS> navx{std::make_unique<AHRS>(frc::SPI::Port::kMXP)};

frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, Drivetrain::getHeading()};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Drivetrain::init() { resetGyro(); }

void Drivetrain::resetGyro() { navx->ZeroYaw(); }

units::degree_t Drivetrain::getAngle() { return units::degree_t{-navx->GetAngle() - 90}; }

frc::Pose2d Drivetrain::getOdometryPose() { return m_odometry.GetPose(); }

frc::Rotation2d Drivetrain::getHeading() { return {getAngle()}; }

void Drivetrain::drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, K_MAX_SPEED);

  auto [fl, fr, bl, br] = states;

  MODULES::FRONT_LEFT.setDesiredState(fl);
  MODULES::FRONT_RIGHT.setDesiredState(fr);
  MODULES::BACK_LEFT.setDesiredState(bl);
  MODULES::BACK_RIGHT.setDesiredState(br);
}

void Drivetrain::updateOdometry()
{
  m_odometry.Update(getHeading(),
                    MODULES::FRONT_LEFT.getState(),
                    MODULES::FRONT_RIGHT.getState(),
                    MODULES::BACK_LEFT.getState(),
                    MODULES::BACK_RIGHT.getState());
}