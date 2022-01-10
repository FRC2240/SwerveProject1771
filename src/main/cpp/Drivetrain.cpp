#include "Drivetrain.hpp"

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/
namespace Modules
{
  SwerveModule FRONT_LEFT{40, 41, 12, {11_in, 11_in}};
  SwerveModule FRONT_RIGHT{30, 31, 11, {11_in, -11_in}};
  SwerveModule BACK_LEFT{50, 51, 13, {-11_in, 11_in}};
  SwerveModule BACK_RIGHT{60, 61, 14, {-11_in, -11_in}};
}

frc::AnalogGyro m_gyro{0};

frc::SwerveDriveKinematics<4> m_kinematics{
    Modules::FRONT_LEFT,
    Modules::FRONT_RIGHT,
    Modules::BACK_LEFT,
    Modules::BACK_RIGHT};

frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative)
{
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  Modules::FRONT_LEFT.setDesiredState(fl);
  Modules::FRONT_RIGHT.setDesiredState(fr);
  Modules::BACK_LEFT.setDesiredState(bl);
  Modules::BACK_RIGHT.setDesiredState(br);
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(m_gyro.GetRotation2d(), Modules::FRONT_LEFT.getState(),
                    Modules::FRONT_RIGHT.getState(), Modules::BACK_LEFT.getState(),
                    Modules::BACK_RIGHT.getState());
}
