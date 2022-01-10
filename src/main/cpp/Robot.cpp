#include "Drivetrain.hpp"
#include "Buttons.hpp"

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/filter/SlewRateLimiter.h>

class Robot : public frc::TimedRobot
{
public:
  void AutonomousPeriodic() override
  {
    DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override { DriveWithJoystick(true); }

private:
  Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative)
  {
    // Get the x speed.
    const auto xSpeed = m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(BUTTON::ps5.GetX(), 0.04)) *
                        Drivetrain::kMaxSpeed;
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(BUTTON::ps5.GetY(), 0.04)) *
                        Drivetrain::kMaxSpeed;
    const auto rot = m_rotLimiter.Calculate( //Might need to be inverted in the future
                         frc::ApplyDeadband(BUTTON::ps5.GetZ(), 0.04)) *
                     Drivetrain::kMaxAngularSpeed;
    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
