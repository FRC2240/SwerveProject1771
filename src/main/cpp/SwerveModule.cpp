#include "SwerveModule.hpp"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include <units/angular_velocity.h>
#include <ctre\Phoenix.h>>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

static constexpr units::meter_t kWheelRadius = units::inch_t{2};
static constexpr int kEncoderResolution = 2048;

// DistancePerPulse(2 * wpi::numbers::pi * kWheelRadius.value() / kEncoderResolution);
static constexpr auto kModuleMaxAngularVelocity = wpi::numbers::pi * 1_rad_per_s;           // radians per second
static constexpr auto kModuleMaxAngularAcceleration = wpi::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

SwerveModule::SwerveModule(can_adr drive_motor_adr, can_adr turning_motor_adr, can_adr cancoder_adr, frc::Translation2d wheel_position)
    : drive_motor{drive_motor_adr},
      turning_motor{turning_motor_adr},
      direction{cancoder_adr},
      wheel_pos(wheel_position)
{
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    direction.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    direction.

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
    // divided by the encoder resolution.
    m_turningEncoder.SetDistancePerPulse(2 * wpi::numbers::pi /
                                         kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.EnableContinuousInput(
        -units::radian_t(wpi::numbers::pi), units::radian_t(wpi::numbers::pi));
}

frc::SwerveModuleState SwerveModule::getState() const
{
    return {units::meters_per_second_t{m_driveEncoder.GetRate()},
            frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

void SwerveModule::setDesiredState(
    const frc::SwerveModuleState &referenceState)
{
    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize(
        referenceState, units::radian_t(m_turningEncoder.Get()));

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(
        m_driveEncoder.GetRate(), state.speed.value());

    const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

    // Calculate the turning motor output from the turning PID controller.
    const auto turnOutput = m_turningPIDController.Calculate(
        units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

    const auto turnFeedforward = m_turnFeedforward.Calculate(
        m_turningPIDController.GetSetpoint().velocity);

    // Set the motor outputs.
    m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
