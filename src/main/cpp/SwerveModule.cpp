#include "SwerveModule.hpp"

#include <wpi/numbers>
#include <units/angular_velocity.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

inline static constexpr units::meter_t WHEEL_RADIUS = units::inch_t{2};
inline static constexpr int TALON_ENCODER_TICKS_PER_ROTATION = 2048;
inline static constexpr int CANCODER_TICKS_PER_ROTATION = 4096;

inline static constexpr double DRIVER_GEAR_RATIO = 8.16;
inline static constexpr double TURNER_GEAR_RATIO = 12.8;

double constexpr TALON_ENCODER_TICKS_PER_MOTOR_RADIAN =
    TALON_ENCODER_TICKS_PER_ROTATION / (2 * wpi::numbers::pi); // Number of ticks per radian

double constexpr DRIVER_ENCODER_TICKS_PER_WHEEL_RADIAN =
    DRIVER_GEAR_RATIO * TALON_ENCODER_TICKS_PER_MOTOR_RADIAN; // Total amount of ticks per wheel radian

double constexpr HUNDREDMILLISECONDS_TO_1SECOND = 10; // Ticks / 100 milliseconds * 10 = Ticks / 1 second
double constexpr ONESECOND_TO_100MILLISECONDS = .1;   // Ticks / second * .1 = Ticks / 100 milliseconds

inline static constexpr double TALON_ENCODER_DEGREES_TO_TICKS = TALON_ENCODER_TICKS_PER_ROTATION / 360;
inline static constexpr double CANCODER_DEGREES_TO_TICKS = CANCODER_TICKS_PER_ROTATION / 360;

inline static constexpr auto MODULE_MAX_ANGULAR_VELOCITY = wpi::numbers::pi * 1_rad_per_s;           // radians per second
inline static constexpr auto MODULE_MAX_ANGULAR_ACCELERATION = wpi::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

// Set Variables
SwerveModule::SwerveModule(int driver_adr, int turner_adr, int cancoder_adr, frc::Translation2d position)
    : driver{driver_adr},
      turner{turner_adr},
      cancoder{cancoder_adr},
      position{position}
{
    // Configure CANCoder
    CANCoderConfiguration cancoder_config{};
    cancoder_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    cancoder_config.unitString = "deg";
    cancoder_config.sensorDirection = false; // Counter-Clock Wise
    cancoder_config.absoluteSensorRange = AbsoluteSensorRange::Signed_PlusMinus180;
    cancoder.ConfigAllSettings(cancoder_config);
    // cancoder.ConfigSensorDirection()

    // Configure Driver
    TalonFXConfiguration driver_config{};
    driver_config.slot0.kP = .1;
    driver_config.slot0.kI = 0;
    driver_config.slot0.kD = 0;
    driver_config.slot0.kF = 0;
    driver_config.closedloopRamp = .2;
    // driver_config.voltageCompSaturation = 12;
    driver.ConfigAllSettings(driver_config);
    driver.SetNeutralMode(NeutralMode::Brake);

    // Configure Turner
    TalonFXConfiguration turner_config{};
    turner_config.slot0.kP = .1;
    turner_config.slot0.kI = 0;
    turner_config.slot0.kD = 0;
    turner_config.slot0.kF = 0;
    turner_config.neutralDeadband = 0.001771;
    turner_config.peakOutputForward = .5;
    turner_config.peakOutputReverse = -.5;
    turner_config.remoteFilter0.remoteSensorDeviceID = cancoder.GetDeviceNumber();
    turner_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    turner_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    turner_config.closedloopRamp = .000;
    turner.ConfigAllSettings(turner_config);
}

frc::SwerveModuleState SwerveModule::getState()
{
    return {units::meters_per_second_t{(driver.GetSelectedSensorVelocity() * HUNDREDMILLISECONDS_TO_1SECOND / DRIVER_ENCODER_TICKS_PER_WHEEL_RADIAN * WHEEL_RADIUS) / 1_s},
            frc::Rotation2d(getAngle())};
}

units::degree_t SwerveModule::getAngle()
{
    return units::degree_t{cancoder.GetAbsolutePosition()};
}

void SwerveModule::setDesiredState(frc::SwerveModuleState const &desired_state)
{
    frc::Rotation2d const current_rotation = getAngle();

    // Optimize the reference state to avoid spinning further than 90 degrees
    auto const [optimized_speed, optimized_angle] = frc::SwerveModuleState::Optimize(desired_state, current_rotation);

    // Convert speed (m/s) to ticks per 100 milliseconds
    double const desired_driver_velocity_ticks =
        (optimized_speed / WHEEL_RADIUS * DRIVER_ENCODER_TICKS_PER_WHEEL_RADIAN * ONESECOND_TO_100MILLISECONDS).value();

    // Difference between desired angle and current angle
    frc::Rotation2d delta_rotation = optimized_angle - current_rotation;

    // Convert change in angle to change in (turner) ticks and account for gear ratio
    double const delta_ticks = delta_rotation.Degrees().value() * TALON_ENCODER_DEGREES_TO_TICKS * TURNER_GEAR_RATIO;

    // Get the current turner position
    double const current_ticks = turner.GetSelectedSensorPosition();

    // Finally, calculate what the new tick value should be
    double const desired_turner_pos_ticks = current_ticks + delta_ticks;

    driver.Set(TalonFXControlMode::Velocity, desired_driver_velocity_ticks);
    turner.Set(TalonFXControlMode::Position, desired_turner_pos_ticks);
}

void SwerveModule::setTurnerAngle(units::degree_t const& desired_angle)
{
    double const delta_ticks = desired_angle.value() * TALON_ENCODER_DEGREES_TO_TICKS * TURNER_GEAR_RATIO;
    turner.Set(TalonFXControlMode::Position, delta_ticks);
}