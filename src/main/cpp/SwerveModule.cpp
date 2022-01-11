#include "SwerveModule.hpp"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include <units/angular_velocity.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

static constexpr units::meter_t K_WHEEL_RADIUS = units::inch_t{2};
static constexpr int K_ENCODER_TICKS_PER_ROTATION = 2048;
static constexpr int CANCODER_TICKS_PER_ROTATION = 4096;
static constexpr double GEAR_RATIO = 8.16;

double constexpr K_ENCODER_TICKS_PER_MOTOR_RADIAN =
    K_ENCODER_TICKS_PER_ROTATION / (2 * wpi::numbers::pi); // Number of ticks per radian

double constexpr K_ENCODER_TICKS_PER_WHEEL_RADIAN =
    GEAR_RATIO * K_ENCODER_TICKS_PER_MOTOR_RADIAN; // Total amount of ticks per wheel radian

double constexpr HUNDREDMILLISECONDS_TO_1SECOND = 10; // Ticks / 100 milliseconds * 10 = Ticks / 1 second
double constexpr ONESECOND_TO_100MILLISECONDS = .1;   // Ticks / second * .1 = Ticks / 100 milliseconds

static constexpr double K_ENCODER_DEGREES_TO_TICKS = K_ENCODER_TICKS_PER_ROTATION / 360;
static constexpr double CANCODER_DEGREES_TO_TICKS = CANCODER_TICKS_PER_ROTATION / 360;

static constexpr auto K_MODULE_MAX_ANGULAR_VELOCITY = wpi::numbers::pi * 1_rad_per_s;           // radians per second
static constexpr auto K_MODULE_MAX_ANGULAR_ACCELERATION = wpi::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

// Set Variables
SwerveModule::SwerveModule(can_adr drive_motor_adr, can_adr turning_motor_adr, can_adr cancoder_adr, frc::Translation2d wheel_position)
    : driver{drive_motor_adr},
      turner{turning_motor_adr},
      direction_encoder{cancoder_adr},
      wheel_pos(wheel_position)
{
    // Configure CANCoder
    CANCoderConfiguration direction_config{};
    direction_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    direction_config.unitString = "deg";
    direction_config.sensorDirection = false; // Counter-Clock Wise
    direction_config.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    direction_encoder.ConfigAllSettings(direction_config);
    // direction_encoder.ConfigSensorDirection()

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
    turner_config.slot0.kP = 2.5;
    turner_config.slot0.kI = 0;
    turner_config.slot0.kD = 0;
    turner_config.slot0.kF = 0;
    turner_config.neutralDeadband = 0.001771;
    turner_config.peakOutputForward = .5;
    turner_config.peakOutputReverse = -.5;
    turner_config.remoteFilter0.remoteSensorDeviceID = direction_encoder.GetDeviceNumber();
    turner_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    turner_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    turner_config.closedloopRamp = .000;
    turner.ConfigAllSettings(turner_config);
}

frc::SwerveModuleState SwerveModule::getState()
{
    return {units::meters_per_second_t{(driver.GetSelectedSensorVelocity() * HUNDREDMILLISECONDS_TO_1SECOND / K_ENCODER_TICKS_PER_WHEEL_RADIAN * K_WHEEL_RADIUS) / units::second_t(1)},
            frc::Rotation2d(getAngle())};
}

units::degree_t SwerveModule::getAngle()
{
    return units::degree_t{direction_encoder.GetAbsolutePosition()};
}

void SwerveModule::setDesiredState(frc::SwerveModuleState const &desired_state)
{
    frc::Rotation2d const current_rotation = getAngle();

    // Optimize the reference state to avoid spinning further than 90 degrees
    auto const optimized_desired_state = frc::SwerveModuleState::Optimize(desired_state, current_rotation);

    // Convert speed to ticks per 100 milliseconds
    double const desired_driver_velocity_ticks = (optimized_desired_state.speed / K_WHEEL_RADIUS * K_ENCODER_TICKS_PER_WHEEL_RADIAN * ONESECOND_TO_100MILLISECONDS).value();

    // Difference between desired angle and current angle
    frc::Rotation2d delta_rotation = optimized_desired_state.angle - current_rotation;

    // Convert change in angle to change in ticks
    double const delta_ticks = delta_rotation.Degrees().value() * K_ENCODER_DEGREES_TO_TICKS;

    // Convert the CANCoder from it's position reading back to ticks
    double const current_ticks = direction_encoder.GetAbsolutePosition() * CANCODER_DEGREES_TO_TICKS;

    // Finally, calculate what the new tick value should be
    double const desired_turner_pos_ticks = current_ticks + delta_ticks;

    driver.Set(ControlMode::Velocity, desired_driver_velocity_ticks);
    turner.Set(ControlMode::Position, desired_turner_pos_ticks);
}