#include "SwerveModule.hpp"
#include "ngr.hpp"

#include <wpi/numbers>
#include <units/angular_velocity.h>

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

local constexpr units::meter_t WHEEL_RADIUS = units::inch_t{2};
local constexpr int MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS = 2048;
local constexpr int CANCODER_TICKS_PER_ROTATION = 4096;

local constexpr double DRIVER_GEAR_RATIO = 8.16;
local constexpr double TURNER_GEAR_RATIO = 12.8;

local constexpr double MOTOR_RADIAN_TO_TALON_ENCODER_TICKS =
    MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS / (2 * wpi::numbers::pi); // Number of ticks per radian

local constexpr int DRIVER_WHEEL_ROTATIONS_TO_TICKS =
    MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS * DRIVER_GEAR_RATIO;

local constexpr double WHEEL_RADIAN_TO_DRIVER_ENCODER_TICKS =
     MOTOR_RADIAN_TO_TALON_ENCODER_TICKS * DRIVER_GEAR_RATIO; // Total amount of ticks per wheel radian

local constexpr double HUNDREDMILLISECONDS_TO_1SECOND = 10; // Ticks / 100 milliseconds * 10 = Ticks / 1 second
local constexpr double ONESECOND_TO_100MILLISECONDS = .1;   // Ticks / second * .1 = Ticks / 100 milliseconds

local constexpr double TALON_ENCODER_DEGREES_TO_TICKS = MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS / 360;
local constexpr double CANCODER_DEGREES_TO_TICKS = CANCODER_TICKS_PER_ROTATION / 360;

local constexpr auto MODULE_MAX_ANGULAR_VELOCITY = wpi::numbers::pi * 1_rad_per_s;           // radians per second
local constexpr auto MODULE_MAX_ANGULAR_ACCELERATION = wpi::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

// Set Variables
SwerveModule::SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, double const &magnet_offset)
    : driver{driver_adr},
      turner{turner_adr},
      cancoder{cancoder_adr},
      magnet_offset{magnet_offset}
{
    // Config done in init() to avoid issues with configuring before connection with Talons/CANCoder is established
}

void SwerveModule::init()
{
    // Configure CANCoder
    CANCoderConfiguration cancoder_config{};
    cancoder_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    cancoder_config.unitString = "deg";
    cancoder_config.sensorDirection = false; // Counter-Clock Wise
    cancoder_config.absoluteSensorRange = AbsoluteSensorRange::Signed_PlusMinus180;
    cancoder_config.magnetOffsetDegrees = magnet_offset;
    cancoder.ConfigAllSettings(cancoder_config);
    // cancoder.ConfigSensorDirection()

    // Configure Driver
    TalonFXConfiguration driver_config{};
    driver_config.slot0.kP = 0.1;
    driver_config.slot0.kI = 0;
    driver_config.slot0.kD = 0;
    driver_config.slot0.kF = 0;
    driver_config.closedloopRamp = .1;
    // driver_config.voltageCompSaturation = 12;
    driver.ConfigAllSettings(driver_config);
    driver.SetNeutralMode(NeutralMode::Brake);

    // Configure Turner
    TalonFXConfiguration turner_config{};
    turner_config.slot0.kP = 0.5;
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
    return {units::meters_per_second_t{(driver.GetSelectedSensorVelocity() * HUNDREDMILLISECONDS_TO_1SECOND / WHEEL_RADIAN_TO_DRIVER_ENCODER_TICKS * WHEEL_RADIUS) / 1_s},
            frc::Rotation2d(-getAngle())};
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
        (optimized_speed / WHEEL_RADIUS * DRIVER_WHEEL_ROTATIONS_TO_TICKS * ONESECOND_TO_100MILLISECONDS).value();

    // Difference between desired angle and current angle
    frc::Rotation2d delta_rotation = optimized_angle - current_rotation;

    // Convert change in angle to change in (cancoder) ticks
    double const delta_ticks = delta_rotation.Degrees().value() * CANCODER_DEGREES_TO_TICKS;

    // Get the current cancoder position
    double const current_ticks = turner.GetSelectedSensorPosition();
    // Can (theoretically) be replaced with current_ticks = current_rotation * CANCODER_DEGREES_TO_TICKS;

    // Finally, calculate what the new tick value should be
    double const desired_turner_pos_ticks = current_ticks + delta_ticks;

    /*
    fmt::print("desired_speed: {}, desired_rotation {}\n, current_rotation: {}, optimized_speed: {}, optimized_angle: {},\ndelta_rotation: {}, delta_ticks: {}, current_ticks: {},\ndesired_driver: {}, desired_turner: {}\n",
                desired_state.speed.value(), desired_state.angle.Degrees().value(),
                optimized_speed.value(), optimized_angle.Degrees().value(),
                delta_rotation.Degrees().value(), delta_ticks, current_ticks,
                desired_driver_velocity_ticks, desired_turner_pos_ticks);
    */

    driver.Set(TalonFXControlMode::Velocity, desired_driver_velocity_ticks);
    turner.Set(TalonFXControlMode::Position, desired_turner_pos_ticks);
}

void SwerveModule::setTurnerAngle(units::degree_t const &desired_angle)
{

    // double const delta_ticks = desired_angle.value() * TALON_ENCODER_DEGREES_TO_TICKS * TURNER_GEAR_RATIO;
    // turner.Set(TalonFXControlMode::Position, delta_ticks);

    frc::Rotation2d const current_rotation = getAngle();

    // Optimize the reference state to avoid spinning further than 90 degrees
    auto const optimized = frc::SwerveModuleState::Optimize(frc::SwerveModuleState{0_mps, frc::Rotation2d{desired_angle}}, frc::Rotation2d{getAngle()});

    // Difference between desired angle and current angle
    frc::Rotation2d delta_rotation = optimized.angle - current_rotation;

    // Convert change in angle to change in (cancoder) ticks
    double const delta_ticks = delta_rotation.Degrees().value() * CANCODER_DEGREES_TO_TICKS;

    // Get the current cancoder position
    double const current_ticks = turner.GetSelectedSensorPosition();
    // Or current_ticks = current_rotation * CANCODER_DEGREES_TO_TICKS;

    // Finally, calculate what the new tick value should be
    double const desired_turner_pos_ticks = current_ticks + delta_ticks;

    driver.Set(TalonFXControlMode::Velocity, 0);
    turner.Set(TalonFXControlMode::Position, desired_turner_pos_ticks);
}