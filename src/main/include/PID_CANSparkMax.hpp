#pragma once

/**
 * This is intended for controlling brushless motors
 * where the mechanism they're controlling has a
 * limited range of motion
 *
 * The intention of this is to make it impossible to
 * tell it to go to a position outside of a range
 */

#include <numeric>
#include <rev\CANSparkMax.h>
#include <type_traits>

/**
 * This class exists to add output range protection.
 * This is helpfull because it makes it imposible to accidentally
 * tell a motor to drive to a position that could damage a mechanism.
 *
 * This class also combines the several objects that need to be initialized
 * for CAN_SparkMax PID control into one object
 *
 * Note: This class will not help if encoder positions are lost
 */

class PID_CANSparkMax : public rev::CANSparkMax
{
private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/
    rev::SparkMaxPIDController pid_controller;

    double min_position = -std::numeric_limits<double>().infinity();
    double max_position = std::numeric_limits<double>().infinity();

    static constexpr ControlType default_control_type = ControlType::kPosition;

public:
    /******************************************************************/
    /*                   Public Variable Definitions                   */
    /******************************************************************/
    rev::SparkMaxRelativeEncoder encoder;

    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    PID_CANSparkMax(int const &id, MotorType const &motor_type);

    // Sets motor to Zero if ouside range
    void Set(double) override;
    void SetVoltage(units::volt_t output) override;

    void SetOutputRange(double const &min, double const &max);
    constexpr void SetPositionRange(double const &min, double const &max)
    {
        min_position = min;
        max_position = max;
    }

    /**
     * Provides direct access to the internal PID controller.
     *
     * Warning: This completely bypasses the range protection
     * built into this class
     */
    [[deprecated]] rev::SparkMaxPIDController GetPIDController();

    /**
     * If the target is within the position range, it will go to the target
     * If not, it will go as close as it can to the target without exceding the range
     */
    void SetTarget(double const &target, ControlType const& control_type = default_control_type);

    void SetPID(double const &P, double const &I, double const &D);
};