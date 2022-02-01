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
    rev::SparkMaxPIDController pid_controller;

    double min_position = -std::numeric_limits<double>().infinity();
    double max_position = std::numeric_limits<double>().infinity();

    inline static rev::ControlType const default_control_type = rev::ControlType::kPosition;

public:
    rev::SparkMaxRelativeEncoder encoder;
    PID_CANSparkMax(int id, MotorType motor_type);

    // Sets motor to Zero if ouside range
    void Set(double) override;
    void SetVoltage(units::volt_t output) override;

    void           SetOutputRange(double min, double max);
    constexpr void SetPositionRange(double min, double max)
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
    void SetTarget(double target, rev::ControlType = default_control_type);

    void SetP(double P);
    void SetI(double I);
    void SetD(double D);
};
