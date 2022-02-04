#include "PID_CANSparkMax.hpp"

#include <algorithm>

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
PID_CANSparkMax::PID_CANSparkMax(int id, MotorType motor_type)
    : rev::CANSparkMax(id, motor_type), pid_controller{rev::CANSparkMax::GetPIDController()}, encoder{GetEncoder()}
{
    pid_controller.SetFeedbackDevice(encoder);
}

void PID_CANSparkMax::Set(double value)
{
    auto current_position = encoder.GetPosition();
    if (current_position > min_position && current_position < max_position)
        rev::CANSparkMax::Set(value);
    else
        rev::CANSparkMax::Set(0);
}

void PID_CANSparkMax::SetVoltage(units::volt_t value)
{
    auto current_position = encoder.GetPosition();
    if (current_position > min_position && current_position < max_position)
        rev::CANSparkMax::SetVoltage(value);
    else
        rev::CANSparkMax::Set(0);
}

void PID_CANSparkMax::SetOutputRange(double min, double max)
{
    if (min >= max)
        fmt::print("Invalid Output Range: min: {}, max: \n", min, max);
    pid_controller.SetOutputRange(min, max);
}

void PID_CANSparkMax::SetTarget(double position, CANSparkMax::ControlType control_type)
{
    pid_controller.SetReference(std::clamp(position, min_position, max_position), control_type);
}

void PID_CANSparkMax::SetP(double P)
{
    pid_controller.SetP(P);
}

void PID_CANSparkMax::SetI(double I)
{
    pid_controller.SetI(I);
}

void PID_CANSparkMax::SetD(double D)
{
    pid_controller.SetD(D);
}

rev::SparkMaxPIDController PID_CANSparkMax::GetPIDController()
{
    return rev::CANSparkMax::GetPIDController();
}