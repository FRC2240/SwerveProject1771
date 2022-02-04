#include "PID_CANSparkMax.hpp"
#include "ngr.hpp"

#include <algorithm>

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
PID_CANSparkMax::PID_CANSparkMax(int const &id, MotorType const &motor_type)
    : rev::CANSparkMax(id, motor_type), pid_controller{rev::CANSparkMax::GetPIDController()}, encoder{GetEncoder()}
{
    pid_controller.SetFeedbackDevice(encoder);
}

void PID_CANSparkMax::Set(double value)
{
    if (ngr::valueInRange(encoder.GetPosition(), min_position, max_position))
        rev::CANSparkMax::Set(value);
    else
        rev::CANSparkMax::Set(0);
}

void PID_CANSparkMax::SetVoltage(units::volt_t value)
{
    if (ngr::valueInRange(encoder.GetPosition(), min_position, max_position))
        rev::CANSparkMax::SetVoltage(value);
    else
        rev::CANSparkMax::Set(0);
}

void PID_CANSparkMax::SetOutputRange(double const &min, double const &max)
{
    if (min >= max)
        fmt::print("Invalid Output Range: min: {}, max: \n", min, max);
    pid_controller.SetOutputRange(min, max);
}

void PID_CANSparkMax::SetTarget(double const &position, CANSparkMax::ControlType const &control_type)
{
    pid_controller.SetReference(std::clamp(position, min_position, max_position), control_type);
}

void PID_CANSparkMax::SetPID(double const &P, double const &I, double const &D)
{
    pid_controller.SetP(P);
    pid_controller.SetI(I);
    pid_controller.SetD(D);
}

rev::SparkMaxPIDController PID_CANSparkMax::GetPIDController()
{
    return rev::CANSparkMax::GetPIDController();
}