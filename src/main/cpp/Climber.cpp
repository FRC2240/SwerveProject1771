#include "Climber.hpp"
#include "PID_CANSparkMax.hpp"
#include "Buttons.hpp"

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

constexpr int PORT_1 = 9;
constexpr int PORT_2 = 35;
constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;
constexpr double P = 0.1771;
constexpr double I = 0.0;
constexpr double D = 0.0;
constexpr double MAX_OUTPUT = 1;

// Climber Position Constants are held in Climber.hpp

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static PID_CANSparkMax climber_1{PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static PID_CANSparkMax climber_2{PORT_2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Climber::init()
{
    climber_1.RestoreFactoryDefaults();
    climber_2.RestoreFactoryDefaults();

    climber_1.SetIdleMode(IDLE_MODE);
    climber_2.SetIdleMode(IDLE_MODE);

    climber_1.SetPID(P, I, D);
    climber_1.SetOutputRange(-MAX_OUTPUT, MAX_OUTPUT);
    climber_1.SetPositionRange(Climber::POSITION::ZERO, Climber::POSITION::UP);
    climber_1.SetTarget(Climber::POSITION::ZERO);

    climber_2.SetPID(P, I, D);

    climber_2.SetOutputRange(-MAX_OUTPUT, MAX_OUTPUT);
    climber_2.SetPositionRange(-Climber::POSITION::UP, Climber::POSITION::ZERO);
    climber_2.SetTarget(Climber::POSITION::ZERO);
}

void Climber::set(Climber::POSITION position)
{
    climber_1.SetTarget(position, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    climber_2.SetTarget(-position, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::joystickControl(double val)
{
    climber_1.Set(val);
    climber_2.Set(-val);
    printStatus();
}

void Climber::printStatus()
{
    fmt::print("Climber 1: {}\n", climber_1.encoder.GetPosition());
    fmt::print("Climber 2: {}\n", climber_2.encoder.GetPosition());
}

void Climber::buttonManager()
{
    static bool hasBeenPressed = false;
    if (BUTTON::CLIMBER::RAISE && BUTTON::oStick.GetThrottle() < 0)
    {
        hasBeenPressed = true;
        set(Climber::POSITION::UP);
    }
    else if (hasBeenPressed)
        set(Climber::POSITION::DOWN);
}

wpi::array<double, 2> Climber::getTemps()
{
    return {climber_1.GetMotorTemperature(),
            climber_2.GetMotorTemperature()};
}