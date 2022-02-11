#include "Intake.hpp"

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

using can_adr = int;
/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/
constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

constexpr double IN_SPEED = -1;
constexpr double OUT_SPEED = 1;

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/
static frc::Solenoid intake_air{frc::PneumaticsModuleType::CTREPCM, 1};

static bool deployed = false;

static rev::CANSparkMax wheels{22, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Intake::init()
{
    wheels.SetIdleMode(IDLE_MODE);
    wheels.SetSmartCurrentLimit(20);
}

void Intake::drive(Intake::DIRECTION const &mode)
{
    switch (mode)
    {
    case Intake::DIRECTION::IN:
        wheels.Set(IN_SPEED);
        break;
    case Intake::DIRECTION::OFF:
        wheels.Set(0);
        break;
    case Intake::DIRECTION::OUT:
        wheels.Set(OUT_SPEED);
        break;
    default:
        fmt::print("Invalad Intake Direction\n");
    }
}

void Intake::deploy(bool const &val)
{
    intake_air.Set(val);
    deployed = val;
}

bool Intake::isIntakeDown()
{
    return deployed;
}

double Intake::getWheelTemp()
{
    return wheels.GetMotorTemperature();
}
