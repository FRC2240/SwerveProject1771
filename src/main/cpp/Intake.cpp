#include "Intake.hpp"
#include "ngr.hpp"

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

using can_adr = int;
/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/
const auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

const double IN_SPEED = -1;
const double OUT_SPEED = 1;

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/
local frc::Solenoid intake_air{frc::PneumaticsModuleType::CTREPCM, 1};

bool intake_deployed = false;

local rev::CANSparkMax wheels{22, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Intake::init()
{
    wheels.SetIdleMode(IDLE_MODE);
    wheels.SetSmartCurrentLimit(20);
}

void Intake::drive(Intake::DIRECTION mode)
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

void Intake::deploy(bool val)
{
    intake_air.Set(val);
    intake_deployed = val;
}

bool Intake::isIntakeDown()
{
    return intake_deployed;
}
