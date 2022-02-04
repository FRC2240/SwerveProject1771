#include "ShooterWheel.hpp"
#include "ngr.hpp"

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

using can_adr = int;

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

constexpr can_adr PORT = 18;
constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;
constexpr double SHOOTING_RPM = 6700; // previous value was 6750, then 6100
constexpr double IDLE_RPM = 6500;

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static rev::CANSparkMax shooter{PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static auto shooter_encoder = shooter.GetEncoder();
static bool run_at_max_speed;

/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

void ShooterWheel::init()
{
    // shooter.RestoreFactoryDefaults();
    shooter.SetIdleMode(IDLE_MODE);
}

void ShooterWheel::bangbang() // original code with commented code removed
{
    shooter.SetOpenLoopRampRate(0);

    if ((abs(shooter_encoder.GetVelocity()) < 2000))
        shooter.Set(-.5);
    else if (run_at_max_speed && abs(shooter_encoder.GetVelocity()) < SHOOTING_RPM)
        shooter.Set(-1),
            fmt::print("1\n");
    else if (abs(shooter_encoder.GetVelocity()) < IDLE_RPM)
        shooter.Set(-1),
            fmt::print("1\n");
    else
        shooter.Set(0), fmt::print("0\n");
}

double ShooterWheel::getSpeed()
{
    return shooter_encoder.GetVelocity();
}

void ShooterWheel::stop()
{
    shooter.Set(0);
}

double ShooterWheel::getTemp()
{
    return shooter.GetMotorTemperature();
}

// True makes bangbang use SHOOTING_RPM, false uses IDLE_RPM
void ShooterWheel::setShooting(bool input)
{
    run_at_max_speed = input;
}
