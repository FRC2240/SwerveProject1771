#include "ShooterWheel.hpp"

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

using can_adr = int;
/******************************************************************/
/*                             Constants                          */
/******************************************************************/
constexpr can_adr PORT_1       = 18;
constexpr auto    IDLE_MODE    = rev::CANSparkMax::IdleMode::kCoast;
constexpr double  SHOOTING_RPM = 6700; // previous value was 6750, then 6100
constexpr double  IDLE_RPM     = 6500;
/******************************************************************/
/*                          Non-constant Vars                     */
/******************************************************************/
inline static rev::CANSparkMax shooter_1 { PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static rev::SparkMaxRelativeEncoder  shooter_encoder = shooter_1.GetEncoder();
inline static bool             run_at_max_speed;
/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/
void ShooterWheel::init()
{
    // shooter_1.RestoreFactoryDefaults();
    shooter_1.SetIdleMode(IDLE_MODE);
}

void ShooterWheel::bangbang() //original code with commented code removed
{
    shooter_1.SetOpenLoopRampRate(0);


    if((abs(shooter_encoder.GetVelocity()) < 2000))
        shooter_1.Set(-.5);
    else if(run_at_max_speed && abs(shooter_encoder.GetVelocity()) < SHOOTING_RPM)
        shooter_1.Set(-1),
            printf("1\n");
    else if(abs(shooter_encoder.GetVelocity()) < IDLE_RPM)
        shooter_1.Set(-1),
            printf("1\n");
    else
        shooter_1.Set(0), printf("0\n");
}

double ShooterWheel::getSpeed()
{
    return shooter_encoder.GetVelocity();
}

void ShooterWheel::stop()
{
    shooter_1.Set(0);
}

double ShooterWheel::getTemp()
{
    return shooter_1.GetMotorTemperature();
}

// True makes bangbang use SHOOTING_RPM, false uses IDLE_RPM
void ShooterWheel::setShooting(bool input)
{
    run_at_max_speed = input;
}
