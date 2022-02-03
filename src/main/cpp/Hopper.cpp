#include "Hopper.hpp"
#include "ShooterWheel.hpp"
#include "ngr.hpp"

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include <cmath>

using can_adr = int;
/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/
namespace INDEXER
{
    const can_adr PORT = 10;
    const double SPEED = 0.7;

    const auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;
}

namespace TRANSPORT
{
    const can_adr PORT = 3;

    const auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

    const double SPEED = 0.4;
    const double SHOOT_SPEED = 1.0; // previous value was 1.0

    const double DISTANCE = 79.0 / 3;
    const double TOLERANCE = 1;

    const double P = 0.3;
    const double I = 0;
    const double D = 0.0001;
}

using can_adr = int;

const can_adr LIMIT_SWITCH = 0;
/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

local rev::CANSparkMax indexer{INDEXER::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
local rev::CANSparkMax transport{TRANSPORT::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
local auto pid_controller = transport.GetPIDController();
local auto encoder = transport.GetEncoder();
local frc::DigitalInput limit_switch{LIMIT_SWITCH};
int number_of_balls = 3;
double target_distance = TRANSPORT::DISTANCE;
bool is_transporting = false;
std::atomic<bool> invalid_stop_flag{false};

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Hopper::init()
{
    // indexer.Set(INDEXER::SPEED);

    indexer.SetIdleMode(INDEXER::IDLE_MODE);
    indexer.SetSmartCurrentLimit(20);

    transport.SetIdleMode(TRANSPORT::IDLE_MODE);
    transport.SetSmartCurrentLimit(40);

    pid_controller.SetP(TRANSPORT::P);
    pid_controller.SetI(TRANSPORT::I);
    pid_controller.SetD(TRANSPORT::D);

    pid_controller.SetFeedbackDevice(encoder);
    pid_controller.SetOutputRange(-TRANSPORT::SPEED, TRANSPORT::SPEED);

    encoder.SetPosition(0);
}

bool Hopper::index(bool warn_if_shooting)
{
    if (invalid_stop_flag)
    {
        if (warn_if_shooting)
            fmt::print("Stop not called after shooting: Indexer Aborting\n");
        return false;
    }

    if (!limit_switch.Get() && number_of_balls < 3 && !is_transporting)
    {
        pid_controller.SetReference(target_distance, rev::CANSparkMax::ControlType::kPosition);
        number_of_balls++;
        is_transporting = true;
    }

    if (is_transporting && encoder.GetPosition() > (target_distance - TRANSPORT::TOLERANCE))
    {
        target_distance += TRANSPORT::DISTANCE;
        is_transporting = false;
    }

    if (limit_switch.Get() && number_of_balls < 4)
        indexer.Set(INDEXER::SPEED);
    else
        indexer.Set(0);
    return true;
}

void Hopper::shoot()
{
    invalid_stop_flag = true;
    indexer.Set(INDEXER::SPEED - 0.3);
    ShooterWheel::setShooting(true);
    transport.Set(TRANSPORT::SHOOT_SPEED);
}

void Hopper::stop()
{
    ShooterWheel::setShooting(false);
    if (invalid_stop_flag)
    {
        invalid_stop_flag = false;
        is_transporting = false;
        number_of_balls = 0;
        target_distance = TRANSPORT::DISTANCE;
        encoder.SetPosition(0);

        transport.Set(0);
    }
}
