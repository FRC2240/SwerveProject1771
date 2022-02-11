#include "Hopper.hpp"
#include "ShooterWheel.hpp"

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include <cmath>

using can_adr = int;

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/
namespace INDEXER
{
    constexpr can_adr LIMIT_SWITCH_PORT = 0;
    constexpr can_adr PORT = 10;
    constexpr double SPEED = 0.7;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;
}

namespace TRANSPORT
{
    constexpr can_adr PORT = 3;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

    constexpr double SPEED = 0.4;
    constexpr double SHOOT_SPEED = 1.0; // previous value was 1.0

    constexpr double DISTANCE = 79.0 / 3;
    constexpr double TOLERANCE = 1;

    constexpr double P = 0.3;
    constexpr double I = 0;
    constexpr double D = 0.0001;
}

/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

static rev::CANSparkMax indexer{INDEXER::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax transport{TRANSPORT::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static auto pid_controller = transport.GetPIDController();
static auto encoder = transport.GetEncoder();
static frc::DigitalInput limit_switch{INDEXER::LIMIT_SWITCH_PORT};
static int number_of_balls = 3;
static double target_distance = TRANSPORT::DISTANCE;
static bool is_transporting = false;
static std::atomic<bool> invalid_stop_flag{false};

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

bool Hopper::index(bool const &warn_if_shooting)
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

wpi::array<double, 2> Hopper::getTemps()
{
    return {indexer.GetMotorTemperature(),
            transport.GetMotorTemperature()};
}