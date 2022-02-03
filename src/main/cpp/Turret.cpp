#include "Turret.hpp"
//#include "PhotonVision.hpp"
#include "LimeLight.hpp"
#include "ngr.hpp"
#include "PID_CANSparkMax.hpp"

#include <wpi/numbers>
#include <units/angle.h>

#include <cmath>

/******************************************************************/
/*                             Constants                          */
/******************************************************************/

constexpr units::degree_t CAMERA_X_OFFSET{3.75}; // 4.2517710;

constexpr int PORT = 6;

constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

//TOLERANCE in .hpp

constexpr double TICKS_PER_REVOLUTION = 212;
constexpr double TICKS_PER_RADIAN = TICKS_PER_REVOLUTION / (2 * wpi::numbers::pi);
constexpr double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360;

constexpr double TRAVERSE_SPEED = .7;

constexpr double P = 0.1;
constexpr double I = 0.0;
constexpr double D = 0.0;

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

extern LimeLight camera; // camera from Robot.cpp

local PID_CANSparkMax turretTurnyTurny{PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
local Turret::POSITION position = Turret::POSITION::ZERO;
local bool tracking = false;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Turret::init()
{
    turretTurnyTurny.RestoreFactoryDefaults();

    turretTurnyTurny.SetIdleMode(IDLE_MODE);

    turretTurnyTurny.SetSmartCurrentLimit(20);

    turretTurnyTurny.SetP(P);
    turretTurnyTurny.SetI(I);
    turretTurnyTurny.SetD(D);
    turretTurnyTurny.SetOutputRange(-TRAVERSE_SPEED, TRAVERSE_SPEED);
    turretTurnyTurny.SetPositionRange(Turret::POSITION::MAX_LEFT, Turret::POSITION::MAX_RIGHT);
    turretTurnyTurny.SetTarget(Turret::POSITION::ZERO);
}

bool Turret::goToPosition(Turret::POSITION pos, double tolerance)
{
    if (pos != position)
    {
        turretTurnyTurny.SetTarget(pos);
        position = pos;
    }

    tracking = false; // Reset for Turret::visionTrack(...)

    return std::fabs(turretTurnyTurny.encoder.GetPosition() - pos) < tolerance;
}

// Turret::visionState Turret::visionTrack_v1(Turret::POSITION initPosition, double tolerance)
// {
//     if(! tracking) // move to initPosition
//     {
//         tracking = goToPosition(initPosition);
//         return { false, false };
//     }

//     if(camera.hasTarget())
//     {
//         double const xOffset = camera.getX() + CAMERA_X_OFFSET;
//         double const output  = xOffset / 35;
//         turretTurnyTurny.Set(output);
//         return { true, fabs(xOffset) < tolerance };
//     }
//     turretTurnyTurny.Set(0);
//     return { false, false };
// }

Turret::visionState Turret::visionTrack(Turret::POSITION initPosition, double tolerance)
{
    if (!tracking) // move to initPosition
    {
        tracking = goToPosition(initPosition);
        return {false, false};
    }
    //
    // photonlib::PhotonPipelineResult result = camera.GetLatestResult();

    if (camera.hasTarget())
    {
        // auto const target = result.GetBestTarget();
        auto const x_offset_deg = units::degree_t{camera.getX()} + CAMERA_X_OFFSET;
        double const x_offset_ticks = x_offset_deg.value() * TICKS_PER_DEGREE;

        double const x_pos = turretTurnyTurny.encoder.GetPosition();
        double const x_target = x_pos + x_offset_ticks;

        static auto prev_offset_deg = 0_deg;
        if (prev_offset_deg == x_offset_deg) // prevents reusing outdated data
            return {true, fabs(x_offset_deg.value()) < tolerance};
        prev_offset_deg = x_offset_deg;

        turretTurnyTurny.SetTarget(x_target);

        return {true, fabs(x_offset_deg.value()) < tolerance};
    }

    return {false, false};
}

void Turret::manualPositionControl(double pos)
{
    turretTurnyTurny.SetTarget(scaleOutput(
                                   -1,
                                   1,
                                   POSITION::MAX_LEFT,
                                   POSITION::MAX_RIGHT,
                                   std::clamp(pos, -1.0, 1.0)),
                               rev::CANSparkMax::ControlType::kPosition);
}
