#include "Turret.hpp"
//#include "PhotonVision.hpp"
#include "LimeLight.hpp"
#include "ngr.hpp"
#include "PID_CANSparkMax.hpp"

#include <wpi/numbers>
#include <cmath>


/******************************************************************/
/*                             Constants                          */
/******************************************************************/
constexpr double CAMERA_X_OFFSET = 3.75; //4.2517710;

constexpr int PORT = 6;

constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

//TOLERANCE is in .hpp file

constexpr double TICKS_PER_REVOLUTION = 212; // replace me with correct, number. this should be close if not exact
constexpr double TICKS_PER_RADIAN     = 21;  //TICKS_PER_REVOLUTION / (2 * pi);

constexpr double TRAVERSE_SPEED = .7;

constexpr double P = 0.1;
constexpr double I = 0.0;
constexpr double D = 0.0;
/******************************************************************/
/*                          Non-constant Vars                     */
/******************************************************************/
extern LimeLight camera; //camera from Robot.cpp

inline static PID_CANSparkMax  turretTurnyTurny { PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static Turret::POSITION position = Turret::POSITION::ZERO;
inline static bool             tracking = false;
/******************************************************************/
/*                      Non Static Functions                      */
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
    if(pos != position)
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
    if(! tracking) // move to initPosition
    {
        tracking = goToPosition(initPosition);
        return { false, false };
    }
    //
    // photonlib::PhotonPipelineResult result = camera.GetLatestResult();

    if(camera.hasTarget())
    {
        // auto const target = result.GetBestTarget();
        double const xOffsetDeg = camera.getX() + CAMERA_X_OFFSET;
        double const xOffsetRad = (xOffsetDeg) * wpi::numbers::pi / 180;
        double const xOffset    = xOffsetRad * TICKS_PER_RADIAN;

        double const xPosition = turretTurnyTurny.encoder.GetPosition();
        double const xTarget   = xPosition + xOffset;

        static double prevOffsetDeg = 0;
        if(prevOffsetDeg == xOffsetDeg) // prevents reusing outdated data
            return { true, fabs(xOffsetDeg) < tolerance };
        prevOffsetDeg = xOffsetDeg;

        turretTurnyTurny.SetTarget(xTarget);

        return { true, fabs(xOffsetDeg) < tolerance };
    }

    // return { true, true };
    return { false, false };
}

void Turret::manualPositionControl(double pos)
{
    turretTurnyTurny.SetTarget(scaleOutput(
                                   -1,
                                   1,
                                   POSITION::MAX_LEFT,
                                   POSITION::MAX_RIGHT,
                                   std::clamp(pos, -1.0, 1.0)),
                               rev::ControlType::kPosition);
}
