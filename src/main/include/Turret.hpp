#pragma once

namespace Turret
{
    struct visionState
    {
        bool isTracking;
        bool readyToShoot;
    };

    constexpr double TOLERANCE = 10;


    enum POSITION {
        MAX_LEFT                  = -74,
        FRONT                     = -53,
        SAFE_TO_DEPLOY_HOOD_FRONT = -44,
        ZERO                      = 0,
        BACK                      = 53,
        MAX_RIGHT                 = 74
    };

    void init();

    /// returns true if tolerance is met
    bool goToPosition(POSITION position, double tolerance = 1);

    /// goes to position and then starts tracking, returns true if tolerance (in degrees) is met
    [[depricated]] visionState visionTrack_v1(POSITION initPosition, double tolerance = 10);

    /// goes to position, then determines angle of target and goes to that angle
    visionState visionTrack(POSITION initPosition, double tolerance = TOLERANCE);

    /// used for tuning interpolation tables
    void manualPositionControl(double position);
} // namespace Turret
