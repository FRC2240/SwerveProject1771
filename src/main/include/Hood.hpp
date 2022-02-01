#pragma once

namespace Hood
{
    constexpr double TOLERANCE = 1;

    enum POSITION {
        BOTTOM       = 0,
        TRAVERSE     = -9,
        SAFE_TO_TURN = -38,
        MIDPOINT     = -26,
        BATTER       = -89
    };
    
    //Public Function Definitions
    void init();

    /// returns true if tolerance is met
    bool goToPosition(Hood::POSITION position, double tolerance = TOLERANCE);

    /// returns true if tolerance is met
    bool visionTrack(double tolerance = TOLERANCE);

    /// used for tuning interpolation tables
    void manualPositionControl(double position);

    void   printAngle();
    double getAngle();
    double getCameraY();
} // namespace Hood
