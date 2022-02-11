#pragma once

namespace Hood
{
    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/
    constexpr double TOLERANCE = 1;

    enum POSITION
    {
        BOTTOM = 0,
        TRAVERSE = -9,
        SAFE_TO_TURN = -38,
        MIDPOINT = -26,
        BATTER = -89
    };

    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();

    /// returns true if tolerance is met
    bool goToPosition(Hood::POSITION const &target_position, double const &tolerance = TOLERANCE);

    /// returns true if tolerance is met
    bool visionTrack(double const &tolerance = TOLERANCE);

    /// used for tuning interpolation tables
    void manualPositionControl(double const &target_position);

    void printAngle();
    [[nodiscard]] double getAngle();
    [[nodiscard]] double getCameraY();

    [[nodiscard]] double getTemp();
}
