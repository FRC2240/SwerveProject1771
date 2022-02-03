#pragma once

namespace Hopper
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();
    // index does not override shoot
    // returns whether or not it's indexing
    bool index(bool warn_if_shooting = true);
    void shoot(); // must call Hopper::stop() to stop shooting
    void stop();
}
