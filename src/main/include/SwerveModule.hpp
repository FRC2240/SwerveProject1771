#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>

#include <ctre/Phoenix.h>

class SwerveModule
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, double const &magnet_offset);

    [[nodiscard]] frc::SwerveModuleState getState();

    [[nodiscard]] units::degree_t getAngle();

    void setDesiredState(const frc::SwerveModuleState &state);

    // No copies/moves should be occuring (Talons don't support this)
    SwerveModule(SwerveModule const &) = delete;
    SwerveModule(SwerveModule &&) = delete;

private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    WPI_TalonFX driver, turner;
    WPI_CANCoder cancoder;
    double const magnet_offset;
};
