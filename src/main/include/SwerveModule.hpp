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

    SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, frc::Translation2d const &position, double const &magnet_offset);

    void init();

    frc::SwerveModuleState getState();

    units::degree_t getAngle();

    void setDesiredState(const frc::SwerveModuleState &state);

    void setTurnerAngle(units::degree_t const &desired_angle);

    // Allows SwerveModule to be placed into Kinematics
    constexpr operator frc::Translation2d() const { return position; }

    // No copies/moves should be occuring
    SwerveModule(SwerveModule const &) = delete;
    SwerveModule(SwerveModule &&) = delete;

private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    TalonFX driver, turner;
    CANCoder cancoder;
    frc::Translation2d const position;
    double const magnet_offset;
};
