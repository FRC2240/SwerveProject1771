#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>

#include <ctre/Phoenix.h>

using can_adr = int;




struct SwerveModuleInfo
{
  can_adr const driver_adr, turner_adr, cancoder_adr;
   frc::Translation2d wheel_pos;
};

class SwerveModule
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    SwerveModule(can_adr driver_adr, can_adr turner_adr, can_adr cancoder_adr, frc::Translation2d wheel_position);
    SwerveModule(SwerveModuleInfo const& info);
    
    frc::SwerveModuleState getState();
    units::degree_t getAngle();
    void setDesiredState(const frc::SwerveModuleState &state);

    // Allows SwerveModule to be placed into Kinematics
    constexpr operator frc::Translation2d() const { return wheel_pos; }

    // No copies/moves should be occuring
    SwerveModule(SwerveModule const &) = delete;
    SwerveModule(SwerveModule &&) = delete;

    

private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    TalonFX driver, turner;
    CANCoder direction_encoder;
    frc::Translation2d wheel_pos;
    
};
