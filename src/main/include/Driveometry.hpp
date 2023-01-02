#ifndef DRIVEOMETRY_H_
#define DRIVEOMETRY_H_

/*
Oh, code of sorrow and despair
Proceed with caution, if you dare
For within these lines lies heartache and pain
And the tears that will fall like acid rain

The bugs that lurk, the crashes that come
The endless debugging, it never is done
The late nights, the hair tearing out
The endless frustration, there is no doubt.


To anyone reading this, turn back now.
No wisdom, prosperity nor joy awaits you here, only mistakes.
Abandon hope, all who enter.
*/

/* Drivetrain HPP */
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/array.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <numbers>
#include "SwerveModule.hpp"
#include <frc/smartdashboard/SmartDashboard.h>


namespace Drivetrain
{
    void print_angle();

    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/

    // Absolute max module speed
    constexpr units::meters_per_second_t MODULE_MAX_SPEED = 13.9_fps;

    /*
    Max effective speed considering pi radians/second max angular speed
    ROBOT SAT: 9.535f/s
    */

    // Formula for determing ROBOT_MAX_SPEED is Wheel Max Speed = Robot Max Speed + Omega max speed * distance of module from center
    // Or Robot Max Speed = Max wheel speed - Omega max speed * distance from center
    // Distance of module from center is 1.294ft

    // Max effective linear speed
    constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 9.533_fps; // 5.47_fps;
    constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{std::numbers::pi};

    constexpr units::meters_per_second_t TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{std::numbers::pi};
    constexpr units::meters_per_second_t TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::acceleration::meters_per_second_squared_t TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 0.5_s;
    constexpr units::radians_per_second_t TRAJ_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
    constexpr units::radians_per_second_squared_t TRAJ_MAX_ANGULAR_ACCELERATION{std::numbers::pi};

    constexpr auto ROTATE_P = 2; // Modifier for rotational speed -> (degree * ROTATE_P) / 1sec

    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();

    // Returns values with 0 being front and positive angles going CW
    [[nodiscard]] units::degree_t getAngle();

    frc::Rotation2d getCCWHeading();

    frc::Rotation2d getCWHeading();

    [[nodiscard]] frc::Rotation2d test_heading();

    [[nodiscard]] wpi::array<double, 4> getDriverTemps();

    [[nodiscard]] wpi::array<double, 4> getTurnerTemps();

    [[nodiscard]] frc::ChassisSpeeds getRobotRelativeSpeeds();

    [[nodiscard]] const wpi::array<frc::SwerveModuleState, 4> getModuleStates();

    wpi::array<frc::SwerveModulePosition, 4> get_module_pos();

    // Handles inversing
    void tankDrive(double const &x_speed, double const &y_speed);

    void drive(units::meters_per_second_t const &xSpeed,
               units::meters_per_second_t const &ySpeed,
               units::radians_per_second_t const &rot,
               bool const &fieldRelative);

    void drive(frc::ChassisSpeeds const &speeds);

    void drive(wpi::array<frc::SwerveModuleState, 4> states);

    void stop();

    // For theta, positive is CCW
    void faceDirection(units::meters_per_second_t const &dx,
                       units::meters_per_second_t const &dy,
                       units::degree_t const &theta,
                       bool const &field_relative,
                       double const &rot_p = ROTATE_P,
                       units::degrees_per_second_t const &max_rot_speed = TELEOP_MAX_ANGULAR_SPEED);

    void faceClosest(units::meters_per_second_t const &dx,
                     units::meters_per_second_t const &dy,
                     bool const &field_relative,
                     double const &rot_p = ROTATE_P,
                     units::degrees_per_second_t const &max_rot_speed = TELEOP_MAX_ANGULAR_SPEED);

    void tuneTurner(units::degree_t const &desired_angle);

    void manualPercentOutput(double const &percent_output);

    void manualVelocity(double const &velocity_ticks_per_100ms);
}

// Odometry hpp


#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/array.h>
#include "ngr.hpp"


namespace Odometry
{
/*    const wpi::array<frc::SwerveModulePosition, 4> module_position{
    frc::Translation2d{0_in, 0_deg},
    frc::Translation2d{0_deg},
    frc::Translation2d{0_deg},
    frc::Translation2d{0_deg},
    }
*/

    const frc::SwerveDriveKinematics<4> kinematics{frc::Translation2d{9.125_in, -9.125_in},
                                         frc::Translation2d{9.125_in, 9.125_in},
                                         frc::Translation2d{-9.125_in, -9.125_in},
                                         frc::Translation2d{-9.125_in, 9.125_in}};

    void putField2d();

    [[nodiscard]] frc::Pose2d getPose();

    void update();

    void resetPosition(const frc::Pose2d &pose, const frc::Rotation2d &gyroAngle);

    [[nodiscard]] frc::FieldObject2d *getField2dObject(std::string_view name);

    [[nodiscard]] frc::ChassisSpeeds const getFieldRelativeSpeeds();
}

#endif // DRIVEOMETRY_H_
