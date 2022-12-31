#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP
#include "Odometry.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

frc::Field2d field2d;

// The numbers in pose2d have no bearing in reality and are stolen from docs.
// YOLO
    static frc::SwerveDriveOdometry<4> odometry{
        Odometry::kinematics,
        Drivetrain::getCCWHeading(),
        Drivetrain::get_module_pos(),
        frc::Pose2d{5_m, 13.5_m, 0_rad}
        };

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Odometry::putField2d()
{
    frc::SmartDashboard::PutData("Odometry Field", &field2d);
}

void Odometry::update()
{
    frc::Pose2d pose = odometry.Update(Drivetrain::getCCWHeading(),
                                       Drivetrain::get_module_pos());
    if constexpr (debugging)
        frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::Pose2d Odometry::getPose() { return odometry.GetPose(); }

frc::ChassisSpeeds const Odometry::getFieldRelativeSpeeds()
{
    // Init for first time
    static frc::Timer speed_timer;
    speed_timer.Start();
    static frc::Pose2d previous_pose{};

    frc::Pose2d const current_pose = odometry.GetPose();

    frc::Pose2d const delta_pose = current_pose.RelativeTo(previous_pose);

    auto const time_elapsed = speed_timer.Get();
    units::meters_per_second_t const X = delta_pose.X() / time_elapsed;

    units::meters_per_second_t const Y = delta_pose.Y() / time_elapsed;

    units::degrees_per_second_t const rot{delta_pose.Rotation().Degrees() / time_elapsed};

    previous_pose = odometry.GetPose(); // Set the previous_pose for the next time this loop is run

    speed_timer.Reset(); // Time how long until next call

    return frc::ChassisSpeeds{X, Y, rot};
}

void Odometry::resetPosition(const frc::Pose2d &pose, const frc::Rotation2d &gyro_angle)
{
    odometry.ResetPosition(gyro_angle, Drivetrain::get_module_pos(), pose);
}

frc::FieldObject2d *Odometry::getField2dObject(std::string_view name)
{
    return field2d.GetObject(name);
}
#endif
