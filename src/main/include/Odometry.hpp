#ifndef ODOMETRY_H
#define ODOMETRY_H
#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/array.h>
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
#endif
