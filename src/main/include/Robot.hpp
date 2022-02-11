#pragma once

#include <frc/TimedRobot.h>
#include "Turret.hpp"

class Robot : public frc::TimedRobot
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    Robot();

    void RobotInit() override;
    void RobotPeriodic() override;
    
    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    // void DisabledInit() override;
    // void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

private:
    /******************************************************************/
    /*                 Private Function Declarations                  */
    /******************************************************************/

    void tunePID();
    void tankDrive();
    void swerveDrive(bool const &field_relative);
    void buttonManager();
    bool shooterTempUpdate();
    bool aim(Turret::POSITION direction);
};