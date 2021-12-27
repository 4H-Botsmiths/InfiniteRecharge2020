/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <iostream>

#include "Robot.h"

void Robot::RobotInit() {
    frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    frc::CameraServer::GetInstance()->StartAutomaticCapture(1);

    motorNW.RestoreFactoryDefaults();
    motorNE.RestoreFactoryDefaults();
    motorSE.RestoreFactoryDefaults();
    motorSW.RestoreFactoryDefaults();

    driveRocker.ClearAllPCMStickyFaults();
}

void Robot::RobotPeriodic() {
    starDustRobot.RobotPeriodic();
}

void Robot::AutonomousInit() {
    starDustRobot.AutonomousInit();

    //rev shooter wheels, start aligning
    shooter.Set(SHOOTER_SPEED);
    Timer { true, [=]{ limelightAlign(); }, 3.0 };
    limelight.turnLightsOff();

    //move belt and stop shooting
    ballBelt.Set(BALL_BELT_SPEED, 3);
    shooter.Set(0);

    //move off auto line
    drivetrain.drive(0, -0.3, 0, 2);
}

void Robot::AutonomousPeriodic() {
    starDustRobot.AutonomousPeriodic();
}

void Robot::TeleopInit() {
    starDustRobot.TeleopInit();
}

void Robot::TeleopPeriodic() {
    starDustRobot.TeleopPeriodic();

    if (auxController.GetTriggerRightDeadzone() > 0 && intakeArm.isExtended()) {
        ballIntake.Set(BALL_INTAKE_SPEED);

        //lower beam broken and mag not full
        if (!lowerBallDetector.Get() && magazineFullDetector.Get()) {
            ballBelt.Set(BALL_BELT_SPEED);
        }
        else {
            ballBelt.Set(0);
        }
    }
    else {
        ballIntake.Set(0);
    }

    shooter.Set(
        auxController.GetAButton() ? SHOOTER_SPEED : 0
    );

    if (auxController.GetLeftBumper()) {
        ballBelt.Set(-BALL_BELT_SPEED);
    }
    else if (auxController.GetRightBumper()) {
        ballBelt.Set(BALL_BELT_SPEED);
    }
    else {
        ballBelt.Set(0);
    }

    if (driveController.GetTriggerRightDeadzone() > 0) drivetrain.useNormal();
    else drivetrain.useMecanum();

    if (driveController.GetTriggerLeftDeadzone() > 0) {
        limelightAlign();
    }
    else {
        limelight.turnLightsOff();

        int pov=driveController.GetPOV();

        if (usingGyroMode) {
            if (pov==-1) {
                driveAUX.drive(&driveController);
            }
            else {
                driveAUX.driveGyro(
                    pov,
                    2,
                    &driveController
                );
            }
        }
        else {
            drivetrain.drive(&driveController);
        }
    }
}

void Robot::TestPeriodic() {
    starDustRobot.TestPeriodic();
}

void Robot::DisabledInit() {}

void Robot::limelightAlign() {
    limelight.turnLightsOn();

    if (limelight.getTV()) {
        const double targetX=limelight.getTX();

        if (!(-LIMELIGHT_RANGE < targetX && targetX < LIMELIGHT_RANGE)) {
            if (targetX < 0) {
                drivetrain.drive(
                    0,
                    -TURN_THRESHOLD - ( -targetX * LIMELIGHT_TURN_MULT )
                );
            }
            else {
                drivetrain.drive(
                    0,
                    TURN_THRESHOLD + ( targetX * LIMELIGHT_TURN_MULT )
                );
            }
        }
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif