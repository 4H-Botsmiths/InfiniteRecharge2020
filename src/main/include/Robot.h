/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>

#include "StarDust/sensor/vision/limelight/Limelight.hpp"
#include "StarDust/pneumatics/DoubleSolenoid.hpp"
#include "StarDust/sensor/motion/AHRS_Gyro.hpp"
#include "StarDust/control/XboxController.hpp"
#include "StarDust/core/StarDustRobot.hpp"
#include "StarDust/file/ConfigParser.hpp"
#include "StarDust/drive/DriveSpider.hpp"
#include "StarDust/file/ParserParam.hpp"
#include "StarDust/motor/MotorGroup.hpp"
#include "StarDust/drive/DriveAUX.hpp"
#include "StarDust/motor/Motor.hpp"

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
	void DisabledInit() override;

	void limelightAlign();

private:
	Limelight limelight;
	double LIMELIGHT_RANGE=2;
	double LIMELIGHT_TURN_MULT=0.01;

	DoubleSolenoid driveRocker { 0, 1, false };

	DoubleSolenoid intakeArm { 4, 5, true };

	rev::CANSparkMax motorNW { 5, rev::CANSparkMax::MotorType::kBrushless };
	rev::CANSparkMax motorNE { 2, rev::CANSparkMax::MotorType::kBrushless };
	rev::CANSparkMax motorSE { 3, rev::CANSparkMax::MotorType::kBrushless };
	rev::CANSparkMax motorSW { 4, rev::CANSparkMax::MotorType::kBrushless };

	AHRS_Gyro gyro;

	DriveSpider drivetrain {
		&motorNW,
		&motorNE,
		&motorSE,
		&motorSW,
		&driveRocker
	};

	double TURN_THRESHOLD=0.05;
	DriveAUX driveAUX {
		&drivetrain,
		&gyro,
		TURN_THRESHOLD
	};

	Motor shooterRight { 1, true };
	Motor shooterLeft { 0, true };
	MotorGroup shooter {{
		&shooterLeft,
		&shooterRight
	}};
	double SHOOTER_SPEED=0.70;

	Motor ballBeltLower { 2, true };
	Motor ballBeltUpper { 3, 0.75, true };

	MotorGroup ballBelt {{
		&ballBeltLower,
		&ballBeltUpper
	}};
	double BALL_BELT_SPEED=0.85;

	Motor ballIntake { 4, true };
	double BALL_INTAKE_SPEED=0.5;

	XboxController driveController { 0, 0.15, 0.15, {
		{ XboxController::on::BackButtonPressed, [=]{
			usingGyroMode=!usingGyroMode;
		}}
	}};

	XboxController auxController { 1, 0.15, 0.15, {
		{ XboxController::on::BButtonPressed, [=]{
			intakeArm.Invert();
		}}
	}};

	ConfigParser config {{
		new ParserParam { "TURN_THRESHOLD", &TURN_THRESHOLD },
		new ParserParam { "SHOOTER_SPEED", &SHOOTER_SPEED },
		new ParserParam { "BALL_BELT_SPEED", &BALL_BELT_SPEED },
		new ParserParam { "BALL_INTAKE_SPEED", &BALL_INTAKE_SPEED },
		new ParserParam { "LIMELIGHT_RANGE", &LIMELIGHT_RANGE },
		new ParserParam { "LIMELIGHT_TURN_MULT", &LIMELIGHT_TURN_MULT }
	}};

	StarDustRobot starDustRobot {{
		&limelight,
		&drivetrain,
		&gyro,
		&driveAUX,
		&driveController,
		&auxController,
		&driveRocker,
		&intakeArm,
		&config
	}};

	frc::DigitalInput magazineFullDetector { 0 };
	frc::DigitalInput lowerBallDetector { 1 };

	bool usingGyroMode=true;
};