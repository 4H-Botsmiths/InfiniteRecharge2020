#pragma once

#include <frc/drive/MecanumDrive.h>
#include <frc/SpeedController.h>

#include "StarDust/util/Timer.hpp"
#include "DriveBase.hpp"

class DriveMecanum : public DriveBase {
public:
    DriveMecanum(frc::SpeedController* motor_0, frc::SpeedController* motor_1, frc::SpeedController* motor_2, frc::SpeedController* motor_3) :
        DriveBase()
    {
        //make a new mecanum drive reference
        mecanum=new frc::MecanumDrive(*motor_0, *motor_3, *motor_1, *motor_2);
    }

    DriveMecanum(frc::SpeedController* motor_0, frc::SpeedController* motor_1, frc::SpeedController* motor_2, frc::SpeedController* motor_3, double x_mult, double y_mult, double rotation_mult) :
        DriveBase(x_mult, y_mult, rotation_mult)
    {
        //make a new mecanum drive reference
        mecanum=new frc::MecanumDrive(*motor_0, *motor_3, *motor_1, *motor_2);
    }

    void drive(double y) {
        DriveMecanum::drive(0, y, 0);
    }

    void drive(double y, double rot) {
        DriveMecanum::drive(0, y, rot);
    }

    void drive(double x, double y, double rot) {
        mecanum->DriveCartesian(
            x * getx(),
            -y * gety(),
            rot * getr()
        );
    }

    void drive(double x, double y, double rot, double time) {
        Timer{true, [=]{
            DriveMecanum::drive(x, y, rot);
        }, time};
    }

    void drive(XboxController* controller) {
        DriveMecanum::drive(
            controller->GetXLeftDeadzone(),
            controller->GetYLeftDeadzone(),
            controller->GetXRightDeadzone()
        );
    }

private:
    frc::MecanumDrive* mecanum;
};