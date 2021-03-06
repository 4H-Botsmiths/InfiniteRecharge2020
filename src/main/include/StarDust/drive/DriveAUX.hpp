#pragma once

#include "StarDust/sensor/motion/Gyro_Base.hpp"
#include "StarDust/core/StarDustComponent.hpp"
#include "StarDust/control/XboxController.hpp"
#include "StarDust/drive/DriveBase.hpp"

class DriveAUX : public StarDustComponent {
public:
    DriveAUX(DriveBase* driveBase, Gyro_Base* gyro, double threshold);

    void GyroRotateToZero(double range);
    void GyroRotateTo(double degree, double range);
    void GyroRotateTo(double degree, double range, double y_mult);
    void GyroRotateTo(double degree, double range, double x_mult, double y_mult);

    void driveGyro(double degree, double range, XboxController* controller);
    void driveGyro(double degree, double range, double x_mult, double y_mult);

    void drive(XboxController* controller);
    void drive(double x, double y, double z);
    void drive(double x, double y, double z, double deg);

private:
    double calculateRotation(double degree, double range);

    DriveBase* driveBase;
    Gyro_Base* gyro;

    //minimum amount needed to move drivetrain
    double threshold;
};