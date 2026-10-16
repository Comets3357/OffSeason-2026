#include "subsystems/IndexSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>



IndexSubsystem::IndexSubsystem() {
    HopperMotor.SetInverted(false);
    HopperMotor.SetCurrentLimit(40);
    HopperMotor.SetVoltageCompensation(9.5);
    HopperMotor.ApplyConfiguration();

    KickerMotor.SetInverted(false);
    KickerMotor.SetCurrentLimit(70);
    KickerMotor.SetVoltageCompensation(8);
    KickerMotor.ApplyConfiguration();

    KickerFollower.SetInverted(true);
    KickerFollower.SetCurrentLimit(70);
    KickerFollower.SetVoltageCompensation(8);
    KickerFollower.ApplyConfiguration();
    KickerFollower.SetFollow(kKickerCanId);
}

void IndexSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Hopper Speed", getHopperSpeed());
    frc::SmartDashboard::PutNumber("Hopper Current", HopperMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("HSL Current", KickerMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("HSL Follower", KickerFollower.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("High Speed Rail Speed", getHighSpeedRailSpeed());
}

void IndexSubsystem::SimulationPeriodic() {
    HopperMotor.SimulationPeriodic();
    KickerMotor.SimulationPeriodic();
    KickerFollower.SimulationPeriodic();
}


void IndexSubsystem::setHopperSpeed(double percent) {
    percent = std::clamp(percent, -1.0, 1.0);
    HopperMotor.Set(percent);
    HopperSpeed = percent;
}

void IndexSubsystem::setKickerSpeed(double percent) {
    percent = std::clamp(percent, -1.0, 1.0);
    KickerMotor.Set(percent);
    HighSpeedRailSpeed = percent;
}

double IndexSubsystem::getHopperSpeed() {
    return HopperSpeed;
}

double IndexSubsystem::getHighSpeedRailSpeed() {
    return HighSpeedRailSpeed;
}