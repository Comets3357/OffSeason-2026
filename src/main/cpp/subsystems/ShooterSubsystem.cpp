#include "subsystems/ShooterSubsystem.h"
#include "subsystems/DriveSubsystem.h"

#include <algorithm>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem(DriveSubsystem& drive) : m_drive(drive) {
    MotorBase::MotorConfig flywheelConfig;
    flywheelConfig.currentLimitAmps = 60;
    flywheelConfig.enableVoltageCompensation = false;
    flywheelConfig.positionConversionFactor = 0.5;
    flywheelConfig.velocityConversionFactor = 0.5;
    for (auto* motor : {&primaryFlywheelMotor, &secondaryFlywheelMotor, &thirdFlywheelMotor}) {
        motor->ApplyConfiguration(flywheelConfig);
        motor->SetPID(flywheelP, flywheelI, flywheelD, flywheelFF, 0);
    }
    thirdFlywheelMotor.SetInverted(true);
    setFlywheelSpeed(0);

    // Use the existing internal encoder position for control and soft limits.
    shooterPitchMotor.SetCurrentLimit(20);
    shooterPitchMotor.SetInverted(false);
    shooterPitchMotor.SetSoftLimitSource(thrifty::Motor::FeedbackSensorType::INTERNAL);
    shooterPitchMotor.SetForwardSoftLimit(units::turn_t{kHoodMaxTurns});
    shooterPitchMotor.SetReverseSoftLimit(units::turn_t{kHoodMinTurns});
    shooterPitchMotor.EnableForwardSoftLimit(true); // Nova enables both limits together.
    shooterPitchMotor.SetMaxOutput(1);
    shooterPitchMotor.Set(0);
    shooterPitchMotor.SetPID(2, 0, 0, 0, 0);
    shooterPitchMotor.SetInternalEncoderPosition(0_deg);
    shooterPitchMotor.ApplyConfiguration();
    targetHoodMotorTurns = getHoodPitch();

    shooterLookup.Init();
    frc::SmartDashboard::PutNumber("Smart Dashboard Flywheel Velocity", 0);
    frc::SmartDashboard::PutNumber("Flywheel P", flywheelP);
    frc::SmartDashboard::PutNumber("Flywheel I", flywheelI);
    frc::SmartDashboard::PutNumber("Flywheel D", flywheelD);
    frc::SmartDashboard::PutNumber("Flywheel FF", flywheelFF);
    frc::SmartDashboard::PutNumber("Hood Target (motor turns)", kHoodDefaultTurns);
}

void ShooterSubsystem::Periodic() {
    updateFlywheelGains();

    const double position = getHoodPitch();
    frc::SmartDashboard::PutNumber("Hood Position (motor turns)", position);
    frc::SmartDashboard::PutNumber("Hood Raw Internal Position (motor turns)",
        units::turn_t{shooterPitchMotor.getPosition()}.value());
    frc::SmartDashboard::PutNumber("Hood Applied Target (motor turns)", targetHoodMotorTurns);
    frc::SmartDashboard::PutNumber("Hood Position Error (motor turns)", targetHoodMotorTurns - position);
    frc::SmartDashboard::PutNumber("Hood Applied Voltage", shooterPitchMotor.GetAppliedVoltage());
    frc::SmartDashboard::PutNumber("Hood Current (amps)", shooterPitchMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Flywheel Velocity", primaryFlywheelMotor.getVelocity().value());
    frc::SmartDashboard::PutNumber("Secondary Flywheel RPM", secondaryFlywheelMotor.getVelocity().value());
    frc::SmartDashboard::PutNumber("Third Flywheel RPM", thirdFlywheelMotor.getVelocity().value());

    const auto distance = m_drive.GetEstimatedPose().Translation().Distance(getTargetPose().Translation());
    const auto shot = calculateShot(distance);
    frc::SmartDashboard::PutNumber("Lookup Distance (meters)", distance.value());
    frc::SmartDashboard::PutNumber("Lookup Hood Angle (degrees)", shot.hoodAngle.value());
    frc::SmartDashboard::PutNumber("Lookup Flywheel RPM", shot.flywheelRpm.value());
}

void ShooterSubsystem::SimulationPeriodic() {
    primaryFlywheelMotor.SimulationPeriodic();
    secondaryFlywheelMotor.SimulationPeriodic();
    thirdFlywheelMotor.SimulationPeriodic();
    shooterPitchMotor.SimulationPeriodic();
}

void ShooterSubsystem::setFlywheelSpeed(double percent) {
    const double output = std::isfinite(percent) ? std::clamp(percent, -1.0, 1.0) : 0.0;
    primaryFlywheelMotor.Set(output);
    secondaryFlywheelMotor.Set(output);
    thirdFlywheelMotor.Set(output);
}

void ShooterSubsystem::setFlywheelVelocity(double rpm) {
    if (!std::isfinite(rpm) || rpm == 0) {
        setFlywheelSpeed(0);
        return;
    }
    primaryFlywheelMotor.SetVelocity(units::revolutions_per_minute_t{rpm});
    secondaryFlywheelMotor.SetVelocity(units::revolutions_per_minute_t{rpm});
    thirdFlywheelMotor.SetVelocity(units::revolutions_per_minute_t{rpm});
}

bool ShooterSubsystem::flywheelsAtSpeed(double targetRpm) {
    if (!std::isfinite(targetRpm) || std::abs(targetRpm) < 100.0) return false;
    const double tolerance = std::abs(targetRpm) * 0.05;
    for (auto* motor : {&primaryFlywheelMotor, &secondaryFlywheelMotor, &thirdFlywheelMotor}) {
        const double actual = motor->getVelocity().value();
        if (!std::isfinite(actual) || std::abs(actual - targetRpm) > tolerance) return false;
    }
    return true;
}

double ShooterSubsystem::getSmartDashboardVelocity() {
    return frc::SmartDashboard::GetNumber("Smart Dashboard Flywheel Velocity", 0);
}

void ShooterSubsystem::setHoodPitch(units::turn_t position) {
    if (!std::isfinite(position.value())) return;
    targetHoodMotorTurns = std::clamp(position.value(), kHoodMinTurns, kHoodMaxTurns);
    shooterPitchMotor.SetPosition(units::turn_t{targetHoodMotorTurns});
}

void ShooterSubsystem::setHoodFromDashboard() {
    setHoodPitch(units::turn_t{frc::SmartDashboard::GetNumber(
        "Hood Target (motor turns)", kHoodDefaultTurns)});
}

double ShooterSubsystem::getHoodPitch() {
    return units::turn_t{shooterPitchMotor.getPosition()}.value();
}

ShooterSubsystem::ShotSetpoints ShooterSubsystem::calculateShot(units::meter_t distance) {
    if (!std::isfinite(distance.value()) || distance.value() < 0) return {};
    return {units::degree_t{shooterLookup.calculateHoodAngle(distance)},
            units::revolutions_per_minute_t{shooterLookup.calculateFlywheelVelocity(distance)}};
}

ShooterSubsystem::ShotSetpoints ShooterSubsystem::calculateTrajectory(frc::Pose2d targetPose) {
    return calculateShot(m_drive.GetEstimatedPose().Translation().Distance(targetPose.Translation()));
}

frc::Pose2d ShooterSubsystem::getTargetPose() {
    return m_drive.IsBlueAlliance() ? frc::Pose2d{4.625_m, 4.035_m, 0_rad}
                                    : frc::Pose2d{11.92_m, 4.035_m, 0_rad};
}

void ShooterSubsystem::updateFlywheelGains() {
    const double p = frc::SmartDashboard::GetNumber("Flywheel P", flywheelP);
    const double i = frc::SmartDashboard::GetNumber("Flywheel I", flywheelI);
    const double d = frc::SmartDashboard::GetNumber("Flywheel D", flywheelD);
    const double ff = frc::SmartDashboard::GetNumber("Flywheel FF", flywheelFF);
    if (!std::isfinite(p) || !std::isfinite(i) || !std::isfinite(d) || !std::isfinite(ff)) return;
    if (p == flywheelP && i == flywheelI && d == flywheelD && ff == flywheelFF) return;

    flywheelP = p;
    flywheelI = i;
    flywheelD = d;
    flywheelFF = ff;
    for (auto* motor : {&primaryFlywheelMotor, &secondaryFlywheelMotor, &thirdFlywheelMotor}) {
        motor->SetPID(p, i, d, ff, 0);
    }
}
