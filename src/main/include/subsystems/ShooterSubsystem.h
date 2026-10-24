#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "common/ShooterLookup.h"
#include "wrappers/NovaBase.h"

class DriveSubsystem;

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    explicit ShooterSubsystem(DriveSubsystem& drive);

    void Periodic() override;
    void SimulationPeriodic() override;

    void setFlywheelSpeed(double percent);
    void setFlywheelVelocity(double rpm);
    double getSmartDashboardVelocity();
    bool flywheelsAtSpeed(double targetRpm);
    // Positive trim increases shooting speed, preserving the lookup direction.
    void adjustFlywheelTrim(double deltaRpm);
    double applyFlywheelTrim(double lookupRpm) const;

    // Native internal encoder turns, clamped to 0–18.5 with no offset.
    void setHoodPitch(units::turn_t position);
    void setHoodFromDashboard();
    double getHoodPitch();

    // Lookup hood positions use native internal motor rotations.
    struct ShotSetpoints {
        units::turn_t hoodMotorTurns;
        units::revolutions_per_minute_t flywheelRpm;
    };
    ShotSetpoints calculateShot(units::meter_t distance);
    ShotSetpoints calculateTrajectory(frc::Pose2d targetPose);
    frc::Pose2d getTargetPose();

private:
    void updateFlywheelGains();

    static constexpr double kHoodMinTurns = 0.0;
    static constexpr double kHoodMaxTurns = 18.5;
    static constexpr double kHoodDefaultTurns = (kHoodMinTurns + kHoodMaxTurns) / 2.0;

    DriveSubsystem& m_drive;
    ShooterLookup shooterLookup;
    NovaMotor primaryFlywheelMotor{31};
    NovaMotor secondaryFlywheelMotor{32};
    NovaMotor thirdFlywheelMotor{33};
    NovaMotor shooterPitchMotor{41};

    double flywheelTrimRpm = 0.0;
    double targetHoodMotorTurns = 0.0;
    double flywheelP = 0.004;
    double flywheelI = 0.0;
    double flywheelD = 0.018;
    double flywheelFF = 0.0042;
};
