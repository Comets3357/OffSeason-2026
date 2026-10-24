#include "commands/DriverCommands.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <frc/controller/PIDController.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IndexSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace {

// Hub-facing rotation PID: radians in, radians/second out.
constexpr double kHubAimP = 3.0;
constexpr double kHubAimI = 0.0;
constexpr double kHubAimD = 0.0;
constexpr double kMinimumAimDistanceMeters = 0.1;

constexpr units::turn_t kIntakeDown{40};
constexpr units::turn_t kIntakeUp{1};
constexpr double kIntakeRollerSpeed = 0.9;
constexpr double kFlywheelTrimStepRpm = 50.0;
constexpr double kHopperSpeed = 0.25;
constexpr double kKickerSpeed = 0.85;
constexpr units::turn_t kHoodRestPosition{1};

double ApplyDriverDeadband(double input) {
    return frc::ApplyDeadband(input, OIConstants::kDriveDeadband);
}

// Operator left trigger overrides rotation only; driver translation stays active.
void DriveWithJoystick(
    DriveSubsystem* drive,
    frc2::CommandXboxController* driver,
    ShooterSubsystem* shooter,
    frc::PIDController& aimPid,
    bool aimAtHub) {
    const double translationSign = drive->IsBlueAlliance() ? -1.0 : 1.0;
    const auto translationScale = translationSign * DriveConstants::kMaxSpeed;

    const auto forward = ApplyDriverDeadband(driver->GetLeftY()) * translationScale;
    const auto strafe = ApplyDriverDeadband(driver->GetLeftX()) * translationScale;
    auto turnRate = -ApplyDriverDeadband(driver->GetRightX()) * DriveConstants::kMaxAngularSpeed;

    if (aimAtHub) {
        const auto pose = drive->GetEstimatedPose();
        const auto hub = shooter->getTargetPose();
        // Front faces away from the hub so the rear shooter faces toward it.
        const double dx = (pose.X() - hub.X()).value();
        const double dy = (pose.Y() - hub.Y()).value();
        const double heading = pose.Rotation().Radians().value();
        if (std::isfinite(dx) && std::isfinite(dy) && std::isfinite(heading) &&
            std::hypot(dx, dy) > kMinimumAimDistanceMeters) {
            const double output = aimPid.Calculate(heading, std::atan2(dy, dx));
            const double maxRate = DriveConstants::kMaxAngularSpeed.value();
            turnRate = units::radians_per_second_t{std::clamp(output, -maxRate, maxRate)};
        } else {
            aimPid.Reset();
            turnRate = 0_rad_per_s;
        }
    } else {
        aimPid.Reset();
    }

    drive->Drive(forward, strafe, turnRate, true);
}

void PublishShotStatus(double rpm, bool ready, bool feeding) {
    frc::SmartDashboard::PutNumber("Shooter/Commanded RPM", rpm);
    frc::SmartDashboard::PutBoolean("Shooter/At Speed", ready);
    frc::SmartDashboard::PutBoolean("Shooter/Auto Feed Active", feeding);
}

} // namespace

void DriverCommands(
    DriveSubsystem* drive,
    IndexSubsystem* index,
    IntakeSubsystem* intake,
    ShooterSubsystem* shooter,
    frc2::CommandXboxController* driver,
    frc2::CommandXboxController* operatorController) {
    // Driver controls movement and intake deployment.
    auto leftBumper = driver->LeftBumper();
    auto leftTrigger = driver->LeftTrigger();

    // Operator controls shooting; either controller can run the hopper floor.
    auto aimTrigger = operatorController->LeftTrigger();
    auto rightTrigger = operatorController->RightTrigger();
    auto hopperButton = operatorController->B() || driver->B();
    auto reverseKicker = operatorController->X();
    auto manualKicker = operatorController->Y();
    auto dashboardHood = operatorController->A();

    // These commands only adjust trim, so they must not interrupt shooting.
    operatorController->POVUp().OnTrue(frc2::cmd::RunOnce(
        [=] { shooter->adjustFlywheelTrim(kFlywheelTrimStepRpm); }));
    operatorController->POVDown().OnTrue(frc2::cmd::RunOnce(
        [=] { shooter->adjustFlywheelTrim(-kFlywheelTrimStepRpm); }));

    // Keep the controller alive for the lifetime of the default command.
    auto aimPid = std::make_shared<frc::PIDController>(kHubAimP, kHubAimI, kHubAimD);
    aimPid->EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

    drive->SetDefaultCommand(frc2::cmd::RunEnd(
        [=] {
            DriveWithJoystick(drive, driver, shooter, *aimPid, aimTrigger.Get());
        },
        [=] {
            aimPid->Reset();
            drive->Stop();
        },
        {drive}));

    // Either B runs the hopper floor; releasing both stops it.
    // Right trigger: lookup hood position and RPM, then feed at speed.
    // Y runs the kicker manually; A uses the dashboard hood target.
    (rightTrigger || hopperButton || reverseKicker || manualKicker || dashboardHood).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            const bool lookupShooting = rightTrigger.Get();
            double rpm = 0.0;

            if (lookupShooting) {
                const auto shot = shooter->calculateTrajectory(shooter->getTargetPose());
                rpm = shooter->applyFlywheelTrim(shot.flywheelRpm.value());
                if (!dashboardHood.Get()) shooter->setHoodPitch(shot.hoodMotorTurns);
            } else if (!dashboardHood.Get()) {
                shooter->setHoodPitch(kHoodRestPosition);
            }

            if (dashboardHood.Get()) shooter->setHoodFromDashboard();

            shooter->setFlywheelVelocity(rpm);

            // Recheck every loop: pause automatic feeding whenever flywheel speed drops.
            const bool ready = lookupShooting && shooter->flywheelsAtSpeed(rpm);

            index->setHopperSpeed(hopperButton.Get() ? kHopperSpeed : 0.0);
            // X reverse takes priority; Y runs forward without waiting for flywheel speed.
            const bool reversing = reverseKicker.Get();
            const bool feeding = ready && !reversing;
            const bool runKicker = !reversing && (manualKicker.Get() || ready);
            index->setKickerSpeed(reversing ? -kKickerSpeed : (runKicker ? kKickerSpeed : 0.0));
            PublishShotStatus(rpm, ready, feeding);
        },
        [=] {
            // Stop all spinning mechanisms when these controls release or are interrupted.
            shooter->setHoodPitch(kHoodRestPosition);
            shooter->setFlywheelSpeed(0);
            index->setHopperSpeed(0);
            index->setKickerSpeed(0);
            PublishShotStatus(0, false, false);
        },
        {shooter, index}));

    // Left trigger lowers the intake and runs both rollers.
    // Left bumper takes priority: raise the intake and stop both rollers.
    (leftBumper || leftTrigger).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            if (leftBumper.Get()) {
                intake->SetAngle(kIntakeUp);
            } else if (leftTrigger.Get()) {
                intake->SetAngle(kIntakeDown);
            }
            intake->IntakeRoller(leftBumper.Get() ? 0.0 : kIntakeRollerSpeed);
        },
        // Leave deploy at its last target; stop rollers on release or interruption.
        [=] { intake->IntakeRoller(0); },
        {intake}));

    // Start aligns heading to alliance forward, including while disabled.
    driver->Start().OnTrue(frc2::cmd::RunOnce(
        [=] { drive->ZeroHeading(); },
        {drive}).IgnoringDisable(true));

    // Back resets both field position and heading to the origin.
    driver->Back().OnTrue(frc2::cmd::RunOnce(
        [=] { drive->ResetOdometry(frc::Pose2d{}); },
        {drive}));
}
