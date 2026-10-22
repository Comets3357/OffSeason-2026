#include "commands/DriverCommands.h"

#include <algorithm>
#include <cmath>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IndexSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace {

constexpr double kFeedingDriveMultiplier = 0.25;
constexpr double kAimP = 3.0;
constexpr double kMinimumAimDistanceMeters = 0.1;

constexpr units::turn_t kIntakeDown{40};
constexpr units::turn_t kIntakeUp{1};
constexpr double kIntakeRollerSpeed = 0.9;
constexpr double kManualDeploySpeed = 0.1;
constexpr double kHopperSpeed = 0.25;
constexpr double kKickerSpeed = 0.85;

double ApplyDriverDeadband(double input) {
    return frc::ApplyDeadband(input, OIConstants::kDriveDeadband);
}

// Point the front away from the hub so the rear-mounted shooter faces it.
units::radians_per_second_t GetHubAimTurnRate(
    DriveSubsystem* drive,
    ShooterSubsystem* shooter) {
    const auto pose = drive->GetEstimatedPose();
    const auto hub = shooter->getTargetPose();
    const double dx = (pose.X() - hub.X()).value();
    const double dy = (pose.Y() - hub.Y()).value();

    // A missing position or a position at the hub cannot define a useful heading.
    if (!std::isfinite(dx) || !std::isfinite(dy) ||
        !std::isfinite(pose.Rotation().Radians().value()) ||
        std::hypot(dx, dy) <= kMinimumAimDistanceMeters) {
        return 0_rad_per_s;
    }

    const frc::Rotation2d targetHeading{units::radian_t{std::atan2(dy, dx)}};
    const auto error = targetHeading - pose.Rotation();
    const double maxTurnRate = DriveConstants::kMaxAngularSpeed.value();

    frc::SmartDashboard::PutNumber("Shooter/Aim Error (deg)", error.Degrees().value());

    return units::radians_per_second_t{
        std::clamp(kAimP * error.Radians().value(), -maxTurnRate, maxTurnRate)};
}

void DriveWithJoystick(
    DriveSubsystem* drive,
    IndexSubsystem* index,
    ShooterSubsystem* shooter,
    frc2::CommandXboxController* driver,
    bool aimAtHub) {
    // Slow translation while feeding; preserve the existing alliance-relative controls.
    const double speedMultiplier = index->getHighSpeedRailSpeed() > 0.5
        ? kFeedingDriveMultiplier
        : 1.0;
    const double translationSign = drive->IsBlueAlliance() ? -1.0 : 1.0;
    const auto translationScale = translationSign * speedMultiplier * DriveConstants::kMaxSpeed;

    const auto forward = ApplyDriverDeadband(driver->GetLeftY()) * translationScale;
    const auto strafe = ApplyDriverDeadband(driver->GetLeftX()) * translationScale;
    const auto turnRate = aimAtHub
        ? GetHubAimTurnRate(drive, shooter)
        : -ApplyDriverDeadband(driver->GetRightX()) * speedMultiplier * DriveConstants::kMaxAngularSpeed;

    // Automatic aiming replaces rotation only; the left stick still controls translation.
    drive->Drive(forward, strafe, turnRate, true);
}

void ToggleIntakeTarget(IntakeSubsystem* intake) {
    const double target = intake->GetTargetAngle();
    const bool targetingDown = std::abs(target - kIntakeDown.value()) <
                               std::abs(target - kIntakeUp.value());

    // Use the last requested position instead of maintaining a separate deploy state.
    intake->SetAngle(targetingDown ? kIntakeUp : kIntakeDown);
}

void PublishShotStatus(double rpm, bool ready) {
    frc::SmartDashboard::PutNumber("Shooter/Commanded RPM", rpm);
    frc::SmartDashboard::PutBoolean("Shooter/At Speed", ready);
    frc::SmartDashboard::PutBoolean("Shooter/Auto Feed Active", ready);
}

} // namespace

void DriverCommands(
    DriveSubsystem* drive,
    IndexSubsystem* index,
    IntakeSubsystem* intake,
    ShooterSubsystem* shooter,
    frc2::CommandXboxController* driver,
    [[maybe_unused]] frc2::CommandXboxController* operatorController) {
    auto leftBumper = driver->LeftBumper();
    auto leftTrigger = driver->LeftTrigger();
    auto rightTrigger = driver->RightTrigger();
    auto rightBumper = driver->RightBumper();
    auto hopperButton = driver->X();
    auto kickerButton = driver->Y();
    auto deployUp = driver->POVUp();
    auto deployDown = driver->POVDown();

    // Driving: left stick translates, right stick turns, right trigger aims at the hub.
    drive->SetDefaultCommand(frc2::cmd::Run(
        [=] {
            DriveWithJoystick(drive, index, shooter, driver, rightTrigger.Get());
        },
        {drive}));

    // Shooting controls share requirements so manual kicker input cannot bypass readiness.
    // Right trigger: lookup RPM + feed.
    (rightTrigger || hopperButton || kickerButton).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            const bool shooting = rightTrigger.Get();
            double rpm = 0.0;

            if (shooting) {
                rpm = shooter->calculateTrajectory(shooter->getTargetPose()).flywheelRpm.value();
            }

            shooter->setFlywheelVelocity(rpm);

            // Recheck every loop: pause automatic feeding whenever flywheel speed drops.
            const bool ready = shooting && shooter->flywheelsAtSpeed(rpm);
            const bool runKicker = shooting ? ready : kickerButton.Get();

            index->setHopperSpeed(hopperButton.Get() ? kHopperSpeed : 0.0);
            index->setKickerSpeed(runKicker ? kKickerSpeed : 0.0);
            PublishShotStatus(rpm, ready);
        },
        [=] {
            // Stop all spinning mechanisms when these controls release or are interrupted.
            shooter->setFlywheelSpeed(0);
            index->setHopperSpeed(0);
            index->setKickerSpeed(0);
            PublishShotStatus(0, false);
        },
        {shooter, index}));

    // A toggles the intake target independently of the roller controls.
    driver->A().OnTrue(frc2::cmd::RunOnce(
        [=] { ToggleIntakeTarget(intake); },
        {intake}));

    // Left trigger lowers the intake and runs rollers; right bumper runs rollers only.
    // Left bumper takes priority: raise the intake and stop both rollers.
    (leftBumper || leftTrigger || rightBumper).WhileTrue(frc2::cmd::RunEnd(
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

    // POV up/down manually jogs deploy. Releasing either direction stops the motor.
    (deployUp || deployDown).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            intake->SetSpeed(deployUp.Get() ? kManualDeploySpeed : -kManualDeploySpeed);
        },
        [=] { intake->SetSpeed(0); },
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
