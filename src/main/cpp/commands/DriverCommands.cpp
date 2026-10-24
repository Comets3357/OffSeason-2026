#include "commands/DriverCommands.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IndexSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace {

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

// Shooting controls do not change the driver's speed or rotation input.
void DriveWithJoystick(
    DriveSubsystem* drive,
    frc2::CommandXboxController* driver) {
    const double translationSign = drive->IsBlueAlliance() ? -1.0 : 1.0;
    const auto translationScale = translationSign * DriveConstants::kMaxSpeed;

    const auto forward = ApplyDriverDeadband(driver->GetLeftY()) * translationScale;
    const auto strafe = ApplyDriverDeadband(driver->GetLeftX()) * translationScale;
    const auto turnRate = -ApplyDriverDeadband(driver->GetRightX()) * DriveConstants::kMaxAngularSpeed;

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
    auto rightTrigger = operatorController->RightTrigger();
    auto hopperButton = operatorController->B() || driver->B();
    auto reverseKicker = operatorController->X();

    // These commands only adjust trim, so they must not interrupt shooting.
    operatorController->POVUp().OnTrue(frc2::cmd::RunOnce(
        [=] { shooter->adjustFlywheelTrim(kFlywheelTrimStepRpm); }));
    operatorController->POVDown().OnTrue(frc2::cmd::RunOnce(
        [=] { shooter->adjustFlywheelTrim(-kFlywheelTrimStepRpm); }));

    // Driver sticks always control translation and rotation.
    drive->SetDefaultCommand(frc2::cmd::Run(
        [=] {
            DriveWithJoystick(drive, driver);
        },
        {drive}));

    // Either B runs the hopper floor; releasing both stops it.
    // Right trigger: lookup hood position and RPM, then feed at speed.
    (rightTrigger || hopperButton || reverseKicker).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            const bool shooting = rightTrigger.Get();
            double rpm = 0.0;

            if (shooting) {
                const auto shot = shooter->calculateTrajectory(shooter->getTargetPose());
                shooter->setHoodPitch(shot.hoodMotorTurns);
                rpm = shooter->applyFlywheelTrim(shot.flywheelRpm.value());
            } else {
                // Return the hood even if B or X stays held.
                shooter->setHoodPitch(kHoodRestPosition);
            }

            shooter->setFlywheelVelocity(rpm);

            // Recheck every loop: pause automatic feeding whenever flywheel speed drops.
            const bool ready = shooting && shooter->flywheelsAtSpeed(rpm);

            index->setHopperSpeed(hopperButton.Get() ? kHopperSpeed : 0.0);
            // Manual reverse takes priority over automatic forward feeding.
            const bool reversing = reverseKicker.Get();
            const bool feeding = ready && !reversing;
            index->setKickerSpeed(reversing ? -kKickerSpeed : (feeding ? kKickerSpeed : 0.0));
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
