#include "commands/DriverCommands.h"

#include <cmath>

                                              
constexpr double driveMult = 0.25;

constexpr units::turn_t INTAKE_DOWN{40};
constexpr units::turn_t INTAKE_UP{1};

constexpr double ROLLER_SPEED = 0.9;

void DriverCommands(DriveSubsystem* m_drive, IndexSubsystem* m_index, IntakeSubsystem* m_intake, ShooterSubsystem* m_shooter,
    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_operatorController) {

    frc2::Trigger driverStart = m_driverController->Start();
    frc2::Trigger driverLeftBumper = m_driverController->LeftBumper();
    frc2::Trigger driverLeftTrigger = m_driverController->LeftTrigger();
    frc2::Trigger driverRightTrigger = m_driverController->RightTrigger();
    frc2::Trigger driverRightBumper = m_driverController->RightBumper();
    frc2::Trigger driverX = m_driverController->X();
    frc2::Trigger driverY = m_driverController->Y();
    frc2::Trigger driverPOVUp = m_driverController->POVUp();
    frc2::Trigger driverPOVDown = m_driverController->POVDown();

    frc2::Trigger driverBack = m_driverController->Back();


    //Zeroes the gyro for driving convenience
    driverStart.OnTrue(frc2::cmd::RunOnce([=] {m_drive->ZeroHeading();}, {m_drive}).IgnoringDisable(true)); 

    //Apply deadband to joystick input before scaling into physical speeds.
    m_drive->SetDefaultCommand(frc2::cmd::Run(
        [=] {
            const double speedMultiplier = driveShootMult(m_index)();
            const double translationSign = m_drive->IsBlueAlliance() ? -1.0 : 1.0;
            m_drive->Drive(
                frc::ApplyDeadband(m_driverController->GetLeftY(), OIConstants::kDriveDeadband) *
                    translationSign * speedMultiplier * DriveConstants::kMaxSpeed,
                frc::ApplyDeadband(m_driverController->GetLeftX(), OIConstants::kDriveDeadband) *
                    translationSign * speedMultiplier * DriveConstants::kMaxSpeed,
                -frc::ApplyDeadband(m_driverController->GetRightX(), OIConstants::kDriveDeadband) *
                    speedMultiplier * DriveConstants::kMaxAngularSpeed,
                true);
        }, {m_drive}
    ));

    // Toggle from the current target, so every SetAngle call stays in sync.
    driverRightTrigger.OnTrue(frc2::cmd::RunOnce([=] {
        const double target = m_intake->GetTargetAngle();
        const bool targetingDown = std::abs(target - INTAKE_DOWN.value()) <
                                   std::abs(target - INTAKE_UP.value());
        m_intake->SetAngle(targetingDown ? INTAKE_UP : INTAKE_DOWN);
    }, {m_intake}));

    //Share the shooter requirement so hood and flywheel controls can run together.
    //Hold left bumper for hood pitch; hold left trigger for flywheel RPM.
    (driverLeftBumper || driverLeftTrigger).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            if (driverLeftBumper.Get()) m_shooter->setHoodFromDashboard();
            m_shooter->setFlywheelVelocity(
                driverLeftTrigger.Get() ? m_shooter->getSmartDashboardVelocity() : 0.0);
        },
        [=] { m_shooter->setFlywheelSpeed(0); }, {m_shooter}));

    driverRightBumper.OnTrue(runIntakeRoller(m_intake, ROLLER_SPEED));
    driverRightBumper.OnFalse(runIntakeRoller(m_intake, 0));

    //Manually move the deploy motor while the POV is held; stop on release.
    (driverPOVUp || driverPOVDown).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            m_intake->SetSpeed(driverPOVUp.Get() ? 0.1 : -0.1);
        },
        [=] {
            m_intake->SetSpeed(0);
        }, {m_intake}));

    //One command lets X and Y run independently without interrupting each other.
    (driverX || driverY).WhileTrue(frc2::cmd::RunEnd(
        [=] {
            m_index->setHopperSpeed(driverX.Get() ? 0.5 : 0.0);
            m_index->setKickerSpeed(driverY.Get() ? 0.5 : 0.0);
        },
        [=] {
            m_index->setHopperSpeed(0);
            m_index->setKickerSpeed(0);
        }, {m_index}));


    driverBack.OnTrue(zeroPose(m_drive));

}

std::function<bool()> isMoving(DriveSubsystem* m_drive) {
    return [m_drive] {
        if (std::abs(m_drive->GetRobotRelativeSpeeds().vx.value()) < 0.1 && 
            std::abs(m_drive->GetRobotRelativeSpeeds().vy.value()) < 0.1) {
            return false;
        } else return true;
    };
}

std::function<double()> driveShootMult(IndexSubsystem* m_index) {
    return [m_index] {
        if (m_index->getHighSpeedRailSpeed() > 0.5) {
            return driveMult;
        } else return 1.0;
    };
}


frc2::CommandPtr rotateTo(DriveSubsystem *drive, units::degree_t targetrot, frc2::CommandXboxController *m_driverController) {
    return frc2::cmd::Run([drive, targetrot, m_driverController] {
        drive->Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftY(), OIConstants::kDriveDeadband) *
                    DriveConstants::kMaxSpeed.value() * (drive->IsBlueAlliance() ? 1.0 : -1.0)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftX(), OIConstants::kDriveDeadband) *
                    DriveConstants::kMaxSpeed.value() * (drive->IsBlueAlliance() ? 1.0 : -1.0)},
            units::degrees_per_second_t{shortestRotation(drive->GetGyroHeading().Degrees().value(), targetrot.value())}*1.2,
        true);}, {drive});
}

double shortestRotation(double current, double target) {

    double delta = std::fmod((target-current) + 180, 360) - 180;
    return (delta < -180) ? delta + 360 : delta;
}

frc2::CommandPtr intakeCommands(IntakeSubsystem* m_intake, units::turn_t deployAngle, double rollerSpeed) {
    return frc2::cmd::RunOnce([=] {
        m_intake->SetAngle(deployAngle);
        m_intake->IntakeRoller(rollerSpeed);
    }, {m_intake});
}

frc2::CommandPtr runIntakeRoller(IntakeSubsystem* m_intake, double onTruePercent, double onFalsePercent) {
   return frc2::cmd::RunOnce([=] {
        if (m_intake->getRollerState()) {
            m_intake->IntakeRoller(onTruePercent);
        } else {
            m_intake->IntakeRoller(onFalsePercent);
        }
    }, {m_intake});
}

frc2::CommandPtr runIntakeRoller(IntakeSubsystem* m_intake, double percent) {
   return frc2::cmd::RunOnce([=] {
        m_intake->IntakeRoller(percent);
    }, {m_intake});
}

frc2::CommandPtr XLock(DriveSubsystem* m_drive) {
   return frc2::cmd::RunOnce([=] {
        m_drive->SetX();
    }, {m_drive});
}

frc2::CommandPtr zeroPose(DriveSubsystem* m_drive) {
    return frc2::cmd::RunOnce([=] {
        //Reset field position and heading to the origin.
        m_drive->ResetOdometry(frc::Pose2d{});
    }, {m_drive});
}

frc2::CommandPtr switchRollerState(IntakeSubsystem* m_intake) {
    return frc2::cmd::RunOnce([=] {m_intake->switchRollerState();}, {m_intake});}
