#include "commands/DriverCommands.h"

#include <cmath>

                                              
constexpr double driveMult = 0.25;

constexpr units::degree_t INTAKE_DOWN = 100_deg;
constexpr units::degree_t INTAKE_UP = 15_deg;

constexpr double ROLLER_SPEED = 0.9;

void DriverCommands(DriveSubsystem* m_drive, IndexSubsystem* m_index, IntakeSubsystem* m_intake, 
    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_operatorController) {

    frc2::Trigger driverStart = m_driverController->Start();
    frc2::Trigger driverRightTrigger = m_driverController->RightTrigger();
    frc2::Trigger driverRightBumper = m_driverController->RightBumper();

    frc2::Trigger driverBack = m_driverController->Back();


    //Zeroes the gyro for driving convenience
    driverStart.OnTrue(frc2::cmd::RunOnce([=] {m_drive->ZeroHeading();}, {m_drive}).IgnoringDisable(true)); 

    //Apply deadband to joystick input before scaling into physical speeds.
    // m_drive->SetDefaultCommand(frc2::cmd::Run(
    //     [=] {
    //         const double speedMultiplier = driveShootMult(m_index)();
    //         const double translationSign = m_drive->IsBlueAlliance() ? -1.0 : 1.0;
    //         m_drive->Drive(
    //             frc::ApplyDeadband(m_driverController->GetLeftY(), OIConstants::kDriveDeadband) *
    //                 translationSign * speedMultiplier * DriveConstants::kMaxSpeed,
    //             frc::ApplyDeadband(m_driverController->GetLeftX(), OIConstants::kDriveDeadband) *
    //                 translationSign * speedMultiplier * DriveConstants::kMaxSpeed,
    //             -frc::ApplyDeadband(m_driverController->GetRightX(), OIConstants::kDriveDeadband) *
    //                 speedMultiplier * DriveConstants::kMaxAngularSpeed,
    //             true);
    //     }, {m_drive}
    // ));

    //Toggle first, then command the matching angle in the same scheduled action.
    driverRightTrigger.OnTrue(frc2::cmd::RunOnce([=] {
        m_intake->switchDeployState();
        m_intake->SetAngle(m_intake->getDeployState() ? INTAKE_DOWN : INTAKE_UP);
    }, {m_intake}));

    driverRightBumper.OnTrue(runIntakeRoller(m_intake, ROLLER_SPEED));
    driverRightBumper.OnFalse(runIntakeRoller(m_intake, 0));


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

frc2::CommandPtr intakeCommands(IntakeSubsystem* m_intake, units::degree_t deployAngle, double rollerSpeed) {
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

frc2::CommandPtr setIntakeAngle(IntakeSubsystem* m_intake, units::degree_t onTrueAngle, units::degree_t onFalseAngle) {
    return frc2::cmd::RunOnce([=] {
        if (m_intake->getDeployState()) {
            m_intake->SetAngle(onTrueAngle);
        } else {
            m_intake->SetAngle(onFalseAngle);
        }
    }, {m_intake});
}

frc2::CommandPtr zeroPose(DriveSubsystem* m_drive) {
    return frc2::cmd::RunOnce([=] {
        //Reset field position and heading to the origin.
        m_drive->ResetOdometry(frc::Pose2d{});
    }, {m_drive});
}

frc2::CommandPtr switchRollerState(IntakeSubsystem* m_intake) {
    return frc2::cmd::RunOnce([=] {m_intake->switchRollerState();}, {m_intake});}

frc2::CommandPtr switchDeployState(IntakeSubsystem* m_intake) {
    return frc2::cmd::RunOnce([=] {m_intake->switchDeployState();}, {m_intake});}

    frc2::CommandPtr setDeployState(IntakeSubsystem* m_intake, bool state) {
        return frc2::cmd::RunOnce([=] {m_intake->setDeployState(state);}, {m_intake});
    }