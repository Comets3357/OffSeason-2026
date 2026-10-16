#pragma once

#include <functional>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/IndexSubsystem.h"
#include "subsystems/ShooterSubsystem.h"



#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RunCommand.h>

#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <Constants.h>


void DriverCommands(DriveSubsystem* m_drive, IndexSubsystem* m_index, IntakeSubsystem* m_intake, ShooterSubsystem* m_shooter, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_operatorController);

std::function<bool()> isMoving(DriveSubsystem* m_drive);

std::function<double()> driveShootMult(IndexSubsystem* m_index);

frc2::CommandPtr rotateTo(DriveSubsystem *drive, units::degree_t targetrot, frc2::CommandXboxController *m_driverController);
frc2::CommandPtr setIntakeAngle(IntakeSubsystem* m_intake, units::degree_t onTrueAngle, units::degree_t onFalseAngle);
frc2::CommandPtr runIntakeRoller(IntakeSubsystem* m_intake, double onTruePercent, double onFalsePercent);
frc2::CommandPtr runIntakeRoller(IntakeSubsystem* m_intake, double speed);
frc2::CommandPtr switchRollerState(IntakeSubsystem* m_intake);
frc2::CommandPtr switchDeployState(IntakeSubsystem* m_intake);
frc2::CommandPtr setDeployState(IntakeSubsystem* m_intake, bool state);

frc2::CommandPtr XLock(DriveSubsystem* m_drive);
frc2::CommandPtr zeroPose(DriveSubsystem* m_drive);



double shortestRotation(double current, double target);

double slopedDeadband(double value, double deadband);
frc2::CommandPtr intakeCommands(IntakeSubsystem* m_intake, units::degree_t deployAngle, double rollerSpeed);