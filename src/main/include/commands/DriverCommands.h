#pragma once

#include <frc2/command/button/CommandXboxController.h>

class DriveSubsystem;
class IndexSubsystem;
class IntakeSubsystem;
class ShooterSubsystem;

// Registers driver controls. The operator controller is reserved for future bindings.
void DriverCommands(
    DriveSubsystem* drive,
    IndexSubsystem* index,
    IntakeSubsystem* intake,
    ShooterSubsystem* shooter,
    frc2::CommandXboxController* driver,
    frc2::CommandXboxController* operatorController);
