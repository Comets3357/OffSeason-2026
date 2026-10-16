#pragma once

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IndexSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();
  std::string GetAutonomousName();
  IntakeSubsystem m_intake;
  IndexSubsystem m_index;
  VisionSubsystem m_vision;
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter{m_drive, m_vision};

 private:

  void ConfigureBindings();

  frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operatorController{OIConstants::kSecondaryControllerPort};

  frc::SendableChooser<std::string> autoChooser;


};