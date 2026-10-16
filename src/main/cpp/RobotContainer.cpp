#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <commands/DriverCommands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>


RobotContainer::RobotContainer() {

  // autoChooser.SetDefaultOption("Potato", "potato");
  // autoChooser.AddOption("Left Trench SINGLE Pass Under Trench", "LeftTrenchSinglePassUnderTrench");
  // autoChooser.AddOption("Left Trench DOUBLE Pass Under Trench", "LeftTrenchDoublePassUnderTrench");
  // autoChooser.AddOption("Left Trench SINGLE Pass AND DEPOT", "LeftTrenchSinglePassDepotCollect");
  // autoChooser.AddOption("Right Trench SINGLE Pass Under Trench", "RightTrenchSinglePassUnderTrench");
  // autoChooser.AddOption("Right Trench DOUBLE Pass Under Trench", "RightTrenchDoublePassUnderTrench");
  // autoChooser.AddOption("Middle to Depot", "MidThenDepot");

  frc::SmartDashboard::PutData("Auto Selector", &autoChooser);

  ConfigureBindings();

  // autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  // frc::SmartDashboard::PutData("Sendable Chooser", &autoChooser);
  
} 
void RobotContainer::ConfigureBindings() {
  DriverCommands(&m_drive, &m_index, &m_intake, &m_shooter, &m_driverController, &m_operatorController);
  // OperatorCommands(&m_drive, &m_intake, &m_climb, &m_shooter, &m_index, &m_operatorController);
  // AutonCommands(&m_drive, &m_climb, &m_intake, &m_index, &m_shooter);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

  if (autoChooser.GetSelected().empty()) return frc2::cmd::None();
  return pathplanner::PathPlannerAuto(autoChooser.GetSelected()).ToPtr();
}

std::string RobotContainer::GetAutonomousName() {
  return autoChooser.GetSelected();
}