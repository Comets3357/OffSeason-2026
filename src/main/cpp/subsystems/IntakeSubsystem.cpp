#include "subsystems/IntakeSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <algorithm>

constexpr double INTAKE_ENCODER_RATIO = 1.0 / 105.0;


//Constructor
IntakeSubsystem::IntakeSubsystem() {

    //PID values of intake 
    const double intakeP = 2;
    const double intakeI = 0;
    const double intakeD = 0;

    fuelIntake.SetCurrentLimit(65);
    fuelIntake.SetInverted(true);
    fuelIntake.ApplyConfiguration();

    fuelDeploy.SetCurrentLimit(40);
    fuelDeploy.SetExternalEncoder(51); //Thrifty through-bore CAN encoder
    fuelDeploy.SetAbsoluteConversionFactor(1);
    fuelDeploy.SetSoftLimitSource(thrifty::Motor::FeedbackSensorType::ABS);
    fuelDeploy.SetInverted(true);
    fuelDeploy.SetAbsoluteWrapping(false);
    fuelDeploy.SetForwardSoftLimit(134_deg);
    fuelDeploy.SetReverseSoftLimit(5_deg);
    fuelDeploy.EnableForwardSoftLimit(true);
    fuelDeploy.Set(0);
    fuelDeploy.SetMaxOutput(1);
    fuelDeploy.SetPID(intakeP, intakeI, intakeD, 0, 0);
    fuelDeploy.ApplyConfiguration();



}

void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Intake angle", GetAngle());
    frc::SmartDashboard::PutNumber("Intake target angle", GetTargetAngle());
    frc::SmartDashboard::PutNumber("Intake Amperage", fuelDeploy.GetAppliedVoltage());
    frc::SmartDashboard::PutNumber("Intake roller speed", fuelIntake.getVelocity().value());
}

void IntakeSubsystem::SimulationPeriodic() {
    fuelDeploy.SimulationPeriodic();
    fuelIntake.SimulationPeriodic();
}

//Sets intake percentage (variable)
void IntakeSubsystem::IntakeRoller(double percent) {
    fuelIntake.Set(percent);
}

void IntakeSubsystem::SetAngle(units::degree_t angle) {
    targetAngle = std::clamp(angle.value(), 5.0, 125.0);
    fuelDeploy.SetAbsolutePosition(units::degree_t{targetAngle});
}

void IntakeSubsystem::SetSpeed(double percent){
    fuelDeploy.Set(percent);
}

double IntakeSubsystem::GetTargetAngle() {
    return targetAngle;
}

void IntakeSubsystem::switchDeployState() {
    deployState = !deployState;
}

void IntakeSubsystem::setDeployState(bool state) {
    deployState = state;
}

void IntakeSubsystem::switchRollerState() {
    rollerState = !rollerState;
}

bool IntakeSubsystem::getDeployState() {
    return deployState;
}

bool IntakeSubsystem::getRollerState() {
    return rollerState;
}

//returns angle of fuel deploy
double IntakeSubsystem::GetAngle() {
    return fuelDeploy.getAbsolutePosition().value();
}
