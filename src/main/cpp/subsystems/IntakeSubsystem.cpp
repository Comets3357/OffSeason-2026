#include "subsystems/IntakeSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <algorithm>

constexpr double INTAKE_ENCODER_RATIO = 1.0 / 105.0;
constexpr units::turn_t kDeployReverseLimit{0};
constexpr units::turn_t kDeployForwardLimit{44};


//Constructor
IntakeSubsystem::IntakeSubsystem() {

    //PID values of intake 
    const double intakeP = 0.1; //volts per encoder rotation of error, not per degree
    const double intakeI = 0;
    const double intakeD = 0;

    fuelIntake.SetCurrentLimit(65);
    fuelIntake.SetInverted(true);
    fuelIntake.ApplyConfiguration();

    // The second roller receives the same output with opposite inversion.
    fuelIntakeSecondary.SetCurrentLimit(65);
    fuelIntakeSecondary.SetInverted(false);
    fuelIntakeSecondary.ApplyConfiguration();

    fuelDeploy.SetCurrentLimit(40);
    fuelDeploy.SetExternalEncoder(51); //Thrifty through-bore CAN encoder
    fuelDeploy.SetAbsoluteConversionFactor(1);
    fuelDeploy.SetSoftLimitSource(thrifty::Motor::FeedbackSensorType::INTERNAL);
    fuelDeploy.SetInverted(true);
    fuelDeploy.SetAbsoluteWrapping(false);
    fuelDeploy.SetForwardSoftLimit(kDeployForwardLimit);
    fuelDeploy.SetReverseSoftLimit(kDeployReverseLimit);
    fuelDeploy.EnableForwardSoftLimit(true);
    fuelDeploy.Set(0);
    fuelDeploy.SetMaxOutput(1);

    fuelDeploy.SetInternalEncoderPosition(units::turn_t{intakeEncoder.Status().GetPositionAbs() * 105});
    fuelDeploy.SetPID(intakeP, intakeI, intakeD, 0, 0);
    fuelDeploy.ApplyConfiguration();



}

void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Intake angle (turns)", GetAngle());
    frc::SmartDashboard::PutNumber("Intake absolute encoder (turns)",
        units::turn_t{fuelDeploy.getAbsolutePosition()}.value());
    frc::SmartDashboard::PutNumber("Intake target angle (turns)", GetTargetAngle());
    frc::SmartDashboard::PutNumber("Intake Amperage", fuelDeploy.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Intake Applied Voltage", fuelDeploy.GetAppliedVoltage());
    frc::SmartDashboard::PutNumber("Intake Angle Error (turns)", GetTargetAngle() - GetAngle());
    frc::SmartDashboard::PutNumber("Intake roller speed", fuelIntake.getVelocity().value());
}

void IntakeSubsystem::SimulationPeriodic() {
    fuelDeploy.SimulationPeriodic();
    fuelIntake.SimulationPeriodic();
    fuelIntakeSecondary.SimulationPeriodic();
}

//Sets intake percentage (variable)
void IntakeSubsystem::IntakeRoller(double percent) {
    fuelIntake.Set(percent);
    fuelIntakeSecondary.Set(percent);
}

void IntakeSubsystem::SetAngle(units::turn_t angle) {
    // Use the configured limits; CAN status may not have reported them yet.
    targetAngle = std::clamp(angle.value(),
        kDeployReverseLimit.value(), kDeployForwardLimit.value());
    fuelDeploy.SetPosition(units::turn_t{targetAngle});
}

void IntakeSubsystem::SetSpeed(double percent){
    fuelDeploy.Set(percent);
}

double IntakeSubsystem::GetTargetAngle() {
    return targetAngle;
}

void IntakeSubsystem::switchRollerState() {
    rollerState = !rollerState;
}

bool IntakeSubsystem::getRollerState() {
    return rollerState;
}

// Match the internal motor-turn units used by SetAngle and the soft limits.
double IntakeSubsystem::GetAngle() {
    return units::turn_t{fuelDeploy.getPosition()}.value();
}
