#pragma once
#include <frc2/command/SubsystemBase.h>
#include <thrifty/canEncoder/CanEncoder.h>
#include "wrappers/NovaBase.h"
#include <units/angle.h>

class IntakeSubsystem : public frc2::SubsystemBase { 
public: 
    IntakeSubsystem();
    void Periodic() override;
    void SimulationPeriodic() override;

    //Set Percent of roller
    void IntakeRoller(double percent);

    //Set angle of intake
    void SetAngle(units::turn_t angle);

    //sets speed
    void SetSpeed(double percent);

    //Gets angle of intake in turns
    double GetAngle();
    double GetTargetAngle();

    void switchRollerState();
    bool getRollerState();

private:
    NovaMotor fuelDeploy{11};
    NovaMotor fuelIntake{12};
    thrifty::CanEncoder intakeEncoder{0, 51}; //CAN bus 0, encoder ID 51
    double targetAngle = 0.0; 
    bool rollerState = false;
};
