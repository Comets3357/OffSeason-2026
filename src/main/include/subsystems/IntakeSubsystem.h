#pragma once
#include <frc2/command/SubsystemBase.h>
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
    void SetAngle(units::degree_t angle);

    //sets speed
    void SetSpeed(double percent);

    //Gets angle of intake
    double GetAngle();
    double GetTargetAngle();

    void switchDeployState();
    void setDeployState(bool state);
    void switchRollerState();
    bool getDeployState();
    bool getRollerState();

private:
    NovaMotor fuelDeploy{11};
    NovaMotor fuelIntake{12};
    double targetAngle = 0.0; 
    bool deployState = false;
    bool rollerState = false;
};