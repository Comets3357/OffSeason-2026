#pragma once

#include "wrappers/MotorBase.h"

#include <rev/SparkBase.h>
#include <rev/SparkFlex.h>
#include <frc/RobotBase.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkClosedLoopController.h>

class SparkFlexMotor : public MotorBase
{
private:
    rev::spark::SparkFlex motor;

    rev::spark::SparkAbsoluteEncoder AbsoluteEncoder = motor.GetAbsoluteEncoder();
    rev::spark::SparkRelativeEncoder RelativeEncoder = motor.GetEncoder();
    // rev::spark::SparkFlexExternalEncoder ExternalRelativeEncoder = motor.GetExternalEncoder();
    rev::spark::SparkBaseConfig config;
    rev::spark::SparkClosedLoopController closedLoopController = motor.GetClosedLoopController();
    double lastRelEncPos;

public:
    
    // CONSTURCTOR //
    SparkFlexMotor(int id) : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless}
    {
        SetPID(0, 0, 0, 0);
        config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        if (frc::RobotBase::IsSimulation()) {
            auto plant = frc::LinearSystemId::DCMotorSystem(
                frc::DCMotor::NEO(1), 
                1.0_kg_sq_m,
                0.0005
            ); 
        }
    }

    //basic function for setting speed before anything special.
   void Set(double speed) override;

    //more complicated than Set, but just setting motor voltage.
   void SetVoltage(double voltage) override;

    //stop the motor, overrides all previous commands and stop the motor
   void Stop() override;

    //sets the motor to go a specific velocity.
   void SetVelocity(double rpm) override;

    //sets the motor to a specific position.
   void SetPosition(units::angle::degree_t degree) override;

    //configuration setter for inversion
   void SetInverted(bool inversion) override;

    //configuration setter for neutral move
   void SetNeutralMode(NeutralMode neutralMode) override;

    //configuration setter for enabling current limit
   void EnableCurrentLimit(bool enable) override;

    //configuration setter for current limit value
   void SetCurrentLimit(int current) override;

    //configuration setter for enabling the forward soft limit
   void EnableForwardSoftLimit(bool enable) override;

    //configuration setter for enabling the reverse soft limit
   void EnableReverseSoftLimit(bool enable) override;

    //configuration setter for setting the forward limit position
   void SetForwardSoftLimit(units::angle::degree_t degrees) override;

    //configuration setter for setting the reverse limit position
   void SetReverseSoftLimit(units::angle::degree_t degrees) override;

    //apply the configuration of values
   void ApplyConfiguration(MotorConfig motorConfig) override;

    //virtual function to set the PIDs
   void SetPID(double p, double i, double d, double ff) override;

    //virtual function to recieve if the forward limit is enabled;
   bool IsForwardLimitEnabled() override;

    //virtual function to recieve if the reverse limit is enabled;
   bool IsReverseLimitEnabled() override;

   void SetConversionFactor(double conversion) override;

    //virtual function to pull the current position of the motor
    //(prioritizes absolute)
   units::angle::degree_t GetPosition() override;

    //getter to return the velocity of the current motor controller
    //(Prioritizes absolute)
    //TODO give it a better type!!
   double GetVelocity() override;
};

class SparkMaxMotor : public MotorBase
{
private:
    rev::spark::SparkMax motor;
    rev::spark::SparkAbsoluteEncoder AbsoluteEncoder = motor.GetAbsoluteEncoder();
    rev::spark::SparkRelativeEncoder RelativeEncoder = motor.GetEncoder();
    rev::spark::SparkClosedLoopController closedLoopController = motor.GetClosedLoopController();
    rev::spark::SparkBaseConfig config;
    rev::spark::SparkLimitSwitch ReverseLimit = motor.GetReverseLimitSwitch();
    std::unique_ptr<rev::spark::SparkMaxSim> m_motorSim;
    frc::DCMotor m_motorType = frc::DCMotor::NEO(1);
    double m_lastOutput = 0.0;
    double RPM = 0.0;
public:
    // CONSTURCTOR //
    SparkMaxMotor(int id) : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless}
    {
        SetPID(0, 0, 0, 0);
        config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake); // default idle mode to break
        if (frc::RobotBase::IsSimulation()) {
            m_motorSim = std::make_unique<rev::spark::SparkMaxSim>(
                &motor,
                &m_motorType
            );
        }
    }

    //basic function for setting speed before anything special.
   void Set(double speed) override;

    //more complicated than Set, but just setting motor voltage.
   void SetVoltage(double voltage) override;

    //stop the motor, overrides all previous commands and stop the motor
   void Stop() override;

    //sets the motor to go a specific velocity.
   void SetVelocity(double rpm) override;

    //sets the motor to a specific position.
   void SetPosition(units::angle::degree_t degree) override;

    //configuration setter for inversion
   void SetInverted(bool inversion) override;

    //configuration setter for neutral move
   void SetNeutralMode(NeutralMode neutralMode) override;

    //configuration setter for enabling current limit
   void EnableCurrentLimit(bool enable) override;

    //configuration setter for current limit value
   void SetCurrentLimit(int current) override;

    //configuration setter for enabling the forward soft limit
   void EnableForwardSoftLimit(bool enable) override;

    //configuration setter for enabling the reverse soft limit
   void EnableReverseSoftLimit(bool enable) override;

    //configuration setter for setting the forward limit position
   void SetForwardSoftLimit(units::angle::degree_t degrees) override;

    //configuration setter for setting the reverse limit position
   void SetReverseSoftLimit(units::angle::degree_t degrees) override;

    //apply the configuration of values
   void ApplyConfiguration(MotorConfig motorConfig) override;

    //virtual function to set the PIDs
   void SetPID(double p, double i, double d, double ff) override;

    //virtual function to recieve if the forward limit is enabled;
   bool IsForwardLimitEnabled() override;

    //virtual function to recieve if the reverse limit is enabled;
   bool IsReverseLimitEnabled() override;

   void SetConversionFactor(double conversion) override;

    //virtual function to pull the current position of the motor
    //(prioritizes absolute)
   units::angle::degree_t GetPosition() override;

    //getter to return the velocity of the current motor controller
    //(Prioritizes absolute)
    //TODO give it a better type!!
   double GetVelocity() override;
};
