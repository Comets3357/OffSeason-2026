#pragma once

#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkBaseConfig.h>

#include "wrappers/MotorBase.h"

class SparkMaxMotor : public MotorBase
{
public:
    //constructor for a brushless motor using its built-in relative encoder.
    explicit SparkMaxMotor(int id);

    //function to simulate the motor controller every processing tick.
    //simulation hook; no physics model is included.
    void SimulationPeriodic() override;

    //basic function for setting speed before anything special.
    void Set(double speed) override;

    //more complicated than Set, but just setting motor voltage.
    void SetVoltage(double voltage) override;

    //stop the motor, overrides all previous commands and stop the motor
    void Stop() override;

    //multiply the max output of the motor.
    //use a value from 0 to 1 for duty cycle, voltage, and all pid slots.
    void SetMaxOutput(double percent) override;

    //sets the motor to go a specific velocity.
    //uses pid slot 0.
    void SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm) override;

    //sets the motor to a specific position.
    //uses pid slot 0.
    void SetPosition(units::angle::degree_t degree) override;

    //sets the motor to go a specific velocity using a specific slot
    void SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm, int slot) override;

    //sets the motor to a specific position using a specific slot.
    void SetPosition(units::angle::degree_t degree, int slot) override;

    //setter to set the pid of a specific slot.
    //slots are 0 through 3; ff is kV in volts per RPM.
    void SetPID(double p, double i, double d, double ff, int slot) override;

    //configuration setter for inversion
    void SetInverted(bool inversion) override;

    //configuration setter for neutral mode
    void SetNeutralMode(NeutralMode neutralMode) override;

    //setter to change how much the motor controller changes position whenever it rotates.
    //use output rotations per motor rotation; 0.1 means a 10:1 reduction.
    void SetConversionFactor(double value) override;

    //enable closed-loop position wrapping from -180 to 180 degrees.
    //encoder position readings remain continuous.
    void SetWrapping() override;

    //configuration setter for enabling current limit
    //disabling restores the 80 amp smart limit; controller protections stay active.
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

    //getter to receive the position of the current motorController
    units::angle::degree_t getPosition() override;

    //getter to receive the velocity of the current motorController
    units::angular_velocity::revolutions_per_minute_t getVelocity() override;

    //getter to receive if the forward soft limit on the motorController is enabled.
    bool isForwardSoftLimitEnabled() override;

    //getter to receive if the reverse soft limit on the motorController is enabled.
    bool isReverseSoftLimitEnabled() override;

    //getter to receive the specific degree the forwardLimit is.
    units::angle::degree_t getForwardLimit() override;

    //getter to receive the specific degree the reverseLimit is.
    units::angle::degree_t getReverseLimit() override;

private:
    rev::spark::SparkMax motor;
    rev::spark::SparkRelativeEncoder encoder = motor.GetEncoder();
    rev::spark::SparkClosedLoopController closedLoopController = motor.GetClosedLoopController();
    rev::spark::SparkBaseConfig config;
    double maxOutput = 1.0;
    int currentLimitAmps = 40;
    bool currentLimitEnabled = true;

    //apply settings immediately without writing them to flash.
    void Configure();
};

class SparkFlexMotor : public MotorBase
{
public:
    //constructor for a brushless motor using its built-in relative encoder.
    explicit SparkFlexMotor(int id);

    //function to simulate the motor controller every processing tick.
    //simulation hook; no physics model is included.
    void SimulationPeriodic() override;

    //basic function for setting speed before anything special.
    void Set(double speed) override;

    //more complicated than Set, but just setting motor voltage.
    void SetVoltage(double voltage) override;

    //stop the motor, overrides all previous commands and stop the motor
    void Stop() override;

    //multiply the max output of the motor.
    //use a value from 0 to 1 for duty cycle, voltage, and all pid slots.
    void SetMaxOutput(double percent) override;

    //sets the motor to go a specific velocity.
    //uses pid slot 0.
    void SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm) override;

    //sets the motor to a specific position.
    //uses pid slot 0.
    void SetPosition(units::angle::degree_t degree) override;

    //sets the motor to go a specific velocity using a specific slot
    void SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm, int slot) override;

    //sets the motor to a specific position using a specific slot.
    void SetPosition(units::angle::degree_t degree, int slot) override;

    //setter to set the pid of a specific slot.
    //slots are 0 through 3; ff is kV in volts per RPM.
    void SetPID(double p, double i, double d, double ff, int slot) override;

    //configuration setter for inversion
    void SetInverted(bool inversion) override;

    //configuration setter for neutral mode
    void SetNeutralMode(NeutralMode neutralMode) override;

    //setter to change how much the motor controller changes position whenever it rotates.
    //use output rotations per motor rotation; 0.1 means a 10:1 reduction.
    void SetConversionFactor(double value) override;

    //enable closed-loop position wrapping from -180 to 180 degrees.
    //encoder position readings remain continuous.
    void SetWrapping() override;

    //configuration setter for enabling current limit
    //disabling restores the 80 amp smart limit; controller protections stay active.
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

    //getter to receive the position of the current motorController
    units::angle::degree_t getPosition() override;

    //getter to receive the velocity of the current motorController
    units::angular_velocity::revolutions_per_minute_t getVelocity() override;

    //getter to receive if the forward soft limit on the motorController is enabled.
    bool isForwardSoftLimitEnabled() override;

    //getter to receive if the reverse soft limit on the motorController is enabled.
    bool isReverseSoftLimitEnabled() override;

    //getter to receive the specific degree the forwardLimit is.
    units::angle::degree_t getForwardLimit() override;

    //getter to receive the specific degree the reverseLimit is.
    units::angle::degree_t getReverseLimit() override;

private:
    rev::spark::SparkFlex motor;
    rev::spark::SparkRelativeEncoder encoder = motor.GetEncoder();
    rev::spark::SparkClosedLoopController closedLoopController = motor.GetClosedLoopController();
    rev::spark::SparkBaseConfig config;
    double maxOutput = 1.0;
    int currentLimitAmps = 40;
    bool currentLimitEnabled = true;

    //apply settings immediately without writing them to flash.
    void Configure();
};
