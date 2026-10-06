#pragma once

#include <thrifty/nova/Nova.h>

#include "wrappers/MotorBase.h"

//requires the ThriftyLib C++ dependency; see docs/NovaBase.md for setup.
class NovaMotor : public MotorBase
{
public:
    //constructor for a brushless motor using its built-in relative encoder.
    explicit NovaMotor(int id, thrifty::Motor::MotorType motorType = thrifty::Motor::NEO,
                       int busId = 0);

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
    //slots are 0 and 1; gains use Nova native rotations and rotations per second.
    //ff is Nova's native F gain, not REV's kV gain.
    void SetPID(double p, double i, double d, double ff, int slot) override;

    //sets an absolute encoder position using voltage-based PID, in sensor degrees.
    //configure the connected encoder type/direction in Thrifty Config first.
    void SetAbsolutePosition(units::angle::degree_t degree, int slot = 0);

    //gets the absolute encoder position in sensor degrees, without gear conversion.
    units::angle::degree_t getAbsolutePosition();

    //enables the Nova's onboard shortest-path wrapping for absolute position.
    void SetAbsoluteWrapping(bool enable);

    //configuration setter for inversion
    void SetInverted(bool inversion) override;

    //configuration setter for neutral mode
    void SetNeutralMode(NeutralMode neutralMode) override;

    //setter to change how much the motor controller changes position whenever it rotates.
    //use output rotations per motor rotation; 0.1 means a 10:1 reduction.
    void SetConversionFactor(double value) override;

    //choose the nearest equivalent angle when sending a position command.
    //call SetPosition each tick to update the shortest path; readings stay continuous.
    void SetWrapping() override;

    //configuration setter for enabling current limit
    //disabling restores Nova's default 40 amp stator limit; supply protection stays active.
    void EnableCurrentLimit(bool enable) override;

    //configuration setter for current limit value
    void SetCurrentLimit(int current) override;

    //configuration setter for enabling both soft limits; Nova has one shared switch.
    void EnableForwardSoftLimit(bool enable) override;

    //configuration setter for enabling both soft limits; Nova has one shared switch.
    void EnableReverseSoftLimit(bool enable) override;

    //configuration setter for setting the forward limit position
    void SetForwardSoftLimit(units::angle::degree_t degrees) override;

    //configuration setter for setting the reverse limit position
    void SetReverseSoftLimit(units::angle::degree_t degrees) override;

    //apply the configuration of values
    //forward and reverse enable flags must match; otherwise throws invalid_argument.
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
    thrifty::Nova motor;
    MotorConfig config;
    double maxOutput = 1.0;
    bool wrappingEnabled = false;
    thrifty::Motor::FeedbackSensorType feedbackSensor = thrifty::Motor::FeedbackSensorType::INTERNAL;

    //change the feedback source only when switching control modes.
    void SelectFeedback(thrifty::Motor::FeedbackSensorType sensor);

    //apply the stored settings to the motor controller.
    void Configure();
};
