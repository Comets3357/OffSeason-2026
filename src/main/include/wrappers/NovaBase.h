#pragma once

#include <thrifty/nova/Nova.h>
#include <thrifty/canEncoder/CanEncoder.h>
#include <memory>
#include <optional>

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

    //set the internal encoder's current reading in output degrees, accounting for gearing.
    //does not send a movement command or change the selected feedback sensor.
    void SetInternalEncoderPosition(units::angle::degree_t degree);

    //setter to set the pid of a specific slot.
    //slots are 0 and 1; gains use Nova native rotations and rotations per second.
    //ff is Nova's native F gain, not REV's kV gain.
    void SetPID(double p, double i, double d, double ff, int slot) override;

    //sets an absolute encoder position using voltage-based PID, in output degrees.
    //configure the connected encoder type/direction in Thrifty Config first.
    void SetAbsolutePosition(units::angle::degree_t degree, int slot = 0);

    //tune onboard motion profiling; velocity is rotations/sec, acceleration rotations/sec^2.
    //kV and kA use those units; kS and kG are volts. Uses the matching PID slot.
    void SetMotionProfile(double maxVelocity, double maxAcceleration,
                          double kV, double kA, double kS, double kG,
                          thrifty::Motor::MPGravityType gravityType, int slot = 0);

    //profile an absolute position in degrees, using the paired CAN encoder if present.
    //requires a 1:1 absolute encoder on the mechanism (conversion factor 1).
    void SetAbsoluteMotionProfilePosition(units::angle::degree_t degree, int slot = 0);

    //gets the absolute encoder position in output degrees.
    units::angle::degree_t getAbsolutePosition();

    //enables the Nova's onboard shortest-path wrapping for absolute position.
    void SetAbsoluteWrapping(bool enable);

    //use a Thrifty CAN encoder on this motor's bus for absolute control/readings.
    //IDs are 0-62; kNoCanEncoder returns to the wired absolute encoder.
    //requires Nova firmware supporting CAN_ENCODER (2027.0.0-beta.4 or newer).
    void SetExternalEncoder(int encoderId);

    //output rotations per absolute encoder rotation; defaults to 1.
    void SetAbsoluteConversionFactor(double value);
    void SetAbsoluteEncoderInverted(bool inverted);
    units::angular_velocity::revolutions_per_minute_t getAbsoluteVelocity();

    //choose the encoder used for both soft limits, separately from PID feedback.
    void SetSoftLimitSource(thrifty::Motor::FeedbackSensorType sensor);

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

    //enable voltage compensation and store the nominal voltage.
    void SetVoltageCompensation(double voltage);

    //follow a leader's CAN ID on the same bus; use SetInverted for direction.
    void SetFollow(int leaderCanId);

    //read motor stator current in amps.
    double GetOutputCurrent();
    double GetAppliedVoltage();

    //configuration setter for enabling both soft limits; Nova has one shared switch.
    void EnableForwardSoftLimit(bool enable) override;

    //configuration setter for enabling both soft limits; Nova has one shared switch.
    void EnableReverseSoftLimit(bool enable) override;

    //configuration setter for setting the forward limit in output rotations
    void SetForwardSoftLimit(units::angle::turn_t rotations) override;

    //configuration setter for setting the reverse limit in output rotations
    void SetReverseSoftLimit(units::angle::turn_t rotations) override;

    //reapply the internal configuration updated by the setters.
    void ApplyConfiguration();

    //replace and apply the configuration of values.
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
    int canBusId;
    std::unique_ptr<thrifty::CanEncoder> externalEncoder;
    std::optional<int> externalEncoderId;
    std::optional<bool> absoluteEncoderInverted;
    bool followingMotor = false;
    MotorConfig config;
    double maxOutput = 1.0;
    double absoluteConversionFactor = 1.0;
    thrifty::Motor::FeedbackSensorType softLimitSensor = thrifty::Motor::FeedbackSensorType::INTERNAL;
    double SoftLimitConversionFactor() const;
    bool wrappingEnabled = false;
    thrifty::Motor::FeedbackSensorType feedbackSensor = thrifty::Motor::FeedbackSensorType::INTERNAL;

    //change the feedback source only when switching control modes.
    void SelectFeedback(thrifty::Motor::FeedbackSensorType sensor);

    //apply the stored settings to the motor controller.
    void Configure();
};
