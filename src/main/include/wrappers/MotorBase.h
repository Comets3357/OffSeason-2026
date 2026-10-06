#pragma once

//support both the 2026 and SystemCore unit header names.
#if __has_include(<units/angle.h>)
#include <units/angular_velocity.h>
#include <units/angle.h>
#else
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/angle.hpp>
namespace units = wpi::units;
#endif

class MotorBase {

public:

    //default destructor.
    virtual ~MotorBase() = default;

    //enum containing which neutr;al modes motor controllers are allowed to take.
    enum class NeutralMode {
        Coast,
        Brake
    };

    //main struct containing all configuration values
    struct MotorConfig{
        bool inverted = false;
        NeutralMode neutralMode = NeutralMode::Brake;

        bool enableCurrentLimit = true;
        int currentLimitAmps = 40;

        bool enableVoltageCompensation = true;
        double voltageCompensation = 12.0;

        // Output rotations per motor rotation (1.0 for direct drive).
        double positionConversionFactor = 1.0;
        double velocityConversionFactor = 1.0;

        bool enableForwardSoftLimit = false;
        bool enableReverseSoftLimit = false;

        units::angle::degree_t forwardSoftLimit {0.0};
        units::angle::degree_t reverseSoftLimit {0.0};


    };

    //function to simulate the motor controller every processing tick.
    virtual void SimulationPeriodic() = 0;

    //basic function for setting speed before anything special.
    virtual void Set(double speed) = 0;

    //more complicated than Set, but just setting motor voltage.
    virtual void SetVoltage(double voltage) = 0;

    //stop the motor, overrides all previous commands and stop the motor
    virtual void Stop() = 0;

    //multiply the max output of the motor.
    virtual void SetMaxOutput( double percent) = 0;

    //sets the motor to go a specific velocity.
    virtual void SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm) = 0;

    //sets the motor to a specific position.
    virtual void SetPosition(units::angle::degree_t degree) = 0;

    //sets the motor to go a specific velocity using a specific slot
    virtual void SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm, int slot) = 0;

    //sets the motor to a specific position using a specific slot.
    virtual void SetPosition(units::angle::degree_t degree, int slot) = 0;

    //setter to set the pid of a specific slot.
    virtual void SetPID(double p, double i, double d, double ff, int slot) = 0;

    //configuration setter for inversion
    virtual void SetInverted(bool inversion) = 0;

    //configuration setter for neutral move
    virtual void SetNeutralMode(NeutralMode neutralMode) = 0;

    //setter to change how much the motor controller changes position whenever it rotates.
    virtual void SetConversionFactor(double value) = 0;

    //enable closed-loop position wrapping from -180 to 180 degrees.
    virtual void SetWrapping() = 0;

    //configuration setter for enabling current limit
    virtual void EnableCurrentLimit(bool enable) = 0;

    //configuration setter for current limit value
    virtual void SetCurrentLimit(int current) = 0;

    //configuration setter for enabling the forward soft limit
    virtual void EnableForwardSoftLimit(bool enable) = 0;

    //configuration setter for enabling the reverse soft limit
    virtual void EnableReverseSoftLimit(bool enable) = 0;

    //configuration setter for setting the forward limit position
    virtual void SetForwardSoftLimit(units::angle::degree_t degrees) = 0;

    //configuration setter for setting the reverse limit position
    virtual void SetReverseSoftLimit(units::angle::degree_t degrees) = 0;

    //apply the configuration of values
    virtual void ApplyConfiguration(MotorConfig motorConfig) = 0;

    //getter to recieve the position of the current motorController
    virtual units::angle::degree_t getPosition() = 0;

    //getter to recieve the velocity of the current motorController
    virtual units::angular_velocity::revolutions_per_minute_t getVelocity() = 0;

    //getter to recieve if the forward soft limit on the motorController is enabled.
    virtual bool isForwardSoftLimitEnabled() = 0;

    //getter to recieve if the reverse soft limit on the motorController is enabled.
    virtual bool isReverseSoftLimitEnabled() = 0;

    //getter to recieve the specific degree the forwardLimit is.
    virtual units::angle::degree_t getForwardLimit() = 0;

    //getter to recieve the specific degree the reverseLimit is.
    virtual units::angle::degree_t getReverseLimit() = 0;

    //virtual function to set the PIDs
    virtual void SetPID(double p, double i, double d, double ff) { SetPID(p, i, d, ff, 0); }

    //virtual function to recieve if the forward limit is enabled;
    virtual bool IsForwardLimitEnabled() { return isForwardSoftLimitEnabled(); }

    //virtual function to recieve if the reverse limit is enabled;
    virtual bool IsReverseLimitEnabled() { return isReverseSoftLimitEnabled(); }


    //virtual function to pull the current position of the motor
    virtual units::angle::degree_t GetPosition() { return getPosition(); }

    //getter to return the velocity of the current motor controller
    //TODO give it a better type!!
    virtual double GetVelocity() { return getVelocity().value(); }



};
