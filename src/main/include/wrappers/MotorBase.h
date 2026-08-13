#include "units/angular_velocity.h"
#include "units/angle.h"

class MotorBase {

public:

    //default destructor.
    ~MotorBase() = default;

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

        double positionConversionFactor = 1.0;
        double velocityConversionFactor = 1.0;

        bool enableForwardSoftLimit = false;
        bool enableReverseSoftLimit = false;

        units::angle::degree_t forwardSoftLimit = 0.0_deg;
        units::angle::degree_t reverseSoftLimit = 0.0_deg;


    };

    //basic function for setting speed before anything special.
    virtual void Set(double speed);

    //more complicated than Set, but just setting motor voltage.
    virtual void SetVoltage(double voltage);

    //stop the motor, overrides all previous commands and stop the motor
    virtual void Stop();

    //sets the motor to go a specific velocity.
    virtual void SetVelocity(units::angular_velocity::revolutions_per_minute rpm);

    //sets the motor to a specific position.
    virtual void SetPosition(units::angle::degree_t degree);

    //configuration setter for inversion
    virtual void SetInverted(bool inversion);

    //configuration setter for neutral move
    virtual void SetNeutralMode(NeutralMode neutralMode);

    //configuration setter for enabling current limit
    virtual void EnableCurrentLimit(bool enable);

    //configuration setter for current limit value
    virtual void SetCurrentLimit(int current);

    //configuration setter for enabling the forward soft limit
    virtual void EnableForwardSoftLimit(bool enable);

    //configuration setter for enabling the reverse soft limit
    virtual void EnableReverseSoftLimit(bool enable);

    //configuration setter for setting the forward limit position
    virtual void SetForwardSoftLimit(units::angle::degree_t degrees);

    //configuration setter for setting the reverse limit position
    virtual void SetReverseSoftLimit(units::angle::degree_t degrees);

    //apply the configuration of values
    virtual void ApplyConfiguration(MotorConfig motorConfig);

};