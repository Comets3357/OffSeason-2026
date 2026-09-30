//The 2026 Java-only vendordep cannot build this wrapper.
//Installing the ThriftyLib C++ dependency automatically enables this file.
#if __has_include(<thrifty/nova/Nova.h>)

#include "wrappers/NovaBase.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thrifty/nova/NovaConfig.h>
#include <thrifty/nova/NovaControl.h>

NovaMotor::NovaMotor(int id, thrifty::Motor::MotorType motorType, int busId)
    : motor{busId, id, motorType}
{
    Configure();
    SetMaxOutput(1.0);
    SetPID(0, 0, 0, 0, 0);
    SetPID(0, 0, 0, 0, 1);
}

void NovaMotor::SimulationPeriodic()
{
    //A mechanism-specific physics model belongs in the subsystem.
}

void NovaMotor::Set(double speed)
{
    motor.Control(thrifty::NovaControl::Percent(std::clamp(speed, -1.0, 1.0) * maxOutput));
}

void NovaMotor::SetVoltage(double voltage)
{
    motor.Control(thrifty::NovaControl::Voltage(voltage * maxOutput));
}

void NovaMotor::Stop()
{
    motor.Stop();
}

void NovaMotor::SetMaxOutput(double percent)
{
    maxOutput = std::clamp(percent, 0.0, 1.0);
    motor.Configure(thrifty::NovaConfig::MaxForward(maxOutput),
                    thrifty::NovaConfig::MaxReverse(maxOutput));
}

void NovaMotor::SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm)
{
    SetVelocity(rpm, 0);
}

void NovaMotor::SetPosition(units::angle::degree_t degree)
{
    SetPosition(degree, 0);
}

void NovaMotor::SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm, int slot)
{
    if (slot < 0 || slot > 1) return;
    //Nova expects motor rotations per second.
    double motorRps = rpm.value() / (60.0 * config.velocityConversionFactor);
    motor.Control(thrifty::NovaControl::Velocity(motorRps),
                  thrifty::NovaControl::WithPIDSlot(static_cast<thrifty::NovaControl::PIDSlot>(slot)));
}

void NovaMotor::SetPosition(units::angle::degree_t degree, int slot)
{
    if (slot < 0 || slot > 1) return;
    double target = degree.value();
    if (wrappingEnabled)
    {
        double current = getPosition().value();
        target = current + std::remainder(target - current, 360.0);
    }
    double motorRotations = target / (360.0 * config.positionConversionFactor);
    motor.Control(thrifty::NovaControl::Position(motorRotations),
                  thrifty::NovaControl::WithPIDSlot(static_cast<thrifty::NovaControl::PIDSlot>(slot)));
}

void NovaMotor::SetPID(double p, double i, double d, double ff, int slot)
{
    if (slot == 0)
    {
        motor.Configure(thrifty::NovaConfig::Pid0::P(p), thrifty::NovaConfig::Pid0::I(i),
                        thrifty::NovaConfig::Pid0::D(d), thrifty::NovaConfig::Pid0::F(ff));
    }
    else if (slot == 1)
    {
        motor.Configure(thrifty::NovaConfig::Pid1::P(p), thrifty::NovaConfig::Pid1::I(i),
                        thrifty::NovaConfig::Pid1::D(d), thrifty::NovaConfig::Pid1::F(ff));
    }
}

void NovaMotor::SetAbsolutePosition(units::angle::degree_t degree, int slot)
{
    if (slot < 0 || slot > 1) return;
    motor.Control(thrifty::NovaControl::VoltageAbsPosition(degree.value() / 360.0),
                  thrifty::NovaControl::WithPIDSlot(static_cast<thrifty::NovaControl::PIDSlot>(slot)));
}

units::angle::degree_t NovaMotor::getAbsolutePosition()
{
    return units::angle::degree_t{motor.Status().GetPositionAbs() * 360.0};
}

void NovaMotor::SetAbsoluteWrapping(bool enable)
{
    motor.Configure(thrifty::NovaConfig::AbsWrapping(enable));
}

void NovaMotor::SetInverted(bool inversion)
{
    config.inverted = inversion;
    motor.Configure(thrifty::NovaConfig::Inverted(inversion));
}

void NovaMotor::SetNeutralMode(NeutralMode neutralMode)
{
    config.neutralMode = neutralMode;
    motor.Configure(thrifty::NovaConfig::BrakeMode(neutralMode == NeutralMode::Brake));
}

void NovaMotor::SetConversionFactor(double value)
{
    if (!std::isfinite(value) || value <= 0) return;
    config.positionConversionFactor = value;
    config.velocityConversionFactor = value;
    //Keep the soft limits in mechanism degrees when changing the gear ratio.
    Configure();
}

void NovaMotor::SetWrapping()
{
    //Nova's native wrapping only supports absolute encoders.
    wrappingEnabled = true;
}

void NovaMotor::EnableCurrentLimit(bool enable)
{
    config.enableCurrentLimit = enable;
    motor.Configure(thrifty::NovaConfig::StatorCurrent(enable ? config.currentLimitAmps : 40));
}

void NovaMotor::SetCurrentLimit(int current)
{
    if (current <= 0) return;
    config.currentLimitAmps = current;
    EnableCurrentLimit(config.enableCurrentLimit);
}

void NovaMotor::EnableForwardSoftLimit(bool enable)
{
    //Nova has a single enable setting for both directions.
    config.enableForwardSoftLimit = enable;
    config.enableReverseSoftLimit = enable;
    motor.Configure(thrifty::NovaConfig::SoftLimitEnable(enable));
}

void NovaMotor::EnableReverseSoftLimit(bool enable)
{
    EnableForwardSoftLimit(enable);
}

void NovaMotor::SetForwardSoftLimit(units::angle::degree_t degrees)
{
    config.forwardSoftLimit = degrees;
    motor.Configure(thrifty::NovaConfig::SoftLimitForward(
        degrees.value() / (360.0 * config.positionConversionFactor)));
}

void NovaMotor::SetReverseSoftLimit(units::angle::degree_t degrees)
{
    config.reverseSoftLimit = degrees;
    motor.Configure(thrifty::NovaConfig::SoftLimitReverse(
        degrees.value() / (360.0 * config.positionConversionFactor)));
}

void NovaMotor::ApplyConfiguration(MotorConfig motorConfig)
{
    if (motorConfig.enableForwardSoftLimit != motorConfig.enableReverseSoftLimit)
        throw std::invalid_argument("Nova requires both soft-limit enable flags to match");
    if (!std::isfinite(motorConfig.positionConversionFactor) || motorConfig.positionConversionFactor <= 0 ||
        !std::isfinite(motorConfig.velocityConversionFactor) || motorConfig.velocityConversionFactor <= 0)
        throw std::invalid_argument("Nova conversion factors must be finite and positive");
    if (motorConfig.currentLimitAmps <= 0)
        throw std::invalid_argument("Nova current limit must be positive");
    if (motorConfig.enableVoltageCompensation &&
        (!std::isfinite(motorConfig.voltageCompensation) || motorConfig.voltageCompensation <= 0))
        throw std::invalid_argument("Nova voltage compensation must be finite and positive");
    config = motorConfig;
    Configure();
}

void NovaMotor::Configure()
{
    motor.Configure(
        thrifty::NovaConfig::Inverted(config.inverted),
        thrifty::NovaConfig::BrakeMode(config.neutralMode == NeutralMode::Brake),
        thrifty::NovaConfig::StatorCurrent(config.enableCurrentLimit ? config.currentLimitAmps : 40),
        thrifty::NovaConfig::VoltageComp(config.enableVoltageCompensation ? config.voltageCompensation : 0),
        thrifty::NovaConfig::SoftLimitForward(config.forwardSoftLimit.value() /
                                            (360.0 * config.positionConversionFactor)),
        thrifty::NovaConfig::SoftLimitReverse(config.reverseSoftLimit.value() /
                                            (360.0 * config.positionConversionFactor)),
        thrifty::NovaConfig::SoftLimitEnable(config.enableForwardSoftLimit));
}

units::angle::degree_t NovaMotor::getPosition()
{
    return units::angle::degree_t{motor.Status().GetPositionInternal() *
                                 360.0 * config.positionConversionFactor};
}

units::angular_velocity::revolutions_per_minute_t NovaMotor::getVelocity()
{
    return units::angular_velocity::revolutions_per_minute_t{
        motor.Status().GetVelocityInternal() * 60.0 * config.velocityConversionFactor};
}

bool NovaMotor::isForwardSoftLimitEnabled()
{
    return motor.Status().GetSoftLimitEnable();
}

bool NovaMotor::isReverseSoftLimitEnabled()
{
    return motor.Status().GetSoftLimitEnable();
}

units::angle::degree_t NovaMotor::getForwardLimit()
{
    return units::angle::degree_t{motor.Status().GetSoftLimitForward() *
                                 360.0 * config.positionConversionFactor};
}

units::angle::degree_t NovaMotor::getReverseLimit()
{
    return units::angle::degree_t{motor.Status().GetSoftLimitReverse() *
                                 360.0 * config.positionConversionFactor};
}

#endif
