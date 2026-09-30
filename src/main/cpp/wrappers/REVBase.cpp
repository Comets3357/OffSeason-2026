#include "wrappers/REVBase.h"

#include <algorithm>
#include <cmath>
#include <units/voltage.h>

SparkMaxMotor::SparkMaxMotor(int id)
    : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless}
{
    config.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder);
    for (int slot = 0; slot < 4; ++slot)
    {
        auto pidSlot = static_cast<rev::spark::ClosedLoopSlot>(slot);
        config.closedLoop.Pid(0, 0, 0, pidSlot);
        config.closedLoop.feedForward.kV(0, pidSlot);
        config.closedLoop.OutputRange(-1, 1, pidSlot);
    }
    config.closedLoop.PositionWrappingEnabled(false);
    ApplyConfiguration(MotorConfig{});
}

void SparkMaxMotor::Configure()
{
    motor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                    rev::PersistMode::kNoPersistParameters);
}

void SparkMaxMotor::SimulationPeriodic()
{
    // A mechanism-specific physics model belongs in the subsystem.
}

void SparkMaxMotor::Set(double speed)
{
    motor.Set(std::clamp(speed, -1.0, 1.0) * maxOutput);
}

void SparkMaxMotor::SetVoltage(double voltage)
{
    motor.SetVoltage(units::volt_t{voltage * maxOutput});
}

void SparkMaxMotor::Stop()
{
    motor.StopMotor();
}

void SparkMaxMotor::SetMaxOutput(double percent)
{
    maxOutput = std::clamp(percent, 0.0, 1.0);
    for (int slot = 0; slot < 4; ++slot)
    {
        config.closedLoop.OutputRange(-maxOutput, maxOutput,
                                      static_cast<rev::spark::ClosedLoopSlot>(slot));
    }
    Configure();
}

void SparkMaxMotor::SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm)
{
    SetVelocity(rpm, 0);
}

void SparkMaxMotor::SetPosition(units::angle::degree_t degree)
{
    SetPosition(degree, 0);
}

void SparkMaxMotor::SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm, int slot)
{
    if (slot < 0 || slot > 3) return;
    closedLoopController.SetSetpoint(rpm.value(), rev::spark::SparkLowLevel::ControlType::kVelocity,
                                     static_cast<rev::spark::ClosedLoopSlot>(slot));
}

void SparkMaxMotor::SetPosition(units::angle::degree_t degree, int slot)
{
    if (slot < 0 || slot > 3) return;
    closedLoopController.SetSetpoint(degree.value(), rev::spark::SparkLowLevel::ControlType::kPosition,
                                     static_cast<rev::spark::ClosedLoopSlot>(slot));
}

void SparkMaxMotor::SetPID(double p, double i, double d, double ff, int slot)
{
    if (slot < 0 || slot > 3) return;
    auto pidSlot = static_cast<rev::spark::ClosedLoopSlot>(slot);
    config.closedLoop.Pid(p, i, d, pidSlot);
    config.closedLoop.feedForward.kV(ff, pidSlot);
    Configure();
}

void SparkMaxMotor::SetInverted(bool inversion)
{
    config.Inverted(inversion);
    Configure();
}

void SparkMaxMotor::SetNeutralMode(NeutralMode neutralMode)
{
    config.SetIdleMode(neutralMode == NeutralMode::Brake
        ? rev::spark::SparkBaseConfig::IdleMode::kBrake
        : rev::spark::SparkBaseConfig::IdleMode::kCoast);
    Configure();
}

void SparkMaxMotor::SetConversionFactor(double value)
{
    if (!std::isfinite(value) || value <= 0) return;
    config.encoder.PositionConversionFactor(360.0 * value);
    config.encoder.VelocityConversionFactor(value);
    Configure();
}

void SparkMaxMotor::SetWrapping()
{
    config.closedLoop.PositionWrappingEnabled(true);
    config.closedLoop.PositionWrappingMinInput(-180.0);
    config.closedLoop.PositionWrappingMaxInput(180.0);
    Configure();
}

void SparkMaxMotor::EnableCurrentLimit(bool enable)
{
    currentLimitEnabled = enable;
    config.SmartCurrentLimit(enable ? currentLimitAmps : 80);
    Configure();
}

void SparkMaxMotor::SetCurrentLimit(int current)
{
    if (current <= 0) return;
    currentLimitAmps = current;
    config.SmartCurrentLimit(currentLimitEnabled ? currentLimitAmps : 80);
    Configure();
}

void SparkMaxMotor::EnableForwardSoftLimit(bool enable)
{
    config.softLimit.ForwardSoftLimitEnabled(enable);
    Configure();
}

void SparkMaxMotor::EnableReverseSoftLimit(bool enable)
{
    config.softLimit.ReverseSoftLimitEnabled(enable);
    Configure();
}

void SparkMaxMotor::SetForwardSoftLimit(units::angle::degree_t degrees)
{
    config.softLimit.ForwardSoftLimit(degrees.value());
    Configure();
}

void SparkMaxMotor::SetReverseSoftLimit(units::angle::degree_t degrees)
{
    config.softLimit.ReverseSoftLimit(degrees.value());
    Configure();
}

void SparkMaxMotor::ApplyConfiguration(MotorConfig motorConfig)
{
    //applies the neutral mode for the motor controller
    config.Inverted(motorConfig.inverted);

    //motors are set to neutral on default
    config.SetIdleMode(motorConfig.neutralMode == NeutralMode::Brake
        ? rev::spark::SparkBaseConfig::IdleMode::kBrake
        : rev::spark::SparkBaseConfig::IdleMode::kCoast);

    //if the current limits are enabled, set it to the given value
    currentLimitEnabled = motorConfig.enableCurrentLimit;
    if (motorConfig.currentLimitAmps > 0)
        currentLimitAmps = motorConfig.currentLimitAmps;
    config.SmartCurrentLimit(currentLimitEnabled ? currentLimitAmps : 80);

    //if voltage compensation is enabled, set it to the stated value. If not, disable
    if (motorConfig.enableVoltageCompensation)
        config.VoltageCompensation(motorConfig.voltageCompensation);
    else
        config.DisableVoltageCompensation();
    //applies the conversion factors to the encoders
    if (std::isfinite(motorConfig.positionConversionFactor) && motorConfig.positionConversionFactor > 0)
        config.encoder.PositionConversionFactor(360.0 * motorConfig.positionConversionFactor);
    if (std::isfinite(motorConfig.velocityConversionFactor) && motorConfig.velocityConversionFactor > 0)
        config.encoder.VelocityConversionFactor(motorConfig.velocityConversionFactor);
    
    //sets all the limits
    config.softLimit.ForwardSoftLimitEnabled(motorConfig.enableForwardSoftLimit);
    config.softLimit.ReverseSoftLimitEnabled(motorConfig.enableReverseSoftLimit);
    config.softLimit.ForwardSoftLimit(motorConfig.forwardSoftLimit.value());
    config.softLimit.ReverseSoftLimit(motorConfig.reverseSoftLimit.value());
    Configure();
}

units::angle::degree_t SparkMaxMotor::getPosition()
{
    return units::angle::degree_t{encoder.GetPosition()};
}

units::angular_velocity::revolutions_per_minute_t SparkMaxMotor::getVelocity()
{
    return units::angular_velocity::revolutions_per_minute_t{encoder.GetVelocity()};
}

bool SparkMaxMotor::isForwardSoftLimitEnabled()
{
    return motor.configAccessor.softLimit.GetForwardSoftLimitEnabled();
}

bool SparkMaxMotor::isReverseSoftLimitEnabled()
{
    return motor.configAccessor.softLimit.GetReverseSoftLimitEnabled();
}

units::angle::degree_t SparkMaxMotor::getForwardLimit()
{
    return units::angle::degree_t{motor.configAccessor.softLimit.GetForwardSoftLimit()};
}

units::angle::degree_t SparkMaxMotor::getReverseLimit()
{
    return units::angle::degree_t{motor.configAccessor.softLimit.GetReverseSoftLimit()};
}

SparkFlexMotor::SparkFlexMotor(int id)
    : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless}
{
    config.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder);
    for (int slot = 0; slot < 4; ++slot)
    {
        auto pidSlot = static_cast<rev::spark::ClosedLoopSlot>(slot);
        config.closedLoop.Pid(0, 0, 0, pidSlot);
        config.closedLoop.feedForward.kV(0, pidSlot);
        config.closedLoop.OutputRange(-1, 1, pidSlot);
    }
    config.closedLoop.PositionWrappingEnabled(false);
    ApplyConfiguration(MotorConfig{});
}

void SparkFlexMotor::Configure()
{
    motor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                    rev::PersistMode::kNoPersistParameters);
}

void SparkFlexMotor::SimulationPeriodic()
{
    // A mechanism-specific physics model belongs in the subsystem.
}

void SparkFlexMotor::Set(double speed)
{
    motor.Set(std::clamp(speed, -1.0, 1.0) * maxOutput);
}

void SparkFlexMotor::SetVoltage(double voltage)
{
    motor.SetVoltage(units::volt_t{voltage * maxOutput});
}

void SparkFlexMotor::Stop()
{
    motor.StopMotor();
}

void SparkFlexMotor::SetMaxOutput(double percent)
{
    maxOutput = std::clamp(percent, 0.0, 1.0);
    for (int slot = 0; slot < 4; ++slot)
    {
        config.closedLoop.OutputRange(-maxOutput, maxOutput,
                                      static_cast<rev::spark::ClosedLoopSlot>(slot));
    }
    Configure();
}

void SparkFlexMotor::SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm)
{
    SetVelocity(rpm, 0);
}

void SparkFlexMotor::SetPosition(units::angle::degree_t degree)
{
    SetPosition(degree, 0);
}

void SparkFlexMotor::SetVelocity(units::angular_velocity::revolutions_per_minute_t rpm, int slot)
{
    if (slot < 0 || slot > 3) return;
    closedLoopController.SetSetpoint(rpm.value(), rev::spark::SparkLowLevel::ControlType::kVelocity,
                                     static_cast<rev::spark::ClosedLoopSlot>(slot));
}

void SparkFlexMotor::SetPosition(units::angle::degree_t degree, int slot)
{
    if (slot < 0 || slot > 3) return;
    closedLoopController.SetSetpoint(degree.value(), rev::spark::SparkLowLevel::ControlType::kPosition,
                                     static_cast<rev::spark::ClosedLoopSlot>(slot));
}

void SparkFlexMotor::SetPID(double p, double i, double d, double ff, int slot)
{
    if (slot < 0 || slot > 3) return;
    auto pidSlot = static_cast<rev::spark::ClosedLoopSlot>(slot);
    config.closedLoop.Pid(p, i, d, pidSlot);
    config.closedLoop.feedForward.kV(ff, pidSlot);
    Configure();
}

void SparkFlexMotor::SetInverted(bool inversion)
{
    config.Inverted(inversion);
    Configure();
}

void SparkFlexMotor::SetNeutralMode(NeutralMode neutralMode)
{
    config.SetIdleMode(neutralMode == NeutralMode::Brake
        ? rev::spark::SparkBaseConfig::IdleMode::kBrake
        : rev::spark::SparkBaseConfig::IdleMode::kCoast);
    Configure();
}

void SparkFlexMotor::SetConversionFactor(double value)
{
    if (!std::isfinite(value) || value <= 0) return;
    config.encoder.PositionConversionFactor(360.0 * value);
    config.encoder.VelocityConversionFactor(value);
    Configure();
}

void SparkFlexMotor::SetWrapping()
{
    config.closedLoop.PositionWrappingEnabled(true);
    config.closedLoop.PositionWrappingMinInput(-180.0);
    config.closedLoop.PositionWrappingMaxInput(180.0);
    Configure();
}

void SparkFlexMotor::EnableCurrentLimit(bool enable)
{
    currentLimitEnabled = enable;
    config.SmartCurrentLimit(enable ? currentLimitAmps : 80);
    Configure();
}

void SparkFlexMotor::SetCurrentLimit(int current)
{
    if (current <= 0) return;
    currentLimitAmps = current;
    config.SmartCurrentLimit(currentLimitEnabled ? currentLimitAmps : 80);
    Configure();
}

void SparkFlexMotor::EnableForwardSoftLimit(bool enable)
{
    config.softLimit.ForwardSoftLimitEnabled(enable);
    Configure();
}

void SparkFlexMotor::EnableReverseSoftLimit(bool enable)
{
    config.softLimit.ReverseSoftLimitEnabled(enable);
    Configure();
}

void SparkFlexMotor::SetForwardSoftLimit(units::angle::degree_t degrees)
{
    config.softLimit.ForwardSoftLimit(degrees.value());
    Configure();
}

void SparkFlexMotor::SetReverseSoftLimit(units::angle::degree_t degrees)
{
    config.softLimit.ReverseSoftLimit(degrees.value());
    Configure();
}

void SparkFlexMotor::ApplyConfiguration(MotorConfig motorConfig)
{
    //applies the neutral mode for the motor controller
    config.Inverted(motorConfig.inverted);

    //motors are set to neutral on default
    config.SetIdleMode(motorConfig.neutralMode == NeutralMode::Brake
        ? rev::spark::SparkBaseConfig::IdleMode::kBrake
        : rev::spark::SparkBaseConfig::IdleMode::kCoast);

    //if the current limits are enabled, set it to the given value
    currentLimitEnabled = motorConfig.enableCurrentLimit;
    if (motorConfig.currentLimitAmps > 0)
        currentLimitAmps = motorConfig.currentLimitAmps;
    config.SmartCurrentLimit(currentLimitEnabled ? currentLimitAmps : 80);

    //if voltage compensation is enabled, set it to the stated value. If not, disable
    if (motorConfig.enableVoltageCompensation)
        config.VoltageCompensation(motorConfig.voltageCompensation);
    else
        config.DisableVoltageCompensation();
    //applies the conversion factors to the encoders
    if (std::isfinite(motorConfig.positionConversionFactor) && motorConfig.positionConversionFactor > 0)
        config.encoder.PositionConversionFactor(360.0 * motorConfig.positionConversionFactor);
    if (std::isfinite(motorConfig.velocityConversionFactor) && motorConfig.velocityConversionFactor > 0)
        config.encoder.VelocityConversionFactor(motorConfig.velocityConversionFactor);
    
    //sets all the limits
    config.softLimit.ForwardSoftLimitEnabled(motorConfig.enableForwardSoftLimit);
    config.softLimit.ReverseSoftLimitEnabled(motorConfig.enableReverseSoftLimit);
    config.softLimit.ForwardSoftLimit(motorConfig.forwardSoftLimit.value());
    config.softLimit.ReverseSoftLimit(motorConfig.reverseSoftLimit.value());
    Configure();
}

units::angle::degree_t SparkFlexMotor::getPosition()
{
    return units::angle::degree_t{encoder.GetPosition()};
}

units::angular_velocity::revolutions_per_minute_t SparkFlexMotor::getVelocity()
{
    return units::angular_velocity::revolutions_per_minute_t{encoder.GetVelocity()};
}

bool SparkFlexMotor::isForwardSoftLimitEnabled()
{
    return motor.configAccessor.softLimit.GetForwardSoftLimitEnabled();
}

bool SparkFlexMotor::isReverseSoftLimitEnabled()
{
    return motor.configAccessor.softLimit.GetReverseSoftLimitEnabled();
}

units::angle::degree_t SparkFlexMotor::getForwardLimit()
{
    return units::angle::degree_t{motor.configAccessor.softLimit.GetForwardSoftLimit()};
}

units::angle::degree_t SparkFlexMotor::getReverseLimit()
{
    return units::angle::degree_t{motor.configAccessor.softLimit.GetReverseSoftLimit()};
}
