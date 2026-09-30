#include <wrappers/MotorBase.h>
#include <wrappers/SparkBase.h>

//basic function for setting speed before anything special.
void SparkMaxMotor::Set(double speed) {
    motor.Set(speed);
};

//more complicated than Set, but just setting motor voltage.
void SparkMaxMotor::SetVoltage(double voltage) {
    motor.SetVoltage(units::voltage::volt_t(voltage));
}

//stop the motor, s all previous commands and stop the motor
void SparkMaxMotor::Stop() {
    motor.Set(0);
}

//sets the motor to go a specific velocity.
void SparkMaxMotor::SetVelocity(double rpm) {
    closedLoopController.SetReference(rpm, rev::spark::SparkLowLevel::ControlType::kVelocity , rev::spark::ClosedLoopSlot::kSlot0);
}

//sets the motor to a specific position.
void SparkMaxMotor::SetPosition(units::angle::degree_t degree) {
    closedLoopController.SetReference(degree.value(), rev::spark::SparkLowLevel::ControlType::kPosition , rev::spark::ClosedLoopSlot::kSlot0)
}

//configuration setter for inversion
void SparkMaxMotor::SetInverted(bool inversion) {
    config.absoluteEncoder.Inverted(inversion)
}

//configuration setter for neutral move
void SparkMaxMotor::SetNeutralMode(NeutralMode neutralMode) ;

//configuration setter for enabling current limit
void EnableCurrentLimit(bool enable) ;

//configuration setter for current limit value
void SetCurrentLimit(int current) ;

//configuration setter for enabling the forward soft limit
void EnableForwardSoftLimit(bool enable) ;

//configuration setter for enabling the reverse soft limit
void EnableReverseSoftLimit(bool enable) ;

//configuration setter for setting the forward limit position
void SetForwardSoftLimit(units::angle::degree_t degrees) ;

//configuration setter for setting the reverse limit position
void SetReverseSoftLimit(units::angle::degree_t degrees) ;

//apply the configuration of values
void ApplyConfiguration(MotorConfig motorConfig) ;

// function to set the PIDs
void SetPID(double p, double i, double d, double ff) ;

// function to recieve if the forward limit is enabled;
bool IsForwardLimitEnabled() ;

// function to recieve if the reverse limit is enabled;
bool IsReverseLimitEnabled() ;

void SetConversionFactor(double conversion) ;

// function to pull the current position of the motor
//(prioritizes absolute)
units::angle::degree_t GetPosition();

//getter to return the velocity of the current motor controller
//(Prioritizes absolute)
//TODO give it a better type!!
 double GetVelocity() ;

 
void SparkMaxMotor::SetPercent(double percent)
{
    m_lastOutput = percent;
    if (frc::RobotBase::IsSimulation())
    {
        m_motorSim->SetBusVoltage(percent * 12);
    } else {
        motor.Set(percent);
    }    
}

void SparkMaxMotor::StopMotor()
{
    motor.Set(0);
}

void SparkMaxMotor::setFeedbackSensor(encoderType encoder)
{
    if (encoder == Motor::encoderType::absolute) // sets feedback sensor to absolute
    {
        config.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder);
    }
    else // sets feedback sensor to relative
    {
        config.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder);
    }
}

void SparkMaxMotor::setAbsoluteEncoderInverted(bool inverted)
{
    config.absoluteEncoder.Inverted(inverted);
}

void SparkMaxMotor::setMinOutput(double min)
{
    config.closedLoop.MinOutput(min);
}

void SparkMaxMotor::setMaxOutput(double max)
{
    config.closedLoop.MaxOutput(max);
}

void SparkMaxMotor::setOutputRange(double min, double max)
{
    config.closedLoop.OutputRange(min, max);
}

void SparkMaxMotor::setPositionWrapingMinInput(double minInput)
{
    config.closedLoop.PositionWrappingMinInput(minInput);
}

void SparkMaxMotor::setPositionWrappingMaxInput(double maxInput)
{
    config.closedLoop.PositionWrappingMaxInput(maxInput);
}

void SparkMaxMotor::setPositionWrappingMaxRange(double minInput, double maxInput)
{
    config.closedLoop.PositionWrappingInputRange(minInput, maxInput);
}

void SparkMaxMotor::setPositionWrappingEnabled(bool enab)
{
    config.closedLoop.PositionWrappingEnabled(enab);
}

void SparkMaxMotor::setReference(double ref, controlType ctrl)
{
    if (ctrl == Motor::controlType::position)
    {
        if (frc::RobotBase::IsSimulation()) {
            m_motorSim->SetPosition(ref);
        } else {
            closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    else if (ctrl == Motor::controlType::velocity)
    {
        if (frc::RobotBase::IsSimulation()) {
            m_motorSim->SetVelocity(ref);
        } else {
            closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kVelocity);
        }
    }
}

void SparkMaxMotor::setReference(double ref, controlType ctrl, double slot) {

    rev::spark::ClosedLoopSlot internalSlot;
    if (slot == 0)  internalSlot = rev::spark::kSlot0;
    if (slot == 1)  internalSlot = rev::spark::kSlot1;
    if (slot == 2)  internalSlot = rev::spark::kSlot2;


    if (ctrl == Motor::controlType::position)
    {
        if (frc::RobotBase::IsSimulation()) {
            m_motorSim->SetPosition(ref);
        } else {
            closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kPosition, internalSlot);
        }
    }
    else if (ctrl == Motor::controlType::velocity)
    {
        if (frc::RobotBase::IsSimulation()) {
            m_motorSim->SetVelocity(ref);
        } else {
            closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kVelocity , internalSlot);
        }
    }
}

bool SparkMaxMotor::IsReverseLimitPressed()
{
    return ReverseLimit.Get();
}

double SparkMaxMotor::GetZeroOffset()
{
  return motor.configAccessor.absoluteEncoder.GetZeroOffset();
}

void SparkMaxMotor::SetElevatorAbsolutePosition(double ZeroOffset)
{
    rev::spark::SparkBaseConfig PositionConfig;

    PositionConfig.absoluteEncoder.ZeroOffset(ZeroOffset);

    motor.Configure(PositionConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

double SparkMaxMotor::GetRelativeVelocity()
{
    if (frc::RobotBase::IsSimulation()) {
        return m_motorSim->GetRelativeEncoderSim().GetVelocity();
    }
    return RelativeEncoder.GetVelocity();
}

double SparkMaxMotor::GetRelativePosition()
{
    if (frc::RobotBase::IsSimulation()) {
        return m_motorSim->GetRelativeEncoderSim().GetPosition();
    }
    return RelativeEncoder.GetPosition();
}

double SparkMaxMotor::GetAbsolutePosition()
{
    if (frc::RobotBase::IsSimulation()) {
        return m_motorSim->GetAbsoluteEncoderSim().GetPosition();
    }
    return AbsoluteEncoder.GetPosition();
}

double SparkMaxMotor::GetAbsoluteVelocity()
{
    if (frc::RobotBase::IsSimulation()) {
        return m_motorSim->GetAbsoluteEncoderSim().GetVelocity();
    }
    return AbsoluteEncoder.GetVelocity();
}

double SparkMaxMotor::GetOutputCurrent() {
    return motor.GetOutputCurrent();
}

void SparkMaxMotor::SetRelativePosition(double pos)
{
    RelativeEncoder.SetPosition(pos);
}

void SparkMaxMotor::SetFollow(SparkMaxMotor& leader) 
{
    config.Follow(leader.motor, true);
}

void SparkMaxMotor::SetFollow(int canID) 
{
    config.Follow(canID, true);
}

void SparkMaxMotor::setPID(double p, double i, double d, double ff)
{
    config.closedLoop.Pidf(p, i, d, ff);
}

void SparkMaxMotor::setPID(double p, double i, double d)
{
    config.closedLoop.Pidf(p, i, d, 0);
}

void SparkMaxMotor::setPID(double p, double i, double d, double ff, int slot)
{
    switch (slot)
    {
    case 1:
        config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot1);
        break;
    case 2:
        config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot2);
        break;
    case 3:
        config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot3);
        break;
    default:
        config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot0);
    }
}

void SparkMaxMotor::setForwardSoftLimit(double limit)
{
    config.softLimit.ForwardSoftLimit(limit);
}

void SparkMaxMotor::setReverseSoftLimit(double limit)
{
    config.softLimit.ReverseSoftLimit(limit);
}

void SparkMaxMotor::enableForwardSoftLimit(bool enab)
{
    config.softLimit.ForwardSoftLimitEnabled(enab);
}

void SparkMaxMotor::enableReverseSoftLimit(bool enab)
{
    config.softLimit.ReverseSoftLimitEnabled(enab);
}

void SparkMaxMotor::setInverted(bool b)
{
    config.Inverted(b);
}

void SparkMaxMotor::setAbsolutePositionConversionFactor(double factor)
{
    config.absoluteEncoder.PositionConversionFactor(factor);
}

void SparkMaxMotor::zeroOffset(double offset)
{
    config.absoluteEncoder.ZeroOffset(offset);
}

void SparkMaxMotor::setAbsoluteVelocityConversionFactor(double factor)
{
    config.absoluteEncoder.VelocityConversionFactor(factor);
}

void SparkMaxMotor::SetSmartCurrentLimit(double lim)
{
    config.SmartCurrentLimit(lim);
}

void SparkMaxMotor::setRelativeVelocityConversionFactor(double fac)
{
    config.encoder.VelocityConversionFactor(fac);
}

void SparkMaxMotor::setRelativePositionConversionFactor(double fac)
{
    config.encoder.PositionConversionFactor(fac);
}

double SparkMaxMotor::GetAppliedOutput()
{
    return motor.GetAppliedOutput();
}

void SparkMaxMotor::configure()
{
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    motor.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    config.encoder.UvwAverageDepth(2);
    config.encoder.UvwMeasurementPeriod(10);
}

void SparkMaxMotor::configureNoPersist() {
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    motor.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    config.encoder.UvwAverageDepth(2);
    config.encoder.UvwMeasurementPeriod(10);
}

void SparkMaxMotor::enableVoltageCompensation(double voltage) {
    config.VoltageCompensation(voltage);
}
