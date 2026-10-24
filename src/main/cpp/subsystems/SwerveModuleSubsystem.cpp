#include "subsystems/SwerveModuleSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>
#include <cmath>

#include <units/current.h>

constexpr double DRIVE_GEAR_RATIO = ModuleConstants::kDrivingMotorReduction;
constexpr double DRIVE_WHEEL_DIAMETER = 0.1016; // 4 inch = 0.1016 m
constexpr double DRIVE_METERS_PER_MOTOR_TURN =
    std::numbers::pi * DRIVE_WHEEL_DIAMETER / DRIVE_GEAR_RATIO;

constexpr double RPM_TO_MPS = DRIVE_METERS_PER_MOTOR_TURN / 60.0;
constexpr double MPS_TO_RPM = 1.0 / RPM_TO_MPS;

constexpr units::current::ampere_t steerCurrentLimit = 25_A;
constexpr units::current::ampere_t driveCurrentLimit = 60_A;

// Initial Nova velocity gains use rotations/second instead of the REV loop's RPM.
// Scale the previous duty-cycle gains by 60; verify tuning on the new controllers.
constexpr double driveP = 0.0004 * 1.0;
constexpr double driveI = 0.0;
constexpr double driveD = 0.0;
constexpr double driveFF = 0.0004 * 10.0;

constexpr double steerP = 4;
constexpr double steerI = 0.0;
constexpr double steerD = 0.0;


SwerveModule::SwerveModule(int steerId, int driveId, double offset,
                         bool steerEncoderInverted, bool driveMotorInverted)
    : m_driveMotor{driveId, thrifty::Motor::NEO},
      m_steerMotor{steerId, thrifty::Motor::MINION},
      offset{frc::Rotation2d{units::degree_t{offset}}},
      steerEncoderInverted{steerEncoderInverted}
{
    //configure the Nova steering motor and use its absolute encoder for position.
    MotorBase::MotorConfig steerConfig;
    steerConfig.currentLimitAmps = steerCurrentLimit.value();
    steerConfig.neutralMode = MotorBase::NeutralMode::Coast;
    m_steerMotor.ApplyConfiguration(steerConfig);
    m_steerMotor.SetPID(steerP, steerI, steerD, 0.0, 0);
    m_steerMotor.SetAbsoluteWrapping(true);

    // The Nova wrapper reports degrees and RPM, preserving odometry conversions.
    MotorBase::MotorConfig driveConfig;
    driveConfig.currentLimitAmps = driveCurrentLimit.value();
    driveConfig.inverted = driveMotorInverted;
    driveConfig.enableVoltageCompensation = true;
    driveConfig.voltageCompensation = 12.0;
    m_driveMotor.ApplyConfiguration(driveConfig);
    // Nova uses native F here, not the REV voltage feedforward kV.
    m_driveMotor.SetPID(driveP, driveI, driveD, driveFF, 0);
}

double SwerveModule::GetSteerRotations() {
    //recieves the rotation from the motor, if the encoder is inverted, reverse the number.
    double rotations = units::turn_t{m_steerMotor.getAbsolutePosition()}.value();
    return steerEncoderInverted ? -rotations : rotations;
}

void SwerveModule::FlipEncoder() {
    //reverse the module's encoder coordinates and matching position targets.
    steerEncoderInverted = !steerEncoderInverted;
}

frc::SwerveModuleState SwerveModule::GetState() {
    //recieves the angle from the steer motors, as well as the speed from the drive motors
    frc::Rotation2d angle = frc::Rotation2d{units::degree_t{GetSteerRotations() * 360.0}};
    units::meters_per_second_t speed = units::meters_per_second_t{m_driveMotor.getVelocity().value() * RPM_TO_MPS};

    //intantiates a swerve module state from these values, and outputs it.
    frc::SwerveModuleState state;
    state.angle = (angle - offset);
    state.speed = speed;

    return state;
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &desiredState)
{
    // Keep the kinematics angle and speed paired directly. Optimization normally
    // adds 180 degrees and reverses speed; leave that out while validating the
    // physical steering coordinates and drive direction on these modules.
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle = desiredState.angle + offset;

    // Undo software encoder inversion to get the native absolute sensor target.
    auto steeringTarget = correctedDesiredState.angle.Degrees();
    if (steerEncoderInverted) steeringTarget = -steeringTarget;
    m_steerMotor.SetAbsolutePosition(steeringTarget);

    m_driveMotor.SetVelocity(units::revolutions_per_minute_t{
        correctedDesiredState.speed.value() * MPS_TO_RPM});
}

void SwerveModule::Stop() {
    m_driveMotor.Set(0);
    m_steerMotor.Set(0);
    correctedDesiredState.speed = 0_mps;
}

frc::SwerveModuleState SwerveModule::GetDesiredState() {
    //get the member variable of the desired state of the module
    return correctedDesiredState;
}

void SwerveModule::UpdateDashboard(const std::string& name) {
    auto actual = GetState();
    // Use robot-relative angles for both measured and requested values.
    auto requestedAngle = correctedDesiredState.angle - offset;
    std::string key = "Swerve/" + name;
    frc::SmartDashboard::PutNumber(key + "/Actual Angle (turns)",
        std::remainder(units::turn_t{actual.angle.Radians()}.value(), 1.0));
    frc::SmartDashboard::PutNumber(key + "/Requested Angle (turns)",
        std::remainder(units::turn_t{requestedAngle.Radians()}.value(), 1.0));
    frc::SmartDashboard::PutNumber(key + "/Distance (m)", GetPosition().distance.value());
    frc::SmartDashboard::PutNumber(key + "/Actual Speed (mps)", actual.speed.value());
    frc::SmartDashboard::PutNumber(key + "/Requested Speed (mps)", correctedDesiredState.speed.value());
}

double SwerveModule::GetVelocity() {
    //get the velocity of the drive motor
    return m_driveMotor.getVelocity().value();
}

void SwerveModule::SimulationPeriodic() {
    m_steerMotor.SimulationPeriodic();
    m_driveMotor.SimulationPeriodic();
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    //gets the position of the swerve module via calculating the amount the module has traveled
    frc::SwerveModulePosition position = {units::meter_t{GetRelativePosition() * DRIVE_METERS_PER_MOTOR_TURN},
        units::degree_t{GetSteerRotations() * 360.0} - offset.Degrees()};

    return position;
}

void SwerveModule::ResetEncoders() { 
    //resets the amount recieved from the encoder to zero.
    drivePositionOffset = units::turn_t{m_driveMotor.getPosition()}.value();
}

double SwerveModule::GetRelativePosition() {
    //gets the relative position of the drive motor
    return units::turn_t{m_driveMotor.getPosition()}.value() - drivePositionOffset;
}

double SwerveModule::getInternalPosition() {
    //raw steering position in motor rotations, before chassis offset.
    return units::turn_t{m_steerMotor.getPosition()}.value();
}
