#include "subsystems/SwerveModuleSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>

#include <units/current.h>

constexpr double DRIVE_GEAR_RATIO = 5.54;
constexpr double DRIVE_WHEEL_DIAMETER = 0.1016; // 4 inch = 0.1016 m
constexpr double DRIVE_WHEEL_WEAR = -0.00065598; // Negative if greater than nominal diameter

constexpr double RPM_TO_MPS = (1.0/60.0) /*minutes to seconds*/ * (1.0 / 3.13297132071) /*meter to wheel rotations*/ * (1 / 5.54); /*one wheel rotation to drive motor rotations*/
constexpr double MPS_TO_RPM = (1.0) /*Meters per second*/ * (60.0/1.0) /*Seconds to Minute*/ * (3.13297132071/1.0) /*Wheel Rotations to Meter*/ * (5.54 / 1); /*Drive motor rotations to one wheel rotation*/

constexpr units::current::ampere_t steerCurrentLimit = 25_A;
constexpr units::current::ampere_t driveCurrentLimit = 60_A;

constexpr double driveP = 0.0004;
constexpr double driveI = 0.0;
constexpr double driveD = 0.0;
constexpr double driveFF = 0.0004;

constexpr double steerP = 20.0;
constexpr double steerI = 0.0;
constexpr double steerD = 0.0;


SwerveModule::SwerveModule(int steerId, int driveId, double offset, bool steerEncoderInverted)
    : m_driveMotor{driveId},
      m_steerMotor{steerId, thrifty::Motor::MINION},
      offset{frc::Rotation2d{units::degree_t{offset}}},
      steerEncoderInverted{steerEncoderInverted}
{
    //configure the Nova steering motor and use its absolute encoder for position.
    MotorBase::MotorConfig steerConfig;
    steerConfig.currentLimitAmps = static_cast<int>(steerCurrentLimit.value());
    steerConfig.neutralMode = MotorBase::NeutralMode::Coast;
    m_steerMotor.ApplyConfiguration(steerConfig);
    m_steerMotor.SetPID(steerP, steerI, steerD, 0.0, 0);
    m_steerMotor.SetAbsoluteWrapping(true);

    //the REV wrapper takes a MotorConfig and reports degrees and RPM.
    MotorBase::MotorConfig driveConfig;
    driveConfig.currentLimitAmps = static_cast<int>(driveCurrentLimit.value());
    driveConfig.inverted = true;
    driveConfig.enableVoltageCompensation = true;
    driveConfig.voltageCompensation = 12.0;
    m_driveMotor.ApplyConfiguration(driveConfig);
    //convert the old duty-cycle feedforward to the new volts-per-RPM kV.
    m_driveMotor.SetPID(driveP, driveI, driveD, driveFF * 12.0, 0);
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
    // Apply chassis angular offset to the desired state.
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle =
        desiredState.angle +
        frc::Rotation2d{offset.Degrees()};

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.Optimize(
        frc::Rotation2d(units::degree_t{GetSteerRotations() * 360}));

    //set the drive motorto a specific speed.
    m_driveMotor.SetVelocity(
        units::revolutions_per_minute_t(
            correctedDesiredState.speed.value() * MPS_TO_RPM
        )
    );

    //once we calculate the desired angle state, set the position of the steer motors
    units::turn_t targetRotations{correctedDesiredState.angle.Degrees().value() / 360.0};

    //undo the optional software inversion before sending a native sensor target.
    if (steerEncoderInverted) targetRotations = -targetRotations;
    m_steerMotor.SetAbsolutePosition(units::degree_t{targetRotations});
}

frc::SwerveModuleState SwerveModule::GetDesiredState() {
    //get the member variable of the desired state of the module
    return correctedDesiredState;
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
    frc::SwerveModulePosition position = {units::meter_t{(GetRelativePosition() / DRIVE_GEAR_RATIO) * std::numbers::pi * DRIVE_WHEEL_DIAMETER},
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
