#include "subsystems/GyroWrapper.h"

#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

GyroWrapper::GyroWrapper()
{
    if (frc::RobotBase::IsSimulation()) return;
    redux::sensors::canandgyro::CanandgyroSettings settings;
    settings.SetYawFramePeriod(10_ms);
    const auto failed = m_gyro.SetSettings(settings);
    frc::SmartDashboard::PutBoolean("Drive/Gyro Yaw Configuration OK", failed.IsEmpty());
}

void GyroWrapper::UpdateDashboard()
{
    if (frc::RobotBase::IsSimulation()) return;
    frc::SmartDashboard::PutBoolean("Drive/Gyro Connected", m_gyro.IsConnected());
    frc::SmartDashboard::PutNumber("Drive/Gyro CAN ID", 9);
    frc::SmartDashboard::PutNumber("Drive/Gyro Raw Heading (deg)",
        GetHardwareRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Drive/Gyro Yaw Age (s)",
        (frc::Timer::GetFPGATimestamp() - m_gyro.GetYawFrame().GetTimestamp()).value());
}

void GyroWrapper::ZeroGyro()
{
    SetAngle(units::degree_t{0});
}

void GyroWrapper::SetAngle(units::degree_t angle)
{
    if (frc::RobotBase::IsSimulation())
        m_simHeading = frc::Rotation2d{angle};
    else
        m_headingOffset = frc::Rotation2d{angle} - GetHardwareRotation();
}

frc::Rotation2d GyroWrapper::GetRotation()
{
    if (frc::RobotBase::IsSimulation()) return m_simHeading;
    return GetHardwareRotation() + m_headingOffset;
}

frc::Rotation2d GyroWrapper::GetHardwareRotation()
{
    return m_gyro.GetRotation2d();
}
