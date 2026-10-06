#include "subsystems/GyroWrapper.h"

#include <frc/RobotBase.h>

GyroWrapper::GyroWrapper() = default;

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
