
#pragma once

#include <redux/sensors/Canandgyro.h>
#include <redux/canand/CanandEventLoop.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <units/angle.h>

class GyroWrapper
{
    public: 

        GyroWrapper(); 
        
        //makes whatever angle the robot is currently facing the '0' degree position.
        void ZeroGyro(); 

        /** set the gyro's estimated angle to a specific angle.
        * @param angle the angle you want to set to
        **/
        void SetAngle(units::degree_t angle);

        // gets the rotation of the robot in rotations
        frc::Rotation2d GetRotation(); 

        //sets the simulated gyro heading.
        void SetSimState(frc::Rotation2d rotation) { m_simHeading = rotation; }

    private: 
        frc::Rotation2d GetHardwareRotation();

        redux::sensors::canandgyro::Canandgyro m_gyro{9};
        frc::Rotation2d m_headingOffset{};
        frc::Rotation2d m_simHeading{};
};
