#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "common/ShooterLookup.h"
#include "wrappers/NovaBase.h"

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
        ShooterSubsystem(DriveSubsystem& drive, VisionSubsystem& vision);

        void Periodic() override;
        void SimulationPeriodic() override;

        enum Hub {
            blue, 
            red
        };

        struct shootingParams {
            units::degree_t yawAngle;
            units::degree_t pitchAngle;
            units::revolutions_per_minute_t flywheelRpm;
        };

        shootingParams calculateTrajectory(frc::Pose2d targetPose);
        shootingParams calculateTrajectory(frc::Pose2d targetPose, bool debug);

        frc::Pose2d getTargetPose();
        frc::Pose2d getSetPoints(char side);

        void resetOperatorFlywheelCompensation();
        void changeOperatorFlywheelCompensation(double value);
        double getOperatorFlywheelCompensation();


        //setters

        //sets the primary flywheel's velocity to a given value.
        void setFlywheelSpeed(double velocity);

        //sets the secondary flywheel's velocity to a given value.
        void setSecondaryFlywheelVelocity(double velocity);

        void setFlywheelVelocity(double RPMs);

        //sets the hood yaw in degrees.
        void setHoodYaw(units::degree_t yaw);

        //sets the hood pitch in degrees.
        void setHoodPitch(units::degree_t pitch);

        //sets the motor that yaws the hood to a certain speed (probably dont use this one unless you know what you're doing.)
        void setHoodYawVelocity(double speed);

        //sets the motor that pitches the hood to a certain speed (probably dont use this one unless you know what you're doing.)
        void setHoodPitchVelocity(double speed);

        //getters

        //returns the velocity of the primary flywheel.
        double getPrimaryFlywheelRPM();

        //returns the velocity of the secondary flywheel.
        double getSecondaryFlywheelRPM();
        

        //maybe we would want to merge these together to return a struct or something?

        //returns the velocity of the hood's yaw motor
        double getHoodYawVelocity();

        //returns the velocity of the hood's pitch motor
        double getHoodPitchVelocity();

        //getters for hood pitch and yaw, aswell as the targets
        double getHoodPitch();
        double getTargetHoodPitch();
        double getHoodYaw();
        double getTargetHoodYaw();


        double getSmartDashboardVelocity();
        double getSmartDashboardSpeed();

        units::degree_t getSDTargetPitch();
        units::degree_t getSDTargetYaw();

        bool isYawWithinRange(double angle);

        shootingParams calculateTrajectorySetpoints(frc::Pose2d targetPose, frc::Pose2d robotPose);

    private:

        const double yawP = 0.12;
        const double yawI = 0;
        const double yawD = 0;

        const double lowYawP = 0.08;

        const double pitchP = 0.16;
        const double pitchI = 0;
        const double pitchD = 0;
        
        const double flywheelP = 0.004;
        const double flywheelI = 0.0;
        const double flywheelD = 0.018;
        const double flywheelFF = 0.0042;
        
        const frc::Transform2d robotRelativeTurretTransform{
            frc::Translation2d{-5.5_in, -9.5_in}, // translation relative to robot center
            frc::Rotation2d{}                   // no rotation change
        };
        double phaseDelay = 0.06; // seconds of delay in sensor values, can be tuned, 6328 uses 0.03

        VisionSubsystem& m_vision;
        DriveSubsystem& m_drive;

        NovaMotor shooterYawMotor{31};
        NovaMotor shooterPitchMotor{32};
        NovaMotor primaryFlywheelMotor{33};
        // NovaMotor secondaryFlywheelMotor{34};
        
        double flywheelOperatorOffset = 0;
        double HSLRpmOffsetConstant = 50; /*THIS NEEDS TO BE TUNEDDDDDDDDDDD :) */

        ShooterLookup shooterLookup{};
};