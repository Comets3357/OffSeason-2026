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
            units::turn_t yawAngle;
            units::turn_t pitchAngle;
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

        //sets all flywheels to a duty cycle from -1 to 1 with individual motor commands.
        void setFlywheelSpeed(double percent);


        void setFlywheelVelocity(double RPMs);


        //sets the hood pitch in turns.
        void setHoodPitch(units::turn_t pitch);

        // Apply the dashboard target in internal motor turns, clamped to the soft limits.
        void setHoodFromDashboard();


        //sets the motor that pitches the hood to a certain speed (probably dont use this one unless you know what you're doing.)
        void setHoodPitchVelocity(double speed);

        //getters

        //returns the velocity of the primary flywheel.
        double getPrimaryFlywheelRPM();

        //returns the velocity of the secondary flywheel.
        double getSecondaryFlywheelRPM();
        

        //maybe we would want to merge these together to return a struct or something?


        //returns the velocity of the hood's pitch motor
        double getHoodPitchVelocity();

        //getters for hood pitch and yaw in turns, aswell as the targets
        double getHoodPitch();
        double getTargetHoodPitch();
        double getHoodYaw();
        double getTargetHoodYaw();


        double getSmartDashboardVelocity();
        double getSmartDashboardSpeed();

        units::turn_t getSDTargetPitch();
        units::turn_t getSDTargetYaw();

        bool isYawWithinRange(double angle);

        shootingParams calculateTrajectorySetpoints(frc::Pose2d targetPose, frc::Pose2d robotPose);

    private:

        const double pitchP = 1;
        const double pitchI = 0;
        const double pitchD = 0;
        
        double flywheelP = 0.004;
        double flywheelI = 0.0;
        double flywheelD = 0.018;
        double flywheelFF = 0.0042;
        
        const frc::Transform2d robotRelativeTurretTransform{
            frc::Translation2d{-5.5_in, -9.5_in}, // translation relative to robot center
            frc::Rotation2d{}                   // no rotation change
        };
        double phaseDelay = 0.06; // seconds of delay in sensor values, can be tuned, 6328 uses 0.03

        VisionSubsystem& m_vision;
        DriveSubsystem& m_drive;

        static constexpr int kPrimaryFlywheelId = 31;
        NovaMotor primaryFlywheelMotor{kPrimaryFlywheelId};
        NovaMotor secondaryFlywheelMotor{32};
        NovaMotor thirdFlywheelMotor{33};
        NovaMotor shooterPitchMotor{41};
        
        double flywheelOperatorOffset = 0;
        double HSLRpmOffsetConstant = 50; /*THIS NEEDS TO BE TUNEDDDDDDDDDDD :) */
        thrifty::CanEncoder shooterEncoder{0, 0}; //CAN bus 0, encoder ID 51

        ShooterLookup shooterLookup{};
};
