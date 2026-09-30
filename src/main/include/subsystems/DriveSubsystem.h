#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include "subsystems/SwerveModuleSubsystem.h"
#include <redux/sensors/Canandgyro.h>
#include "subsystems/GyroWrapper.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "subsystems/VisionSubsystem.h"
#include <photon/PhotonPoseEstimator.h>
#include <Constants.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <units/time.h>

using namespace DriveConstants;


class DriveSubsystem : public frc2::SubsystemBase
{
public:
        DriveSubsystem();

        // Runs once per scheduler loop on the real robot and in sim.
        // Updates module state snapshots, pose estimation, and vision gyro data.
        void Periodic() override;
        
        // Runs once per scheduler loop only in simulation.
        // Updates module motor sims, simulated chassis motion, simulated gyro state, and sim pose output.
        void SimulationPeriodic() override;

        // Commands the swerve drive with desired chassis speeds.
        // If fieldRelative is true, xSpeed/ySpeed are field-oriented and get converted using gyro heading.
        // If fieldRelative is false, speeds are already robot-relative.
        void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed,
              units::radians_per_second_t rot,
              bool fieldRelative);

        // Resets the gyro heading to 0 degrees so future field-relative driving uses the new reference.
        void ZeroHeading();

        // Resets all four module drive/turning encoder positions to their base values.
        void ResetEncoders();

        // Uses simple PID controllers to drive toward targetPos.
        // max_output limits translation speed, and the tolerances decide when the target is close enough.
        void GoToPosition(frc::Pose2d targetPos, double max_output, units::meter_t translationTolerance, units::degree_t degreeTolerance);

        // Points the modules into an X pattern with zero speed so the robot resists being pushed.
        void SetX();

        // Forces the pose estimator, gyro angle, and sim pose if applicable to a known field pose.
        void ResetOdometry(frc::Pose2d pose);

        //getters

        // Returns robot-relative chassis speeds.
        // In sim this returns the limited simulated chassis speed; on real hardware it derives speed from modules.
        frc::ChassisSpeeds GetRobotRelativeSpeeds();
    
        // Returns the pose estimator's current best field pose for the robot.
        frc::Pose2d GetEstimatedPose();

        // Returns straight-line field distance from the estimated robot pose to the target pose.
        double GetDistanceFromTarget(frc::Pose2d target);

        // Returns the current gyro heading as a Rotation2d.
        frc::Rotation2d GetGyroHeading();

        // Reads DriverStation alliance and returns true for blue, false for red.
        // Defaults to blue when the alliance is unknown.
        bool IsBlueAlliance();

private:
        // Groups the drive-only simulation math away from the real robot control path.
        // These helpers keep SimulationPeriodic focused on the order of operations.
        struct Simulation {
                // Step one scalar value toward a target using an acceleration-style limit.
                static double MoveTowards(double current, double target, double maxDelta);

                // Shape field-relative linear speed and omega into something the sim can reach.
                static frc::ChassisSpeeds LimitFieldSpeeds(
                        frc::ChassisSpeeds targetFieldSpeeds,
                        frc::ChassisSpeeds currentFieldSpeeds,
                        units::second_t dt);

                // Advance the simulated field pose from field-relative chassis speeds.
                static frc::Pose2d IntegratePose(
                        frc::Pose2d currentPose,
                        frc::ChassisSpeeds fieldRelativeSpeeds,
                        units::second_t dt);

        };

        // Converts the four measured module states into robot-relative chassis speeds.
        frc::ChassisSpeeds GetMeasuredRobotRelativeSpeeds();

        // Updates field localization.
        // In sim, publishes the simulated pose; on real hardware, fuses odometry and vision measurements.
        void PoseEstimation();

        // Adapter used by PathPlanner to command the drivetrain with a ChassisSpeeds object.
        void DriveFromChassisSpeeds(frc::ChassisSpeeds speed, bool fieldRelative);

        // Registers pose, reset, speed, and drive callbacks with PathPlanner's AutoBuilder.
        void ConfigureAuton();

        // Publishes the simulated pose X, Y, and heading to NetworkTables for dashboards/logging.
        void PublishSimEstimatedPose();

        // Wraps any angle in degrees into the 0-360 range.
        double WrapAngle(double angle); 

        //instantiates a vision object for the drive code to use for localization.
        VisionSubsystem m_vision; 

        //instantiates a gyro object for the drive code to use for direction mapping.
        GyroWrapper m_gyro; 

        //base poses to use for stdDev calculations
        frc::Pose2d blueHub{4.625_m, 4.035_m, 0_rad};
        frc::Pose2d redHub{11.92_m, 4.035_m, 0_rad};

        //instantiates objects of a field and a horizontal reflection of the field
        //used to show location of robot in advantagescope or other applications
        frc::Field2d m_mirrorField; 
        frc::Field2d m_field;

        //fallback pose if the current estimated pose does not work
        frc::Pose2d fallbackPose;

        //instantiates the swerve modules using motor Ids and offsets.
        SwerveModule m_frontLeft {kFrontLeftTurningCanId,  kFrontLeftDrivingCanId,  kFrontLeftChassisAngularOffset,  false};
        SwerveModule m_frontRight{kFrontRightTurningCanId, kFrontRightDrivingCanId, kFrontRightChassisAngularOffset, false};
        SwerveModule m_rearLeft  {kRearLeftTurningCanId,   kRearLeftDrivingCanId,   kRearLeftChassisAngularOffset,   false};
        SwerveModule m_rearRight {kRearRightTurningCanId,  kRearRightDrivingCanId,  kRearRightChassisAngularOffset,  false};

        //helpers to turn desired moving in the swerve drive into individual
        //module states for each module to get a targeted angle and speed.
        frc::SwerveDriveKinematics<4> kDriveKinematics{
                frc::Translation2d{DriveConstants::kWheelBase / 2,
                                DriveConstants::kTrackWidth / 2},
                frc::Translation2d{DriveConstants::kWheelBase / 2,
                                -DriveConstants::kTrackWidth / 2},
                frc::Translation2d{-DriveConstants::kWheelBase / 2,
                                DriveConstants::kTrackWidth / 2},
                frc::Translation2d{-DriveConstants::kWheelBase / 2,
                                -DriveConstants::kTrackWidth / 2}};
                                
        //instantiating an array of all states
        std::array<frc::SwerveModuleState, 4> states;

        //instantiates the pose estimator that reads the positions of the swerve modules
        //also adds standard deviation for the values that odometry recieves
        //we increase the rotation stdDev to 100 because we should not
        //typically use it, and instead use the gyro.
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        kDriveKinematics,
        frc::Rotation2d{},
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
        frc::Pose2d{},
        {0.1, 0.1, 0.1},
        {1.0, 1.0, 100.0}
        };

        frc::ChassisSpeeds m_lastCommandedRobotRelativeSpeeds{0.0_mps, 0.0_mps, 0_rad_per_s};
        frc::ChassisSpeeds m_simFieldRelativeSpeeds{0.0_mps, 0.0_mps, 0_rad_per_s};
        frc::ChassisSpeeds m_simRobotRelativeSpeeds{0.0_mps, 0.0_mps, 0_rad_per_s};
        frc::Pose2d m_simPose{};

        nt::DoublePublisher m_simPoseXPublisher =
            nt::NetworkTableInstance::GetDefault()
                .GetDoubleTopic("/Simulation/Drive/EstimatedPose/XMeters")
                .Publish();
        nt::DoublePublisher m_simPoseYPublisher =
            nt::NetworkTableInstance::GetDefault()
                .GetDoubleTopic("/Simulation/Drive/EstimatedPose/YMeters")
                .Publish();
        nt::DoublePublisher m_simPoseHeadingPublisher =
            nt::NetworkTableInstance::GetDefault()
                .GetDoubleTopic("/Simulation/Drive/EstimatedPose/HeadingDegrees")
                .Publish();
        nt::DoublePublisher m_simHeadingPublisher =
            nt::NetworkTableInstance::GetDefault()
                .GetDoubleTopic("/Simulation/Drive/HeadingDegrees")
                .Publish();
        nt::DoublePublisher m_simPoseHeadingRadiansPublisher =
            nt::NetworkTableInstance::GetDefault()
                .GetDoubleTopic("/Simulation/Drive/EstimatedPose/HeadingRadians")
                .Publish();
        nt::DoublePublisher m_simHeadingRadiansPublisher =
            nt::NetworkTableInstance::GetDefault()
                .GetDoubleTopic("/Simulation/Drive/HeadingRadians")
                .Publish();

};
