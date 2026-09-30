#pragma once

#include "wrappers/REVBase.h"

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "wrappers/NovaBase.h"

    class SwerveModule
    {
        public:
            SwerveModule(int steer_id, int drive_id, double offset, bool steerEncoderInverted = false);

            frc::SwerveModuleState GetState();
            frc::SwerveModulePosition GetPosition();
            
            frc::SwerveModuleState GetDesiredState();
            void SetDesiredState(const frc::SwerveModuleState &desiredState);
            void ResetEncoders(); 
            double getInternalPosition();
            void FlipEncoder(); 
            double GetVelocity();
            void SimulationPeriodic();

            double GetRelativePosition();
        private:
            frc::SwerveModuleState correctedDesiredState{};

            SparkFlexMotor m_driveMotor;
            NovaMotor m_steerMotor;

            //software zero for the drive encoder, in motor rotations.
            double drivePositionOffset = 0.0;

            frc::Rotation2d offset;
            bool steerEncoderInverted;

            double GetSteerRotations();

    };
