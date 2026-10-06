// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants
{
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;
    constexpr units::radians_per_second_t kMaxAngularSpeed = 6.3_rad_per_s;

    constexpr double kDirectionSlewRate = 1.2;  // radians per second
    constexpr double kMagnitudeSlewRate = 1.8;  // percent per second (1 = 100%)
    constexpr double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    constexpr units::meter_t kTrackWidth =
        27_in; // Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase =
        19_in; // Distance between centers of front and back wheels on robot

    // Angular offsets of the modules relative to the chassis in radians
    constexpr double kFrontLeftChassisAngularOffset = -90;
    constexpr double kFrontRightChassisAngularOffset = 0;
    constexpr double kRearLeftChassisAngularOffset = 180;
    constexpr double kRearRightChassisAngularOffset = 90;

    // SPARK MAX CAN IDs
    constexpr int kFrontLeftDrivingCanId = 1;
    constexpr int kRearLeftDrivingCanId = 5;
    constexpr int kFrontRightDrivingCanId = 3;
    constexpr int kRearRightDrivingCanId = 7;

    constexpr int kFrontLeftTurningCanId = 2;
    constexpr int kRearLeftTurningCanId = 6;
    constexpr int kFrontRightTurningCanId = 4;
    constexpr int kRearRightTurningCanId = 8;
} // namespace DriveConstants

namespace DriveSimulationConstants
{
    // Direct tuning knobs for the simplified desktop sim chassis model.
    // These are intentionally easier to reason about than mass/friction/torque estimates.

    // Max change in translational speed per second. Raise this if sim driving feels sluggish.
    constexpr units::meters_per_second_squared_t kMaxLinearAcceleration = 10.0_mps_sq;

    // Max change in rotational speed per second. Raise this if sim turning feels sluggish.
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration = 20_rad_per_s_sq;

    // Rotational resistance that bleeds simulated omega back toward zero each loop.
    // Raise this if the sim spins too freely; lower it if turning feels too damped.
    constexpr units::radians_per_second_squared_t kAngularResistance = 2.5_rad_per_s_sq;

    // Hard cap for simulated body rotation so bad module states cannot create wild spins.
    constexpr units::radians_per_second_t kMaxAngularVelocity = 100_rad_per_s;

    // Ignore tiny angular speeds that usually come from floating-point noise.
    constexpr units::radians_per_second_t kOmegaDeadband = 0.05_rad_per_s;
} // namespace DriveSimulationConstants

namespace ModuleConstants
{
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    constexpr int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    constexpr double kDrivingMotorFreeSpeedRps =
        5676.0 / 60; // NEO free speed is 5676 RPM

    constexpr units::meter_t kWheelDiameter = 0.1016_m;

    constexpr units::meter_t kWheelCircumference =
        kWheelDiameter * std::numbers::pi;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    constexpr double kDrivingMotorReduction = 6.23;

    // (13.0 * 24 * 3) / (36.0 * 18 );
    constexpr double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
        kDrivingMotorReduction;
} // namespace ModuleConstants

namespace AutoConstants
{
    constexpr auto kMaxSpeed = 5.9_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;
    constexpr auto kMaxAngularSpeed = 9.95_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double kPXController = 0.5;
    constexpr double kPYController = 0.5;
    constexpr double kPThetaController = 0.5;

    extern const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints;
} // namespace AutoConstants

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kSecondaryControllerPort = 1;
    constexpr double kDriveDeadband = 0.05;
    constexpr int kDriverForwardAxis = 1;
    constexpr int kDriverStrafeAxis = 0;
    constexpr int kDriverRotateAxis = 2;
    constexpr bool kDriverForwardInverted = true;
    constexpr bool kDriverStrafeInverted = true;
    constexpr bool kDriverRotateInverted = true;
} // namespace OIConstants

namespace PositionConstants
{

}
