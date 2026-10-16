#include "subsystems/ShooterSubsystem.h"
#include <algorithm>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>
#include <cmath>
#include <frc/DriverStation.h>

constexpr double SHOOTER_PITCH_CONVERSION = (1.0/20.0) * (15.0/9.0);
constexpr double SHOOTER_OFFSET = 10; //DEGREES
constexpr double SHOOTER_FORWARD_LIMIT = 450;
constexpr double SHOOTER_REVERSE_LIMIT = 10;

constexpr double PITCH_OFFSET = 10; //DEGREES

constexpr frc::Pose2d blueHub{4.625_m, 4.035_m, 0_rad};
constexpr frc::Pose2d redHub{11.92_m, 4.035_m, 0_rad};

constexpr frc::Pose2d upperBluePass{2_m, 6_m, 0_rad};
constexpr frc::Pose2d lowerBluePass{2_m, 2_m, 0_rad};

constexpr frc::Pose2d upperRedPass{14.5_m, 6_m, 0_rad};
constexpr frc::Pose2d lowerRedPass{14.5_m, 2_m, 0_rad};


ShooterSubsystem::ShooterSubsystem(DriveSubsystem& m_drive, VisionSubsystem& m_vision) : 
    m_vision(m_vision), m_drive(m_drive) {

    //All three motors use the same limits; 32 and 33 follow the leader at 31.
    MotorBase::MotorConfig flywheelConfig;
    flywheelConfig.currentLimitAmps = 60;
    flywheelConfig.enableVoltageCompensation = false; //percent output follows bus voltage
    flywheelConfig.positionConversionFactor = 0.5;
    flywheelConfig.velocityConversionFactor = 0.5;

    primaryFlywheelMotor.ApplyConfiguration(flywheelConfig);
    primaryFlywheelMotor.Set(0);
    primaryFlywheelMotor.SetPID(flywheelP, flywheelI, flywheelD, flywheelFF, 0);

    secondaryFlywheelMotor.ApplyConfiguration(flywheelConfig);
    secondaryFlywheelMotor.SetFollow(kFlywheelLeaderId);
    thirdFlywheelMotor.ApplyConfiguration(flywheelConfig);
    thirdFlywheelMotor.SetFollow(kFlywheelLeaderId);

    shooterPitchMotor.SetInverted(true);
    shooterPitchMotor.SetCurrentLimit(20);
    shooterPitchMotor.SetMaxOutput(0.5);
    shooterPitchMotor.SetAbsoluteWrapping(false);
    shooterPitchMotor.SetAbsoluteConversionFactor(SHOOTER_PITCH_CONVERSION);
    shooterPitchMotor.SetSoftLimitSource(thrifty::Motor::FeedbackSensorType::ABS);
    shooterPitchMotor.SetForwardSoftLimit(20_deg);
    shooterPitchMotor.SetReverseSoftLimit(5.5_deg);
    shooterPitchMotor.EnableForwardSoftLimit(true);
    shooterPitchMotor.SetPID(pitchP, pitchI, pitchD, 0, 0);
    shooterPitchMotor.Set(0);
    shooterPitchMotor.ApplyConfiguration();

    frc::SmartDashboard::PutNumber("Smart Dashboard Flywheel Velocity", 0);
    frc::SmartDashboard::PutNumber("Smart Dashboard Shooter Pitch", 25);
    frc::SmartDashboard::PutNumber("Smart Dashboard Flywheel Compensation", 0);
    frc::SmartDashboard::PutNumber("Smart Dashboard Yaw Compensation", 0);

    shooterLookup.Init();
}

void ShooterSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter Pitch", getHoodPitch());
    frc::SmartDashboard::PutNumber("Shooter Yaw", getHoodYaw());
    frc::SmartDashboard::PutNumber("Secondary Flywheel Current", secondaryFlywheelMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Third Flywheel Current", thirdFlywheelMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Flywheel Velocity", primaryFlywheelMotor.getVelocity().value());
    frc::SmartDashboard::PutNumber("Primary Flywheel Current", primaryFlywheelMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Primary Flywheel Voltage", primaryFlywheelMotor.GetAppliedVoltage());
    frc::SmartDashboard::PutNumber("Recieved Flywheel Velocity Value", getSmartDashboardVelocity());
    frc::SmartDashboard::PutNumber("Recieved Flywheel Speed Value", getSmartDashboardSpeed());
}

void ShooterSubsystem::SimulationPeriodic() {
    primaryFlywheelMotor.SimulationPeriodic();
    shooterPitchMotor.SimulationPeriodic();
    secondaryFlywheelMotor.SimulationPeriodic();
    thirdFlywheelMotor.SimulationPeriodic();
}

void ShooterSubsystem::setFlywheelSpeed(double speed) {
    primaryFlywheelMotor.Set(speed);
}

void ShooterSubsystem::setFlywheelVelocity(double RPMs) {
    if (RPMs == 0) {
        primaryFlywheelMotor.Set(0);
        return;
    }
    primaryFlywheelMotor.SetVelocity(units::revolutions_per_minute_t{RPMs});
}

void ShooterSubsystem::setHoodPitchVelocity(double speed) {
    shooterPitchMotor.Set(speed);
}

void ShooterSubsystem::setHoodPitch(units::degree_t angle) {
    double target = std::clamp(angle.value() - PITCH_OFFSET, 5.5, 19.5);
    shooterPitchMotor.SetAbsolutePosition(units::degree_t{target});
}

double ShooterSubsystem::getHoodPitchVelocity() {
    return shooterPitchMotor.getAbsoluteVelocity().value();
}

double ShooterSubsystem::getPrimaryFlywheelRPM() {
    frc::SmartDashboard::PutNumber("Primary Flywheel RPM", primaryFlywheelMotor.getVelocity().value());
    return primaryFlywheelMotor.getVelocity().value();
}

double ShooterSubsystem::getSecondaryFlywheelRPM() {
    return secondaryFlywheelMotor.getVelocity().value();
}

//returns the current position of the shooter's hood
double ShooterSubsystem::getHoodPitch() {
    return shooterPitchMotor.getAbsolutePosition().value() + PITCH_OFFSET;
}

//returns the angle of the shooter's goal hood position
double ShooterSubsystem::getTargetHoodPitch() {
    return calculateTrajectory(getTargetPose()).pitchAngle.value();
}

//returns the current position of the shooter's hood
double ShooterSubsystem::getHoodYaw() {
    //The static shooter has no yaw motor; report its fixed robot-relative reference.
    return 0;
}

//returns the angle of the shooter's goal hood position
double ShooterSubsystem::getTargetHoodYaw() {
    return calculateTrajectory(getTargetPose()).yawAngle.value();
}

double ShooterSubsystem::getSmartDashboardVelocity() {
    return frc::SmartDashboard::GetNumber("Smart Dashboard Flywheel Velocity", 0);
}

double ShooterSubsystem::getSmartDashboardSpeed() {
    return frc::SmartDashboard::GetNumber("Shooter Flywheel Speed", 0);
}

units::degree_t ShooterSubsystem::getSDTargetPitch() {
    return units::degree_t{frc::SmartDashboard::GetNumber("Smart Dashboard Shooter Pitch", 25)};
}

units::degree_t ShooterSubsystem::getSDTargetYaw() {
    return units::degree_t{frc::SmartDashboard::GetNumber("Smart Dashboard Shooter Yaw", 0)};
}

void ShooterSubsystem::changeOperatorFlywheelCompensation(double value) {
    flywheelOperatorOffset += value;
}

void ShooterSubsystem::resetOperatorFlywheelCompensation() {
    flywheelOperatorOffset = 0;
}

double ShooterSubsystem::getOperatorFlywheelCompensation() {
    return flywheelOperatorOffset;
}

bool ShooterSubsystem::isYawWithinRange(double angle) {
    if (angle >= SHOOTER_REVERSE_LIMIT - SHOOTER_OFFSET &&
        angle <= SHOOTER_FORWARD_LIMIT - SHOOTER_OFFSET) {
        return true;
    } else return false;
}

frc::Pose2d ShooterSubsystem::getTargetPose() {
    // Translation2ds for the hubs of the field.
    //
    // Assuming that 0,0 is the bottom left of the Blue Alliance's side of the field:
    // Position of the Blue Alliance's hub is 156.8 (distance from windows to the hub) + 23.5 (width of hub / 2)
    // 182.1 inches converted to meters is roughly 4.625 meters.
    //
    // 8.07 meters is roughly the width of the field, so the Y for both alliances will be roughly 4.035 meters.
    //
    // Red Alliance's hub will be 16.54 meters subtracted by 4.625 meters, which roughly equals 11.92
    //
    // Overall: Blue Alliance Hub (4.625, 4.035), Red Alliance Hub (11.92, 4.035).

    // for types Pose2d and Translation2d, you need to specify what units your values are in, so for here,
    // I need to add "_m" to specify the units that im using is in meters, and _rad to designate im using radians.
    frc::DriverStation::Alliance alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

    //Suppliers for estimated robot pose values
    frc::Pose2d robotPose = m_drive.GetEstimatedPose();

    //if your alliance is within their respective alliance zone, return the respective alliance's hub pose.
    //else, calculate the closer passsing point to point towards.
    if (alliance == frc::DriverStation::Alliance::kBlue) {
        if (robotPose.X() < 5_m && robotPose.X() > 0_m) {
            return blueHub;
        } else if (robotPose.Y() < 4_m) {
            return lowerBluePass;
        } else {
            return upperBluePass;
        }
    }

    if (alliance == frc::DriverStation::Alliance::kRed) {
        if (robotPose.X() < 16.5_m && robotPose.X() > 11.5_m) {
            return redHub;
        } else if (robotPose.Y() < 4_m) {
            return lowerRedPass;
        } else {
            return upperRedPass;
        }
    }

    return blueHub;
}

frc::Pose2d ShooterSubsystem::getSetPoints(char side) {
    
    frc::DriverStation::Alliance alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
    //if your alliance is within their respective alliance zone, return the respective alliance's hub pose.
    //else, calculate the closer passsing point to point towards.
    if (alliance == frc::DriverStation::Alliance::kBlue) {
        if (side == 'L') {
            return frc::Pose2d(4.019_m , 7.152_m, 0_rad);
        } else if (side == 'R') {
            return frc::Pose2d(4.019_m , 0.848_m, 0_rad);
        }
    }

    else if (alliance == frc::DriverStation::Alliance::kRed) {
        if (side == 'L') {
            return frc::Pose2d(12.418_m , 7.152_m, 0_rad);
        } else if (side == 'R') {
            return frc::Pose2d(12.418_m , 0.848_m, 0_rad);
        }
    }
    return frc::Pose2d{};
}

//calculates the trajectory of the fuel to the hub
//@param robotPose Pose2d of the robot or the shooter, TODO probably add transformations to the robotpose
//@param alliance Char to designate which alliance's hub to aim at, TODO probably change to enum
//@returns struct containing yaw and pitch angle in radians.
ShooterSubsystem::shootingParams ShooterSubsystem::calculateTrajectory(frc::Pose2d targetPose) {

    //instantiating the shooting parameters to return.
    shootingParams params;

    //supplier for the current estimated robot pose
    frc::Pose2d robotPoseUnadjusted = m_drive.GetEstimatedPose();
    
    //we need to convert a robot relative velocity to a field relative to prevent relative issues with
    //the commutation between the robot thinking its going postive x but actually going positive y or
    //another issue etc.
    frc::ChassisSpeeds robotVelocity = m_drive.GetRobotRelativeSpeeds();
    frc::ChassisSpeeds fieldSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotVelocity, m_drive.GetGyroHeading());
    
    //to adjust for the robot rotating while moving, we have to calculate for the tangential velocity of the robot
    //to adjust for shooting in addition with robot translation.
    double omega = robotVelocity.omega.value();

    //uses twist2d to add phase delay for more accurate pose.
    frc::Pose2d robotPose = robotPoseUnadjusted.Exp(frc::Twist2d(units::meter_t(robotVelocity.vx.value() * phaseDelay),
                                                                 units::meter_t(robotVelocity.vy.value() * phaseDelay),
                                                                 units::radian_t(omega * phaseDelay)));
 
    //supplier for the current gyro heading
    double gyroHeading = m_drive.GetGyroHeading().Radians().value() + (omega * 0.03);

    //turret rotation relative to the zero position of the robor in rotational space
    double turretHeading = robotPose.Rotation().Radians().value() +
        std::atan2(robotRelativeTurretTransform.Y().value(), robotRelativeTurretTransform.X().value());

    //finds the absolute field centric coordinates of the turret on the field
    double turretVector = std::hypot(robotRelativeTurretTransform.X().value(), robotRelativeTurretTransform.Y().value());
    //pose of the turret on the field, including its offset from robot center.
    frc::Pose2d turretPose = robotPose.TransformBy(robotRelativeTurretTransform);

    // finds the distance between the x and y values of the current turretPose and the hub poses.
    double dx = turretPose.X().value() - targetPose.X().value();
    double dy = turretPose.Y().value() - targetPose.Y().value();

    //before adjusting for shooting while moving, create a distance assuming the speed of the robot is negligible.
    units::meter_t unadjustedTotalDistance = units::meter_t{std::hypot(dy, dx)};

    //we need to convert the robot relative rotational velocity into field relative, so we have to do
    //some silly calculations to turn them into actual x, y displacement values.
    double rotVelX = turretVector * omega * -std::sin(turretHeading);
    double rotVelY = turretVector * omega * std::cos(turretHeading);

    //instantiation of variables to repeatedly adjust for our iteration table
    units::meter_t adjustedTotalDistance = unadjustedTotalDistance;

    //variables that calculate the rotational and translational adjustments
    double leadX = 0;
    double leadY = 0;

    //the combined x and y after we have all our computated values
    double aimX = dx;
    double aimY = dy;

    //because flight time depends on distance, which then affects how much we have to lead our shots
    //we need to iterate values so that the adjustment converges on a specific target.
    for(int i = 0; i < 4; i++) {
        //for the first iteration, assume we are not moving.
        double flightTime = shooterLookup.calculateFlightTime(adjustedTotalDistance);

        //finds the component of the velocity muliplied by the total flight time to find the displacement for the hub
        leadX = (fieldSpeeds.vx.value() + rotVelX ) * flightTime;
        leadY = (fieldSpeeds.vy.value() + rotVelY ) * flightTime;

        //once we have both the unadjusted robotpose -> target measurements, and the leading displacements when moving
        //combine both to recieve the final distances to cancel out robot movements.
        aimX = dx + leadX;
        aimY = dy + leadY;

        //once all the velocity measurements are added, create a new adjusted distance measurement of our calculated values.
        adjustedTotalDistance = units::meter_t{std::hypot(aimY, aimX)};
    }
    
    //calculating the yaw angle with atan adjustment and accounting for gyro heading
    double yawRadians = (std::atan2(aimY, aimX) - gyroHeading);

      //once we have done all our radian computations, we convert into degrees to output to our shooter
    double yawDegrees = yawRadians * (180 / 3.14159) + 180;

    double cotermPlus = yawDegrees + 360;
    double cotermMinus = yawDegrees - 360;
    double previousShooterYaw = getHoodYaw();


    double candidates[3] = {
        cotermPlus,
        yawDegrees,
        cotermMinus
    };

    std::sort(candidates, candidates + 3, [previousShooterYaw](double a, double b) {
        return std::abs(a - previousShooterYaw) < std::abs(b - previousShooterYaw);
    });

    for (int i = 0; i < 3; i++) {
        if (isYawWithinRange(candidates[i])) {
            yawDegrees = candidates[i];
            break;
        }
    }

    //use our lookup tables to establsh our final values.
    params.pitchAngle = units::degree_t{shooterLookup.calculateHoodAngle(adjustedTotalDistance)};

    params.yawAngle = units::degree_t{yawDegrees + shooterLookup.calculateAngleComp(adjustedTotalDistance) * 
                      shooterLookup.calculateAngleComp(units::degree_t{yawDegrees})};

    params.flywheelRpm = units::revolutions_per_minute_t{shooterLookup.calculateFlywheelVelocity(adjustedTotalDistance) * 
                         shooterLookup.calculateVelocityScalarComp(units::degree_t{yawDegrees})};
        
    params.flywheelRpm += units::revolutions_per_minute_t{getOperatorFlywheelCompensation()};

    //some data we can export
    frc::SmartDashboard::PutNumber("Target Yaw Angle", params.yawAngle.value());
    frc::SmartDashboard::PutNumber("Target Pitch Angle", params.pitchAngle.value());
    frc::SmartDashboard::PutNumber("Target Flywheel Velocity", params.flywheelRpm.value());
    frc::SmartDashboard::PutNumber("Target Distance", unadjustedTotalDistance.value());
    frc::SmartDashboard::PutNumber("Yaw Compensation", shooterLookup.calculateAngleComp(adjustedTotalDistance));
    frc::SmartDashboard::PutNumber("Velocity Compensation", shooterLookup.calculateVelocityComp(adjustedTotalDistance));
    frc::SmartDashboard::PutNumber("Turret X", turretPose.X().value());
    frc::SmartDashboard::PutNumber("Turret Y", turretPose.Y().value());

    frc::SmartDashboard::PutNumber("Adjusted Target Distance", adjustedTotalDistance.value());

    frc::SmartDashboard::PutNumber("Velocity Displacement X", leadX);
    frc::SmartDashboard::PutNumber("Velocity Displacement Y", leadY);

    frc::SmartDashboard::PutNumber("Rotational Displacement X", rotVelX);
    frc::SmartDashboard::PutNumber("Rotational Displacement Y", rotVelY);

    return params;
}


//calculates the trajectory of the fuel to the hub
//@param robotPose Pose2d of the robot or the shooter, TODO probably add transformations to the robotpose
//@param alliance Char to designate which alliance's hub to aim at, TODO probably change to enum
//@returns struct containing yaw and pitch angle in radians.
ShooterSubsystem::shootingParams ShooterSubsystem::calculateTrajectory(frc::Pose2d targetPose, bool debug) {
    if (!debug) return calculateTrajectory(targetPose);


    //instantiating the shooting parameters to return.
    shootingParams params;

    //Suppliers for estimated robot pose values and velocity
    frc::Pose2d robotPose = m_drive.GetEstimatedPose();

    // finds the field-relative position of the turret.
    //pose of the turret on the field, including its offset from robot center.
    frc::Pose2d turretPose = robotPose.TransformBy(robotRelativeTurretTransform);

    // finds the distance between the x and y values of the current turretPose and the hub poses.
    double dx = turretPose.X().value() - targetPose.X().value();
    double dy = turretPose.Y().value() - targetPose.Y().value();


    //calculating the yaw angle with atan adjustment and accounting for gyro heading
    double tempYawAngle = std::atan2(dy, dx) * (180.0 / 3.14159) - m_drive.GetGyroHeading().Degrees().value() + 180.0;
    
    //spin causes some difference between where the shooter is pointing, and the actual output,
    //so we need to add an adjustment lookup table as a sinusoid, because only specific angles alter the yaw
    tempYawAngle -= frc::SmartDashboard::GetNumber("Smart Dashboard Yaw Compensation", 0) 
        * std::cos((tempYawAngle) * 3.14159 / 180.0);

    
    double cotermPlus = tempYawAngle + 360;
    double cotermMinus = tempYawAngle - 360;
    double previousShooterYaw = getHoodYaw();


    double candidates[3] = {
        cotermPlus,
        tempYawAngle,
        cotermMinus
    };

    std::sort(candidates, candidates + 3, [previousShooterYaw](double a, double b) {
        return std::abs(a - previousShooterYaw) < std::abs(b - previousShooterYaw);
    });

    for (int i = 0; i < 3; i++) {
        if (isYawWithinRange(candidates[i])) {
            tempYawAngle = candidates[i];
            break;
        }
    }
    
    params.yawAngle = units::degree_t{tempYawAngle};

    //calculating pitch angle
    params.pitchAngle = units::degree_t{frc::SmartDashboard::GetNumber("Smart Dashboard Shooter Pitch", 0)};

    // add a constant to our flywheel speed because our HSL puts extra spin on the ball that we need to account for.
    params.flywheelRpm = units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Smart Dashboard Flywheel Velocity", 0) + 
        frc::SmartDashboard::GetNumber("Smart Dashboard Flywheel Compensation", 0) * std::sin(params.yawAngle.value() * 3.14159 / 180.0)};

    return params;
}


ShooterSubsystem::shootingParams ShooterSubsystem::calculateTrajectorySetpoints(frc::Pose2d targetPose, frc::Pose2d robotPose) {

    //instantiating the shooting parameters to return.
    shootingParams params;

    
    //we need to convert a robot relative velocity to a field relative to prevent relative issues with
    //the commutation between the robot thinking its going postive x but actually going positive y or
    //another issue etc.
    frc::ChassisSpeeds robotVelocity = m_drive.GetRobotRelativeSpeeds();
    frc::ChassisSpeeds fieldSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotVelocity, m_drive.GetGyroHeading());
    
    //to adjust for the robot rotating while moving, we have to calculate for the tangential velocity of the robot
    //to adjust for shooting in addition with robot translation.
    double omega = robotVelocity.omega.value();
 
    //supplier for the current gyro heading
    double gyroHeading = m_drive.GetGyroHeading().Radians().value() + (omega * 0.03);

    //turret rotation relative to the zero position of the robor in rotational space
    double turretHeading = robotPose.Rotation().Radians().value() +
        std::atan2(robotRelativeTurretTransform.Y().value(), robotRelativeTurretTransform.X().value());

    //finds the absolute field centric coordinates of the turret on the field
    double turretVector = std::hypot(robotRelativeTurretTransform.X().value(), robotRelativeTurretTransform.Y().value());
    //pose of the turret on the field, including its offset from robot center.
    frc::Pose2d turretPose = robotPose.TransformBy(robotRelativeTurretTransform);

    // finds the distance between the x and y values of the current turretPose and the hub poses.
    double dx = turretPose.X().value() - targetPose.X().value();
    double dy = turretPose.Y().value() - targetPose.Y().value();

    //before adjusting for shooting while moving, create a distance assuming the speed of the robot is negligible.
    units::meter_t unadjustedTotalDistance = units::meter_t{std::hypot(dy, dx)};



    //we need to convert the robot relative rotational velocity into field relative, so we have to do
    //some silly calculations to turn them into actual x, y displacement values.
    double rotVelX = turretVector * omega * -std::sin(turretHeading);
    double rotVelY = turretVector * omega * std::cos(turretHeading);

    //instantiation of variables to repeatedly adjust for our iteration table
    units::meter_t adjustedTotalDistance = unadjustedTotalDistance;

    //variables that calculate the rotational and translational adjustments
    double leadX = 0;
    double leadY = 0;

    //the combined x and y after we have all our computated values
    double aimX = dx;
    double aimY = dy;

    //because flight time depends on distance, which then affects how much we have to lead our shots
    //we need to iterate values so that the adjustment converges on a specific target.
    for(int i = 0; i < 4; i++) {
        //for the first iteration, assume we are not moving.
        double flightTime = shooterLookup.calculateFlightTime(adjustedTotalDistance);

        //finds the component of the velocity muliplied by the total flight time to find the displacement for the hub
        leadX = (fieldSpeeds.vx.value() + rotVelX) * flightTime;
        leadY = (fieldSpeeds.vy.value() + rotVelY) * flightTime;

        //once we have both the unadjusted robotpose -> target measurements, and the leading displacements when moving
        //combine both to recieve the final distances to cancel out robot movements.
        aimX = dx + leadX;
        aimY = dy + leadY;

        //once all the velocity measurements are added, create a new adjusted distance measurement of our calculated values.
        adjustedTotalDistance = units::meter_t{std::hypot(aimY, aimX)};
    }
    
    //calculating the yaw angle with atan adjustment and accounting for gyro heading
    double yawRadians = (std::atan2(aimY, aimX) - gyroHeading);

      //once we have done all our radian computations, we convert into degrees to output to our shooter
    double yawDegrees = yawRadians * (180 / 3.14159) + 180;
    
    //spin causes some difference between where the shooter is pointing, and the actual output,
    //so we need to add an adjustment lookup table as a sinusoid, because only specific arcs alter the yaw
    yawDegrees += shooterLookup.calculateAngleComp(adjustedTotalDistance) * std::cos(yawRadians);

    //calculating pitch angle
    params.pitchAngle = units::degree_t{shooterLookup.calculateHoodAngle(adjustedTotalDistance)};


    //If target yaw angle is coterminal, set the target to it.
    if (isYawWithinRange(yawDegrees - 360.0)) {
        params.yawAngle = units::degree_t{yawDegrees} - 360_deg;
    } else if (isYawWithinRange(yawDegrees + 360.0)) {
        params.yawAngle = units::degree_t{yawDegrees} + 360_deg;
    } else {
        params.yawAngle = units::degree_t{yawDegrees};
    }

    if (yawDegrees > 0 && yawDegrees < 180) {
        // add a constant to our flywheel speed because our HSL puts extra spin on the ball that we need to account for.
        params.flywheelRpm = units::revolutions_per_minute_t{shooterLookup.calculateFlywheelVelocity(adjustedTotalDistance) + 
        shooterLookup.calculateVelocityComp(adjustedTotalDistance) * std::sin(params.yawAngle.value() * 3.14159 / 180.0)};

    } else {
        // add a constant to our flywheel speed because our HSL puts extra spin on the ball that we need to account for.
        params.flywheelRpm = units::revolutions_per_minute_t{shooterLookup.calculateFlywheelVelocity(adjustedTotalDistance) + 
        shooterLookup.calculateVelocityScalarComp(adjustedTotalDistance) * shooterLookup.calculateVelocityComp(adjustedTotalDistance) * std::sin(params.yawAngle.value() * 3.14159 / 180.0)};
    }
    
    params.flywheelRpm += units::revolutions_per_minute_t{getOperatorFlywheelCompensation()};

    //some data we can export
    frc::SmartDashboard::PutNumber("Target Yaw Angle", params.yawAngle.value());
    frc::SmartDashboard::PutNumber("Target Pitch Angle", params.pitchAngle.value());
    frc::SmartDashboard::PutNumber("Target Flywheel Velocity", params.flywheelRpm.value());
    frc::SmartDashboard::PutNumber("Target Distance", unadjustedTotalDistance.value());
    frc::SmartDashboard::PutNumber("Yaw Compensation", shooterLookup.calculateAngleComp(adjustedTotalDistance));
    frc::SmartDashboard::PutNumber("Velocity Compensation", shooterLookup.calculateVelocityComp(adjustedTotalDistance));
    frc::SmartDashboard::PutNumber("Turret X", turretPose.X().value());
    frc::SmartDashboard::PutNumber("Turret Y", turretPose.Y().value());

    frc::SmartDashboard::PutNumber("Adjusted Target Distance", adjustedTotalDistance.value());

    frc::SmartDashboard::PutNumber("Velocity Displacement X", leadX);
    frc::SmartDashboard::PutNumber("Velocity Displacement Y", leadY);

    frc::SmartDashboard::PutNumber("Rotational Displacement X", rotVelX);
    frc::SmartDashboard::PutNumber("Rotational Displacement Y", rotVelY);

    return params;
}
