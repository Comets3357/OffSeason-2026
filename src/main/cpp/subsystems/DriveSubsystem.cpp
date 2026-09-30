#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <redux/canand/CanandEventLoop.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <algorithm>
#include <cmath>
#include <exception>

#include "Constants.h"

using namespace pathplanner;

constexpr double SPEED_VISION_WEIGHT = 6;
constexpr double DISTANCE_VISION_WEIGHT = 1.2;
constexpr double STANDARD_DEVIATION_WEIGHT = 0.12;

// Steps a value toward a target without changing by more than maxDelta in one update.
double DriveSubsystem::Simulation::MoveTowards(double current, double target, double maxDelta) {
    if (current < target) {
        return std::min(current + maxDelta, target);
    }

    return std::max(current - maxDelta, target);
}

frc::ChassisSpeeds DriveSubsystem::Simulation::LimitFieldSpeeds( frc::ChassisSpeeds targetFieldSpeeds, 
                                                                 frc::ChassisSpeeds currentFieldSpeeds, 
                                                                 units::second_t dt) {

    double deltaVx = targetFieldSpeeds.vx.value() - currentFieldSpeeds.vx.value();
    double deltaVy = targetFieldSpeeds.vy.value() - currentFieldSpeeds.vy.value();
    const double deltaLinearSpeed = std::hypot(deltaVx, deltaVy);

    //evaluates the highest alloted acceleration over a specified dt interval
    const double maxAcceleration =
        DriveSimulationConstants::kMaxLinearAcceleration.value() * dt.value();

    //if the acceleration we are feeding the simulation is below max
    //linearly scale the acceleration based on how much we are feeding it.
    if (deltaLinearSpeed > maxAcceleration && deltaLinearSpeed > 0.0) {
        const double scale = maxAcceleration / deltaLinearSpeed;
        deltaVx *= scale;
        deltaVy *= scale;
    }

    //evaluates the target omega by taking in the recieved target speed and clamping the values.
    const double targetOmega = std::clamp(
        targetFieldSpeeds.omega.value(),
        -DriveSimulationConstants::kMaxAngularVelocity.value(),
        DriveSimulationConstants::kMaxAngularVelocity.value());

    //Once we have a target omega value, linearly interpolate that
    //value over time with our calculated max acceleration value.
    const double currentOmega = MoveTowards(
        currentFieldSpeeds.omega.value(),
        targetOmega,
        DriveSimulationConstants::kMaxAngularAcceleration.value() * dt.value());

    return frc::ChassisSpeeds{
        units::meters_per_second_t{currentFieldSpeeds.vx.value() + deltaVx},
        units::meters_per_second_t{currentFieldSpeeds.vy.value() + deltaVy},
        units::radians_per_second_t{currentOmega}};
}

frc::Pose2d DriveSubsystem::Simulation::IntegratePose(frc::Pose2d currentPose,
                                                      frc::ChassisSpeeds fieldRelativeSpeeds,
                                                      units::second_t dt) {

    //pulls out and clamps the omega value recieved from the fieldRelativeSpeeds chassisSpeeds object.
    const units::radians_per_second_t omega {
        frc::ApplyDeadband(
            fieldRelativeSpeeds.omega.value(),
            DriveSimulationConstants::kOmegaDeadband.value(),
            9999.0)};

    //evaluates the max omega delta.
    const double targetHeadingChange = (omega * dt).value();
    const double maxHeadingChange =
        DriveSimulationConstants::kMaxAngularVelocity.value() * dt.value();

    //clamps the omega value to calculated max values.
    const double clampedHeadingChange = std::clamp(
        targetHeadingChange,
        -maxHeadingChange,
        maxHeadingChange);

    //once we calculated our omega value, export that rotation + translation into a pose2d.
    return frc::Pose2d{
        frc::Translation2d{
            currentPose.X() + fieldRelativeSpeeds.vx * dt,
            currentPose.Y() + fieldRelativeSpeeds.vy * dt},
        currentPose.Rotation() + frc::Rotation2d{units::radian_t{clampedHeadingChange}}};
}

DriveSubsystem::DriveSubsystem()
{
    frc::SmartDashboard::PutData("Drive Estimated Pose", &m_field);
    frc::SmartDashboard::PutData("Drive Estimated Pose Mirror", &m_mirrorField);
    m_simPose = frc::Pose2d{};

    ConfigureAuton();
}
void DriveSubsystem::ConfigureAuton()
{
    RobotConfig config = RobotConfig::fromGUISettings();
    const PIDConstants translationPid{5.0, 0.0, 0.0};
    const PIDConstants rotationPid{5.0, 0.0, 0.0};

    AutoBuilder::configure(
        [this](){ return GetEstimatedPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // PathPlanner gives us the auto start pose; trust it.
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ DriveFromChassisSpeeds(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            translationPid, // Translation PID constants
            rotationPid // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // These paths are currently drawn on the red side of the field, so we
            // flip when running blue. If you redraw paths on blue later, swap this
            // back to kRed to match the standard PathPlanner examples.

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kBlue;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void DriveSubsystem::DriveFromChassisSpeeds(frc::ChassisSpeeds speed, bool fieldRelative)
{
    //commands the base drive function to take in values from the chassisSpeeds object.
    Drive(speed.vx, speed.vy, speed.omega, fieldRelative); 

    //output commanded chassis speeds values.
    frc::SmartDashboard::PutNumber("Commanded Chassis speeds X", speed.vx.value());
    frc::SmartDashboard::PutNumber("Commanded Chassis speeds Y", speed.vy.value());
    frc::SmartDashboard::PutNumber("Commanded Chassis speeds Omega", speed.omega.value());

    const frc::ChassisSpeeds fieldSpeeds =
        fieldRelative
            ? speed
            : frc::ChassisSpeeds::FromRobotRelativeSpeeds(speed, GetGyroHeading());
    frc::SmartDashboard::PutNumber("Commanded Field speeds X", fieldSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("Commanded Field speeds Y", fieldSpeeds.vy.value());

}

void DriveSubsystem::Periodic()
{
    //put all swerve module states into a states array
    states[0] = m_frontLeft.GetState();
    states[1] = m_frontRight.GetState();
    states[2] = m_rearLeft.GetState();
    states[3] = m_rearRight.GetState();

    //put separate position and angle states into our double_states array
    std::array<double, 8> double_states;

    double_states[0] = states[0].angle.Degrees().value();
    double_states[1] = states[0].speed.value();
    double_states[2] = states[1].angle.Degrees().value();
    double_states[3] = states[1].speed.value();
    double_states[4] = states[2].angle.Degrees().value();
    double_states[5] = states[2].speed.value();
    double_states[6] = states[3].angle.Degrees().value();
    double_states[7] = states[3].speed.value();

    // Wrap all double state angles to 0-360
    for (int i=0; i < 7; i += 2) {
        double_states[i] = WrapAngle(double_states[i]);
    }

    //repeatedly calculates our poseEstimator
    PoseEstimation();
    m_vision.setGyroData(GetGyroHeading());

}

void DriveSubsystem::SimulationPeriodic()
{
    // WPILib calls SimulationPeriodic every 20 ms in normal desktop sim.
    // Keeping this fixed makes the sim deterministic and matches the motor sim timestep.
    constexpr units::second_t dt = 20_ms;

    //instantiating simulation periodic for the modules, which in turn simulate the motor controllers.
    m_frontLeft.SimulationPeriodic();
    m_frontRight.SimulationPeriodic();
    m_rearLeft.SimulationPeriodic();
    m_rearRight.SimulationPeriodic();

    const frc::ChassisSpeeds previousFieldSpeeds = m_simFieldRelativeSpeeds;

    // Convert the robot-relative drive command into field-relative motion once per
    // tick. Field-frame translation keeps the robot driving along the path vector
    // while heading changes independently.
    const frc::ChassisSpeeds targetFieldSpeeds =
        frc::ChassisSpeeds::FromRobotRelativeSpeeds(
            m_lastCommandedRobotRelativeSpeeds,
            m_simPose.Rotation());

    m_simFieldRelativeSpeeds =
        Simulation::LimitFieldSpeeds(targetFieldSpeeds, previousFieldSpeeds, dt);

    m_simPose = Simulation::IntegratePose(m_simPose, m_simFieldRelativeSpeeds, dt);

    // PathPlanner wants robot-relative speeds, so convert the sim's field motion
    // back into the robot frame after updating the heading.
    m_simRobotRelativeSpeeds =
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            m_simFieldRelativeSpeeds.vx,
            m_simFieldRelativeSpeeds.vy,
            m_simFieldRelativeSpeeds.omega,
            m_simPose.Rotation());

    //resets the pose of the robot to the currently simulated position
    //TODO maybe refactor all of this to read sensor positions, or atleast feed these values
    //into the drive class to get tactile sensor feedback.
    m_poseEstimator.ResetPosition(
        m_simPose.Rotation(),
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
         m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
        m_simPose);

    //keep the gyro's simulated heading aligned with the pose estimator.
    m_gyro.SetSimState(m_poseEstimator.GetEstimatedPosition().Rotation());

    // Publish the sim pose separately for dashboards/logging without extra module telemetry spam.
    PublishSimEstimatedPose();
}

double DriveSubsystem::WrapAngle(double angle) {
    //for all angles, wrap the angle between 0-360 degrees.
    angle = std::fmod(angle, 360.0);
    if (angle < 0.0) angle += 360.0;
    return angle;
}

void DriveSubsystem::PoseEstimation() {
    if (frc::RobotBase::IsSimulation()) {
        // In simulation, the simple chassis model is the main workhor
        // in vision calculations that are meant for the real robot.
        m_field.SetRobotPose(m_simPose);
        m_mirrorField.SetRobotPose(m_simPose.RotateAround(
            frc::Translation2d{8.774176_m, 4.0259_m},
            frc::Rotation2d{180_deg}));
        return;
    }

    m_poseEstimator.Update(m_gyro.GetRotation(),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    
    //this code gets all of our speed data to evaluate the standard deviation of our robotPose and our actual.

    //gets the chassis speeds of the robot and squares it to exponentially increase vision deviation when we are faster
    double chassisSpeedSquared = pow(GetRobotRelativeSpeeds().vx.value(), 2) + pow(GetRobotRelativeSpeeds().vy.value(), 2);
    
    //gets the chassis speeds of the robot to add a linear function to std deviation calculation
    double chassisSpeeds = pow(chassisSpeedSquared, 0.5);

    //recieves the percent speed of the robot
    double percentSpeed = (chassisSpeeds / DriveConstants::kMaxSpeed.value());

    //instantiates our main workhorse for standard deviation
    double stdDev;                                                                                                              

    // gets the frc::chassisSpeeds of the bot and does a conversion factor to convert into angular rotation
    //angularSpeedMulti is in radians per second
    if (std::isfinite(GetRobotRelativeSpeeds().omega())) {
        double angularSpeedMulti = GetRobotRelativeSpeeds().omega / 0.0610865_rad_per_s;
    } else {
        double angilarSpeedMulti = 0;
    }

    // //The higher the angularSpeed of the chassis, the less confident we feel in the pose of our Bot.
    //stdDev = std::ceil(angularSpeedMulti) * 5;

    //gets the distance between our alliance's respective hub.
    double distancePose = GetDistanceFromTarget(IsBlueAlliance() ? blueHub : redHub); 

    //gets distance from every tag measurement
    double avgTagDistance = m_vision.averageTagDistance;

    //Uses average tag distance as well as percent speed
    percentSpeed = std::clamp(percentSpeed, 0.0, 1.0);

    //creating mathematical functions off of the speed of our robot and the average tag distance
    double speedFactor = std::exp(SPEED_VISION_WEIGHT * percentSpeed);
    double distanceFactor = std::exp(DISTANCE_VISION_WEIGHT * avgTagDistance);

    //once we have all these values, do standard deviation calculations
    stdDev = STANDARD_DEVIATION_WEIGHT * speedFactor * distanceFactor;
        
    // if robot is disabled, override all previous logic so that 
    //we can rely on vision measurements (usually when setting up for auto).
    if (frc::DriverStation::IsDisabled()) { stdDev = 0.2; }

    // stdDev = std::clamp(stdDev, 0.5, 10.0);
    m_poseEstimator.SetVisionMeasurementStdDevs({stdDev, stdDev, 100.0});
    
    //end std dev calculation

    std::vector<photon::EstimatedRobotPose> estimatedPoseVector;
    estimatedPoseVector = m_vision.getEstimatedGlobalPose(m_vision.prevEstimatedRobotPose);

    // Updates vision measurements with available poses from each camera
    if (!estimatedPoseVector.empty()) {

        //for all vision measurements within the estimated pose array, give them to the poseEstimator
        for (int i = 0; i < estimatedPoseVector.size(); i++) {
            
            //adds the vision estimate to the poseEstimator.
            m_poseEstimator.AddVisionMeasurement(estimatedPoseVector.at(i).estimatedPose.ToPose2d(), 
                                                 estimatedPoseVector.at(i).timestamp);
        }
    }

    //keep the robot inside the field, but only reset if the pose is outside the bounds.
    const double min = DriveConstants::kWheelBase.value() / 2;
    const double maxX = 16.5 - min;
    const double maxY = 8.0 - min;
    const auto pose = m_poseEstimator.GetEstimatedPosition();

    if (pose.X().value() < min || pose.X().value() > maxX ||
        pose.Y().value() < min || pose.Y().value() > maxY) {
        m_poseEstimator.ResetPose(frc::Pose2d{
            units::meter_t{std::clamp(pose.X().value(), min, maxX)},
            units::meter_t{std::clamp(pose.Y().value(), min, maxY)},
            pose.Rotation()});
    }

    //sets the fields in advantageScope to the current estimated position
    m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
    m_mirrorField.SetRobotPose(m_poseEstimator.GetEstimatedPosition().RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg})); 
}

void DriveSubsystem::PublishSimEstimatedPose()
{
    if (!frc::RobotBase::IsSimulation()) {
        return;
    }

    m_simPoseXPublisher.Set(m_simPose.X().value());
    m_simPoseYPublisher.Set(m_simPose.Y().value());
    const double headingDegrees = m_simPose.Rotation().Degrees().value();
    const double headingRadians = m_simPose.Rotation().Radians().value();
    m_simPoseHeadingPublisher.Set(headingDegrees);
    m_simHeadingPublisher.Set(headingDegrees);
    m_simPoseHeadingRadiansPublisher.Set(headingRadians);
    m_simHeadingRadiansPublisher.Set(headingRadians);
    frc::SmartDashboard::PutNumber("Sim/Drive/RobotRelativeVx", m_simRobotRelativeSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("Sim/Drive/RobotRelativeVy", m_simRobotRelativeSpeeds.vy.value());
    frc::SmartDashboard::PutNumber("Sim/Drive/FieldRelativeVx", m_simFieldRelativeSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("Sim/Drive/FieldRelativeVy", m_simFieldRelativeSpeeds.vy.value());
}

 
void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative)
{
    // Keep every drive caller inside the robot's allowed yaw rate. Teleop already scales
    // by this constant, but this also protects autos and helper functions.
    const units::radians_per_second_t limitedRot{
        std::clamp(
            rot.value(),
            -DriveConstants::kMaxAngularSpeed.value(),
            DriveConstants::kMaxAngularSpeed.value())};

    //if the robot is within teleop, lock the wheels if we are under a certain speed
    if (frc::DriverStation::IsTeleop() &&
        units::math::abs(xSpeed) < 0.02_mps &&
        units::math::abs(ySpeed) < 0.02_mps &&
        units::math::abs(limitedRot) < 0.02_rad_per_s) {
            SetX();
            return;
    }

    const frc::ChassisSpeeds rawCommandedSpeeds =
        fieldRelative
            ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                  xSpeed, ySpeed, limitedRot, GetGyroHeading())
            : frc::ChassisSpeeds{xSpeed, ySpeed, limitedRot};

    // The simple sim integrates the desired continuous chassis motion directly.
    // Keep this raw; discretized speeds are only for module setpoints below.
    m_lastCommandedRobotRelativeSpeeds = rawCommandedSpeeds;

    // Convert the commanded speeds into the correct units for the drivetrain
    // Discretizing here keeps real/module motion from skewing over each 20 ms tick,
    // but we do not feed that corrected vector back into the simple pose sim.
    const frc::ChassisSpeeds moduleCommandedSpeeds =
        frc::ChassisSpeeds::Discretize(rawCommandedSpeeds, 20_ms);

    auto states = kDriveKinematics.ToSwerveModuleStates(
        moduleCommandedSpeeds);

    //renormalize the wheel speeds to our given max speed
    kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

    //sets all modules to a desired state
    auto [fl, fr, bl, br] = states;

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_rearLeft.SetDesiredState(bl);
    m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX()
{
    m_lastCommandedRobotRelativeSpeeds = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};

    //sets all module states to 0_mps and set angles to lock the wheels
    m_frontLeft.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{35_deg}});
    m_frontRight.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{-35_deg}});
    m_rearLeft.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{-35_deg}});
    m_rearRight.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{35_deg}});
}

void DriveSubsystem::ResetEncoders()
{
    //resets the encoders of the entire driveBase.
    m_frontLeft.ResetEncoders();
    m_rearLeft.ResetEncoders();
    m_frontRight.ResetEncoders();
    m_rearRight.ResetEncoders();
}

void DriveSubsystem::ZeroHeading() 
{ 
    //zeroes the position of the gyro
    if (frc::RobotBase::IsSimulation()) {
        ResetOdometry(frc::Pose2d{GetEstimatedPose().Translation(), frc::Rotation2d{}});
        return;
    }

    m_gyro.ZeroGyro();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
    if (frc::RobotBase::IsSimulation()) {
        // Report the same limited speed that SimulationPeriodic is integrating.
        // PathPlanner compares this with pose, so command speed and sim speed need
        // to tell the same story.
        return m_simRobotRelativeSpeeds;
    }

    return GetMeasuredRobotRelativeSpeeds();
}

frc::ChassisSpeeds DriveSubsystem::GetMeasuredRobotRelativeSpeeds()
{
    //gets the chassis speeds of the robot by pulling all states
    //from each module then turning them into chassis speeds with a given function.
    return kDriveKinematics.ToChassisSpeeds(
        {m_frontLeft.GetState(), 
         m_frontRight.GetState(),
         m_rearLeft.GetState(), 
         m_rearRight.GetState()
    });
}

frc::Rotation2d DriveSubsystem::GetGyroHeading() {
    if (frc::RobotBase::IsSimulation()) {
        // In sim, the pose estimator is the single source of truth for heading.
        // This keeps PathPlanner from fighting a separately-integrated fake gyro.
        return m_poseEstimator.GetEstimatedPosition().Rotation();
    }

    //returns the heading from the gyro
    return m_gyro.GetRotation();
}

frc::Pose2d DriveSubsystem::GetEstimatedPose() 
{ 
    //gets the pose estimator's pose
    return m_poseEstimator.GetEstimatedPosition(); 
}

double DriveSubsystem::GetDistanceFromTarget(frc::Pose2d target)
{
    //returns the estimated global pose, then finds the distance of that point to a target point.
    return GetEstimatedPose().Translation().Distance(target.Translation()).value(); 
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
    if (frc::RobotBase::IsSimulation()) {
        m_simPose = pose;
        m_lastCommandedRobotRelativeSpeeds = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        m_simFieldRelativeSpeeds = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        m_simRobotRelativeSpeeds = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
    } else {
        //resets the position fed to the poseEstimator
        units::degree_t angle{pose.Rotation().Degrees()};
        m_gyro.SetAngle(angle);
    }

    m_poseEstimator.ResetPosition(
        pose.Rotation(),
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
         m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
        pose);
}

//goes to a given pose on the field via the targetPosition that you want to go to, as well as the max output of the motors
void DriveSubsystem::GoToPosition(frc::Pose2d targetPos, double max_output, 
    units::meter_t translationTolerance, units::degree_t degreeTolerance) {
    
    //recieves the current estimatedPose of the bot
    frc::Pose2d currentPosition = GetEstimatedPose();
 
    //sets the new target pose by calculating the target pose with the vision offset
    frc::Pose2d newTargetPos{targetPos.X(), targetPos.Y(), targetPos.Rotation()};
    
    //frc::Translation2d translate{targetPos.X()-currentPosition.X(), targetPos.Y()-currentPosition.Y()};

    //calculates the difference in position between your current pose and the actual
    double deltaX = (newTargetPos.X() -currentPosition.X()).value();
    double deltaY = (newTargetPos.Y() -currentPosition.Y()).value();

    //the degrees of your current position and your target position.
    double current = currentPosition.Rotation().Degrees().value(); 
    double target = targetPos.Rotation().Degrees().value(); 

    double shortestRotation = 0; 

    //calculates the most efficient angle to rotate to
    double deltaAngle = std::fmod((target-current) + 180, 360) - 180;
    shortestRotation = (deltaAngle < -180) ? deltaAngle + 360 : deltaAngle;

    //PIDs to calculate for the rotation and translation of the bot.
    frc::PIDController positionPID(2,0,0);
    frc::PIDController rotationPID(1,0,0);

    //calculates the velocities of the bot in order to get to said position
    double speedX = positionPID.Calculate(deltaX, 0);
    double speedY = positionPID.Calculate(deltaY, 0);
    double angVel = rotationPID.Calculate(shortestRotation, 0); 

    //reverses the speed if the bot to compensate for different perspective
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance && *alliance == frc::DriverStation::Alliance::kBlue)
    {
        speedX *= -1;
        speedY *= -1;
    }
    
    //calculates the commanded speed that the robot would be going thru this path
    double commandedSpeed = std::sqrt(speedX * speedX + speedY * speedY);

    //if its too high, clamp the speed.
    if (commandedSpeed > max_output)
    {
        speedX = speedX * max_output / commandedSpeed;
        speedY = speedY * max_output / commandedSpeed;
    }

    //if the current location of the robot is within a tolerable margin, stop this function.
    if (deltaX < translationTolerance.value() && deltaY < translationTolerance.value() && deltaAngle < degreeTolerance.value()) {
        return;
    }

    //once we have everything, drive using these calculated values.
    Drive(units::meters_per_second_t{(speedX)}, units::meters_per_second_t{(speedY)}, -units::degrees_per_second_t{angVel}, true);
}


bool DriveSubsystem::IsBlueAlliance()
{
    //recieves the alliance from the driverStation
    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

    //gets the alliance values and sets accordingly, default to blue if no value is recieved from the Driver Station.
    if (!alliance.has_value()) {
        frc::SmartDashboard::PutString("Alliance", "Unknown");
        return true;
    }

    if (*alliance == frc::DriverStation::Alliance::kBlue) {
        frc::SmartDashboard::PutString("Alliance", "Blue");
        return true;
    } else if (*alliance == frc::DriverStation::Alliance::kRed) {
        frc::SmartDashboard::PutString("Alliance", "Red");
        return false;
    } else {
        frc::SmartDashboard::PutString("Alliance", "Error");
        return true;
    } 
}
