# pragma once 

#include <vector>

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/Filesystem.h>
#include <filesystem>

class VisionSubsystem : public frc2::SubsystemBase {

    public: 

        VisionSubsystem();
        void setGyroData(frc::Rotation2d gyroRotation);
        void zeroPose();

        frc::Field2d m_field;

        //insantiating of camera names
        // photon::PhotonCamera cameraOne{"cameraOne"};
        // photon::PhotonCamera cameraTwo{"cameraTwo"};
        // photon::PhotonCamera cameraThree{"cameraThree"};
        // photon::PhotonCamera cameraFour{"cameraFour"};

        std::vector<photon::PhotonCamera> cameras;


        //TODO get the actual 2026 field
        std::filesystem::path path = 
        frc::filesystem::GetDeployDirectory() + "/2026-rebuilt-welded.json";
        frc::AprilTagFieldLayout kTagLayout;

        frc::Transform3d robotToCamFront =
            frc::Transform3d(frc::Translation3d(-1.0_in, 14.25_in, 20.75_in),
                        frc::Rotation3d(0_rad, -0.261799_rad, 0_rad));

        frc::Transform3d robotToCamLeft =
            frc::Transform3d(frc::Translation3d(-2.75_in, 14.25_in, 20.75_in),
                        frc::Rotation3d(0_rad, -0.261799_rad, 1.570796_rad));

         frc::Transform3d robotToCamBack =
            frc::Transform3d(frc::Translation3d(-0.5_in, 0.97_in, 20.68_in),
                        frc::Rotation3d(0_rad, -0.261799_rad, 3.14159_rad));

        frc::Transform3d robotToCamRight =
            frc::Transform3d(frc::Translation3d(-10.026_in, 2.573_in, 20.68_in),
                        frc::Rotation3d(0_rad, -0.261799_rad, 3.1415_rad + 1.570796_rad));


        photon::PhotonPoseEstimator poseEstimatorOne{kTagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFront};
        photon::PhotonPoseEstimator poseEstimatorTwo{kTagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamLeft};
        photon::PhotonPoseEstimator poseEstimatorThree{kTagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBack};
        photon::PhotonPoseEstimator poseEstimatorFour{kTagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamRight};
        
        //instantiating pose estimation for every camera
        std::vector<photon::PhotonPoseEstimator> poseEstimator = {poseEstimatorOne, poseEstimatorTwo, poseEstimatorThree, poseEstimatorFour};


        //Instantiates the previous estimated robot pose to fallback on if current robot pose isn't working.
        frc::Pose3d prevEstimatedRobotPose = frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0_deg));

        //Gets the estimated global pose of the robot
        //@params prevEstimatedRobotPose frc::Pose3d position of the previous robot pose
        std::vector<photon::EstimatedRobotPose> getEstimatedGlobalPose(frc::Pose3d& prevEstimatedRobotPose); 

        std::optional<photon::EstimatedRobotPose> EstimatedPose();

        //returns the estimated pose of the robot 
        std::optional<frc::Pose3d> GetVisionPose();


        double totalTagDistance = 0.0;
        int totalTagCount = 0;
        double averageTagDistance = 0;

    private: 

        units::second_t lastProcessedTime = 0_s;

        bool hasPrevVisionPose = false;
};