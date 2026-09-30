#include "subsystems/VisionSubsystem.h"


VisionSubsystem::VisionSubsystem() : kTagLayout(path.string())
{
    cameras.emplace_back("Front");
    cameras.emplace_back("Left");
    cameras.emplace_back("Back");
    cameras.emplace_back("Right");

    frc::SmartDashboard::PutBoolean("has value", false);
}


void VisionSubsystem::setGyroData(frc::Rotation2d gyroRotation)
{
    //checker to investiage if the gyro is a component of our explosion odometry
    //issue, but still good to check in comp code in general
    if (std::isnan(gyroRotation.Degrees().value()) == true) {
        return;
    }
    for (auto& estimator : poseEstimator) {
        estimator.AddHeadingData(frc::Timer::GetFPGATimestamp(),  gyroRotation);
    }
}

void VisionSubsystem::zeroPose()
{
    for (auto& estimator : poseEstimator) {
        estimator.SetLastPose(frc::Pose3d(frc::Translation3d{0_m,0_m,0_m},frc::Rotation3d{0_deg}));
    }
}

std::vector<photon::EstimatedRobotPose> VisionSubsystem::getEstimatedGlobalPose(frc::Pose3d& prevEstimatedRobotPose) {

    //All the unread results get compiled into our unreadResults vector that contains all the unread results formatted per each individual camera
    std::vector<std::optional<photon::PhotonPipelineResult>> unreadResults;

    double totalTagDistance = 0.0;
    int totalTagCount = 0;
    double averageTagDistance = 0;

    for (size_t i = 0; i < cameras.size(); i++) {
        std::vector<photon::PhotonPipelineResult> unreadResultsVector = cameras.at(i).GetAllUnreadResults();
        if (!unreadResultsVector.empty()) {
            unreadResults.push_back(unreadResultsVector.back());
        } else {
            unreadResults.push_back(std::nullopt);
        }
    }

    //instantiates the total amount of poses we are recieving from the cameras formatted per camera.
    std::vector<photon::EstimatedRobotPose> poses;

    //recieving the current time of the match
    units::second_t currentTime = frc::Timer::GetFPGATimestamp();
    
    for (size_t i = 0; i < unreadResults.size(); i++) {
        //If the unread results for camera array isnt empty, and it has a viable target with low enough pose ambiguity, set the camera results to that pose.
        
        photon::PhotonPipelineResult culledUnreadResult;

        if (!unreadResults.at(i).has_value()) {
            continue;
        } else {
            culledUnreadResult = unreadResults.at(i).value();
        }

        
        if (culledUnreadResult.HasTargets()) {

            //The results of processing give us our estimated robot positions for each camera.
            //sets the component of the cameraResults and adds the results of that specific camera to the array
            std::optional<photon::PhotonPipelineResult> cameraResults = unreadResults.at(i);

            //if the index of that cameraResults does not have a value, skip to the next camera.
            if (!cameraResults.has_value()) {
                continue;
            }

            //creates a culledResult member of data that exists to prevent null values from going into our poses
            photon::PhotonPipelineResult culledResult{cameraResults.value()};

            //if the culledResult does not have a value (interpreted as -1) skip to the next camera.
            if (culledResult.GetTimestamp().value() == -1) {
                continue;
            }
            
            if (!culledUnreadResult.HasTargets())
            {
                continue;
            }

            //If the best target is above our current tolerance of pose ambiguity, skip to next camera.
            if (culledUnreadResult.GetBestTarget().GetPoseAmbiguity() > 0.1) {
                continue;
            }

            //If the best target is more than 4 meters away and single tag, skip
            if (culledUnreadResult.GetBestTarget().GetBestCameraToTarget().Translation().Norm().value() > 3.0) {
                continue;
            }

            bool singleTarget = false;
            if (culledUnreadResult.GetTargets().size() < 2) {
                singleTarget  = true;
            }

            //gets frameTime to measure the time of the taken poseEstimate.
            units::second_t frameTime{culledResult.GetTimestamp().value()};
                    
            //if the frame time is within the given alloted time frame that exists, let this happen.
            if (frameTime > lastProcessedTime && frameTime <= currentTime) {
                frc::SmartDashboard::PutBoolean("has value", true);

                //instantiates our wrapper for our estimatedRobotPoses, evaluates if the data they are going to contain exists.
                std::optional<photon::EstimatedRobotPose> optionalPose;
                if (!singleTarget) {
                    optionalPose = poseEstimator.at(i).EstimateCoprocMultiTagPose(cameraResults.value());
                } else if (singleTarget) {
                    optionalPose = poseEstimator.at(i).EstimateLowestAmbiguityPose(cameraResults.value());
                }

            if (optionalPose.has_value())
            {
                m_field.SetRobotPose(frc::Pose2d(optionalPose->estimatedPose.X(),optionalPose->estimatedPose.Y(), 0_deg));
                frc::SmartDashboard::PutData("Vision Pose", &m_field);
            }
            
                //if the optionalPose has a value, add it to the poses array
                if (optionalPose.has_value()) {

                    //if the estimated pose's coordinates are not finite, ignore
                    if (!std::isfinite(optionalPose->estimatedPose.X().value()) ||
                        !std::isfinite(optionalPose->estimatedPose.Y().value())) {
                        continue;
                    }

                    //if optionalPose has a logical Z value (because Z should always be 0 or close to 0), reject
                    if (/*optionalPose->estimatedPose.Z() < 0.1_m && optionalPose->estimatedPose.Z() > -0.1_m &&*/
                        optionalPose->estimatedPose.X() > 0.1_m && optionalPose->estimatedPose.X() < 17_m &&
                        optionalPose->estimatedPose.Y() > 0.1_m && optionalPose->estimatedPose.Y() < 9_m
                        // true
                    ) {

                        // bool acceptPose = false;

                        // if (!hasPrevVisionPose) acceptPose = true;
                        // else if (optionalPose->estimatedPose.Translation()
                        //                                     .Distance(prevEstimatedRobotPose.Translation()) < 18_m) {
                        //                                         acceptPose = true;
                        //                                     }
                        // if (acceptPose) {
                        //     hasPrevVisionPose = true;

                        poses.push_back(optionalPose.value());
                        prevEstimatedRobotPose = optionalPose->estimatedPose;

                            // This counts the total distance from the tags we measure so that we can find the average distance from the tags. We use this in our standard devs later.
                            for (const auto& target : culledUnreadResult.GetTargets()) {
                                double distance =
                                target.GetBestCameraToTarget()
                                .Translation()
                                .Norm()
                                .value();

                                totalTagDistance += distance;
                                totalTagCount++;
                            }
                        // }
                    }
                }

                lastProcessedTime = frameTime;  // Update last processed time

            } 
                    
            else {
                //fmt::print("Skipping outdated or duplicate frame from Cameras\n");
            }
            
        }
    }

    if (totalTagCount > 0) {
    averageTagDistance = totalTagDistance / totalTagCount;
    }


    return poses;
}
