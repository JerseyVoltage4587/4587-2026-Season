// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  
  final AprilTagFieldLayout kTagLayout;

  final Transform3d kRobotToCamOne;
  final Transform3d kRobotToCamTwo;
  final Transform3d kRobotToCamThree;

  PhotonPoseEstimator photonEstimatorOne;
  PhotonPoseEstimator photonEstimatorTwo;
  PhotonPoseEstimator photonEstimatorThree;
  
  PhotonCamera camOne;
  PhotonCamera camTwo;
  PhotonCamera camThree;
  
  private final EstimateConsumer estConsumer;
  
  // /** Creates a new Vision. 
  //  * @param <EstimateConsumer> */
  public Vision(EstimateConsumer estConsumer) {

    this.estConsumer = estConsumer;

    kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);


    // Creating the locations for each of the 3 cameras on the robot
    kRobotToCamOne = new Transform3d(
      new Translation3d(-0.237, 0.2475, 0.467),
      new Rotation3d(0, -Math.PI / 8, -Math.PI / 2)
    );
    kRobotToCamTwo = new Transform3d(
      new Translation3d(-0.237, -0.2475, 0.467),
      new Rotation3d(0, -Math.PI / 8, Math.PI / 2)
    );
    kRobotToCamThree = new Transform3d(
      new Translation3d(-0.2265, -0.19, 0.4607), 
      new Rotation3d(0, -Math.PI / 8, -Math.PI)
    );
        
    photonEstimatorOne = new PhotonPoseEstimator(kTagLayout, kRobotToCamOne);
    photonEstimatorTwo = new PhotonPoseEstimator(kTagLayout, kRobotToCamTwo);
    photonEstimatorThree = new PhotonPoseEstimator(kTagLayout, kRobotToCamThree);

    camOne = new PhotonCamera("ArduCam_1");
    camTwo = new PhotonCamera("ArduCam_2");
    camThree = new PhotonCamera("ArduCam3");

     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Reading results of all 3 cameras and using MultiTag to calculate pose, with
    // lowest ambiguity as the backup pose estimator. Using Estimate Consumer to take in results after
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

    for (PhotonPipelineResult result : camThree.getAllUnreadResults()) {
      visionEstimate = photonEstimatorThree.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = photonEstimatorThree.estimateLowestAmbiguityPose(result);
      }
    }
    visionEstimate.ifPresent(
      est -> {
        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
      }
    );

    for (PhotonPipelineResult result : camOne.getAllUnreadResults()) {
      visionEstimate = photonEstimatorOne.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = photonEstimatorOne.estimateLowestAmbiguityPose(result);
      }
    }
    visionEstimate.ifPresent(
      est -> {
        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
      }
    );

    for (PhotonPipelineResult result : camTwo.getAllUnreadResults()) {
      visionEstimate = photonEstimatorTwo.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = photonEstimatorTwo.estimateLowestAmbiguityPose(result);
      }
    }
    visionEstimate.ifPresent(
      est -> {
        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
      }
    );

    

  }

  // The use of this allows the data from the vision estimate to be treated as parameters 
  // for the addVisionMeasurement function used in the Swerve Subsystem
  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp);
  }

}
