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

  final Transform3d kRobotToRearCam;
  final Transform3d kRobotToLeftCam;
  final Transform3d kRobotToRightCam;

  PhotonPoseEstimator rearPhotonEstimator;
  PhotonPoseEstimator leftPhotonEstimator;
  PhotonPoseEstimator rightPhotonEstimator;
  
  PhotonCamera rearCam;
  PhotonCamera leftCam;
  PhotonCamera rightCam;
  
  private final EstimateConsumer estConsumer;
  
  // /** Creates a new Vision. 
  //  * @param <EstimateConsumer> */
  public Vision(EstimateConsumer estConsumer) {

    this.estConsumer = estConsumer;

    kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);


    // Creating the locations for each of the 3 cameras on the robot
    kRobotToRearCam = new Transform3d(
      new Translation3d(-0.178, 0, 0.14),
      new Rotation3d(0, -Math.PI / 8, 0)
    );
    kRobotToLeftCam = new Transform3d(
      new Translation3d(-0.232, 0.312, 0.178),
      new Rotation3d(0, -Math.PI / 8, Math.PI / 4)
    );
    kRobotToRightCam = new Transform3d(
      new Translation3d(-0.232, -0.312, 0.178),
      new Rotation3d(0, -Math.PI / 8, -Math.PI / 4)
    );

    rearPhotonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToRearCam);
    leftPhotonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToLeftCam);
    rightPhotonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToRightCam);

    rearCam = new PhotonCamera("Rear Camera");
    leftCam = new PhotonCamera("Left Camera");
    rightCam = new PhotonCamera("Right Camera");

     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Reading results of all 3 cameras and using MultiTag to calculate pose, with
    // lowest ambiguity as the backup pose estimator. Using Estimate Consumer to take in results after
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

    for (PhotonPipelineResult result : rearCam.getAllUnreadResults()) {
      visionEstimate = rearPhotonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = rearPhotonEstimator.estimateLowestAmbiguityPose(result);
      }
    }
    visionEstimate.ifPresent(
      est -> {
        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
      }
    );

    for (PhotonPipelineResult result : leftCam.getAllUnreadResults()) {
      visionEstimate = leftPhotonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = leftPhotonEstimator.estimateLowestAmbiguityPose(result);
      }
    }
    visionEstimate.ifPresent(
      est -> {
        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
      }
    );

    for (PhotonPipelineResult result : rightCam.getAllUnreadResults()) {
      visionEstimate = rightPhotonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = rightPhotonEstimator.estimateLowestAmbiguityPose(result);
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
