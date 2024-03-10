// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  final PhotonCamera photonPI = new PhotonCamera("Photon rPI");
  final PhotonPoseEstimator poseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPipelineResult pipelineResult;
  Pose3d robotPose;

  /** Creates a new Vision. */
  public Vision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch(IOException unhandException) {
      System.out.println("Error loading apriltag field layout");
    }

    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.CAMERA_RELATIVE_TO_ROBOT);
  }

  /**
   * Update the results from vision processing on RIO
   */
  public void updatePipelineResult() {
    pipelineResult = photonPI.getLatestResult();
  }

  /**
   * Use the data gathered from apriltags to estimate the robot's pose relative to the BLUE ALLIANCE wall
   */
  public Pose3d estimatePose() {
    updatePipelineResult();
    if(pipelineResult.hasTargets()) {
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(pipelineResult.getBestTarget().getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(pipelineResult.getBestTarget().getFiducialId()).get(), Constants.VisionConstants.CAMERA_RELATIVE_TO_ROBOT);
    }
    return robotPose;
  }

  /**
   * Get the total distance from camera to target
   * @param target - PhotonTrackedTarget coming from
   * @return the distance in meters from camera to target
   */
  public double getDistanceToTarget() {
    if(pipelineResult.hasTargets()) {
      //Get the position of the target relative to the robot
      Transform3d targetPosition = pipelineResult.getBestTarget().getBestCameraToTarget();
      
      //Pythagorean theorum to get total distance to target
      return Math.sqrt(targetPosition.getY() * targetPosition.getY() + targetPosition.getX() * targetPosition.getX());
    }
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePipelineResult();
    SmartDashboard.putNumber("Reported distance to apriltag in meters: ", getDistanceToTarget());
  }
}
