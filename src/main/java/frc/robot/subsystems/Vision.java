// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
  static final PhotonCamera photonLimelight = new PhotonCamera("Photon Limelight");
  static final PhotonPoseEstimator poseEstimator;
  static PhotonPipelineResult pipelineResult;
  static Translation2d camRelativeTarget;
  static Pose3d robotPosition;

  /** Creates a new Vision. */
  public Vision() {
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null)
  }

  /**
   * Update the results from vision processing on RIO
   */
  void updatePipelineResult() {
    pipelineResult = photonLimelight.getLatestResult();
  }

  /**
   * Calculate distance to apriltag
   * @return - the distance in meters
   */
  double getDistanceToTarget() {
    return PhotonUtils.calculateDistanceToTargetMeters(
    Constants.VisionConstants.CAMERA_HEIGHT_METERS,
    Constants.VisionConstants.TARGET_HEIGHT_METERS,
    Constants.VisionConstants.CAMERA_PITCH_RADIANS,
    Units.degreesToRadians(pipelineResult.getBestTarget().getPitch())
    );
  }

  void estimatePose() {
    updatePipelineResult();
    if(pipelineResult.hasTargets()) {
       camRelativeTarget = PhotonUtils.estimateCameraToTargetTranslation(getDistanceToTarget(), Rotation2d.fromDegrees(-target.getYaw()));
    }



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
