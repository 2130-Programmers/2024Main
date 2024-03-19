// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PiVision extends SubsystemBase {
  private final PhotonCamera photonPI = new PhotonCamera("Photon rPI");
  private final PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPipelineResult pipelineResult;
  private Pose3d robotPose = new Pose3d();

  /** Creates a new Vision. */
  public PiVision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch(IOException unhandException) {
      System.out.println("Error loading apriltag field layout");
    }

    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonPI, Constants.VisionConstants.CAMERA_RELATIVE_TO_ROBOT);
  }

  /**
   * Update the results from vision processing on RIO
   * Should be called locally by methods that require it
   */
  private void updatePipelineResult() {
    pipelineResult = photonPI.getLatestResult();
  }

  /**
   * Estimate the robots field relative pose
   * @return - pose relative to the blue alliance wall, or null if no targets are tracked
   */
  public Pose3d estimatePose() {
    updatePipelineResult();
    poseEstimator.setLastPose(robotPose);
    Optional<EstimatedRobotPose> optionalPose = poseEstimator.update();
    if(pipelineResult.hasTargets()) {
      try {    
        robotPose = optionalPose.orElseThrow().estimatedPose;
        RobotContainer.driveTrain.driveOdometry.addVisionMeasurement(optionalPose.get().estimatedPose.toPose2d(), optionalPose.get().timestampSeconds);
      } catch (NoSuchElementException noSuchElementException) {
        System.out.println("No pose estmiate avaliable");
      }
    }

    SmartDashboard.putNumber("Vision X", robotPose.getX());
    SmartDashboard.putNumber("Vision Y", robotPose.getY());

    return robotPose;
  }

  /**
   * Get the total distance from camera to target
   * @param target - PhotonTrackedTarget coming from
   * @return the distance in meters from camera to speaker, or 0 if not tracking speaker
   */
  public double getDistanceToTarget() {
    updatePipelineResult();
    if(!pipelineResult.hasTargets()) {
      return 0;
    }
    
    //True if tracking center target on either speaker
    List<PhotonTrackedTarget> currentTargets = pipelineResult.getTargets();
    //Will remain null unless the camera is currently tracking the center target on one of the speakers
    PhotonTrackedTarget speakerCenterTarget = null;
    for(PhotonTrackedTarget current : currentTargets) {
      //If tracking the center target, update center target variable
      if(current.getFiducialId() == 4 || current.getFiducialId() == 7) {
        speakerCenterTarget = current;
      }
    }


    if(speakerCenterTarget != null) {
      //Get the position of the target relative to the robot
      Transform3d targetPosition = speakerCenterTarget.getBestCameraToTarget();
      
      //Pythagorean theorum to get total distance to target
      double distance =  Math.sqrt(targetPosition.getY() * targetPosition.getY() + targetPosition.getX() * targetPosition.getX());
      SmartDashboard.putNumber("Distance to speaker", distance);
      return distance;
    }

    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // estimatePose();
  }
}
