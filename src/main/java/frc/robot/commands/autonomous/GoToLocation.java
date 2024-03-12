// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PiVision;

public class GoToLocation extends Command {
  PiVision vision;
  DriveTrain driveTrain;
  Pose3d targetPose;

  /** Creates a new GoToLocation. */
  public GoToLocation(PiVision vision, DriveTrain driveTrain, Pose3d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, driveTrain);
    this.vision = vision;
    this.driveTrain = driveTrain;
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d currentPose = vision.estimatePose();
    double
    xErr = currentPose.getX() - targetPose.getX(),
    yErr = currentPose.getY() - targetPose.getY(),
    rErr = currentPose.getRotation().getAngle() - targetPose.getRotation().getAngle();

    SmartDashboard.putNumber("Position x error", xErr);
    SmartDashboard.putNumber("Position y error", yErr);
    SmartDashboard.putNumber("Position rotation error", rErr);

    driveTrain.calculateKinematics(
      xErr * Constants.DriveTrainConstants.AUTO_TRANSLATION_GAIN,
      yErr * Constants.DriveTrainConstants.AUTO_TRANSLATION_GAIN,
      rErr * Constants.DriveTrainConstants.AUTO_ROTATION_GAIN
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
