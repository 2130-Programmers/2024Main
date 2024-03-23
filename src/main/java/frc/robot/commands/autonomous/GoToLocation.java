// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class GoToLocation extends Command {
  private boolean done = false;
  private DriveTrain driveTrain;
  private Pose2d targetPose;

  /** Creates a new GoToLocation. */
  public GoToLocation(DriveTrain driveTrain, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveTrain.driveOdometry.getEstimatedPosition();
    double
    xErr = currentPose.getX() - targetPose.getX(),
    yErr = currentPose.getY() - targetPose.getY(),
    rErr = currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians();

    driveTrain.altDrive(xErr, yErr, rErr * Constants.DriveTrainConstants.AUTO_ROTATION_GAIN, Constants.DriveTrainConstants.AUTO_TRANSLATION_GAIN);

    done = (Math.abs(xErr) < .1 && Math.abs(yErr) < .1 && Math.abs(rErr) < .25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.calculateKinematics(0, 0, 0);
    driveTrain.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
