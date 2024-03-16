// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVision;

public class PointAtNote extends Command {
  private DriveTrain driveTrain;
  private LimelightVision limelightVision;
  private double angleError;
  /** Creates a new PointAtNote. */
  public PointAtNote(DriveTrain driveTrain, LimelightVision limelightVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, limelightVision);
    this.driveTrain = driveTrain;
    this.limelightVision = limelightVision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = limelightVision.angleToNearestNote();
    driveTrain.calculateKinematics(RobotContainer.driverGamepad.getLeftX() * -1,  RobotContainer.driverGamepad.getLeftY(), angleError * Constants.DriveTrainConstants.AUTO_ROTATION_GAIN * (1 - limelightVision.getTargetArea()));
    SmartDashboard.putNumber("Angle Error is  ", + angleError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleError) < 1.5;
  }
}
