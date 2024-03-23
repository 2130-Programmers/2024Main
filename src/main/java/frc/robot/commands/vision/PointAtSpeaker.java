// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PiVision;

public class PointAtSpeaker extends Command {
  private PiVision vision;
  private DriveTrain driveTrain;
  private double angleError;
  /** Creates a new PointAtAprilTag. */
  public PointAtSpeaker(PiVision vision, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, driveTrain);
    this.vision = vision;
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = vision.angleToSpeaker();
    driveTrain.calculateKinematics(RobotContainer.driverGamepad.getLeftX() * -1,  RobotContainer.driverGamepad.getLeftY(), angleError * .005);
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
