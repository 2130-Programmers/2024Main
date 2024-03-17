// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PiVision;
import frc.robot.subsystems.notehandler.LauncherAngle;

public class AngleFromAprilTag extends Command {
  private final LauncherAngle noteHandler;
  private final PiVision vision;
  /** Creates a new AprilTagLauncher. */
  public AngleFromAprilTag(LauncherAngle noteHandler, PiVision vision) {
    addRequirements(noteHandler, vision);
    this.noteHandler = noteHandler;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteHandler.angleFromDistance(vision.getDistanceToTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteHandler.setRotatePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
