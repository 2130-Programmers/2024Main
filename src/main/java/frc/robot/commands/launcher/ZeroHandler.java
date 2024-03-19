// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.notehandler.LauncherAngle;

public class ZeroHandler extends Command {
  private LauncherAngle noteHandler;
  private boolean done = false;
  /** Creates a new ZeroLauncher. */
  public ZeroHandler(LauncherAngle noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
    this.noteHandler = noteHandler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteHandler.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteHandler.setSetpoint(-2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteHandler.stopAngleMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteHandler.getMeasurement() < 0;
  }
}
