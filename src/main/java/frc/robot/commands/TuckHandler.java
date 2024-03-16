// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

public class TuckHandler extends Command {
  /** Creates a new TuckHandler. */
    NoteHandler noteHandler;
  public TuckHandler() {
    TuckHandler tuckHandler;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
    this.noteHandler = noteHandler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteHandler.moveToAngle(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return( < 1);
  }
}
