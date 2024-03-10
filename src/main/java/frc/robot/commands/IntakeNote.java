// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

public class IntakeNote extends Command {
  private NoteHandler noteHandler;
  private boolean done = false;

  /** Creates a new IntakeNote. */
  public IntakeNote(NoteHandler tempSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tempSub);
    noteHandler = tempSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // done = noteHandler.intakeNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
