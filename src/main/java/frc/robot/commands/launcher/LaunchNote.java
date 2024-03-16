// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

public class LaunchNote extends Command {
  private NoteHandler noteHandler;
  private boolean done = false;
  /** Creates a new LaunchNote. */
  public LaunchNote(NoteHandler noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(noteHandler);
    this.noteHandler = noteHandler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteHandler.setIntakePower(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = !noteHandler.notePresent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteHandler.setIntakePower(0);
    noteHandler.setLaunchPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
