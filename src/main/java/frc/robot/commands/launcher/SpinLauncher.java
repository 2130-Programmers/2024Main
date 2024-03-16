// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

public class SpinLauncher extends Command {
  private NoteHandler noteHandler;
  private boolean done = false;
  private double rpm;
  /** Creates a new SpinLauncher. */
  public SpinLauncher(NoteHandler noteHandler, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
    this.noteHandler = noteHandler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteHandler.setLaunchRpm(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = noteHandler.launcherAtSpeed(rpm);
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
