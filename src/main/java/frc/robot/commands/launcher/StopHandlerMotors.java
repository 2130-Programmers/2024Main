// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.NoteHandler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopHandlerMotors extends InstantCommand {
  NoteHandler noteHandler;
  public StopHandlerMotors(NoteHandler noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
    this.noteHandler = noteHandler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteHandler.setRotatePower(0);
    noteHandler.setIntakePower(0);
    noteHandler.setLaunchPower(0);
  }
}
