// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.notehandler.LauncherIntake;
import frc.robot.subsystems.notehandler.LauncherWheels;

public class LaunchNote extends Command {
  private final LauncherIntake launcherIntake;
  private final LauncherWheels launcherWheels;
  private boolean done = false, firstNoteTriggered = false;
  /** Creates a new LaunchNote. */
  public LaunchNote(LauncherIntake launcherIntake, LauncherWheels launcherWheels) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(launcherIntake, launcherWheels);
    this.launcherIntake = launcherIntake;
    this.launcherWheels = launcherWheels;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherIntake.setIntakePower(1);
    done = false;
    firstNoteTriggered = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean note = launcherIntake.notePresent();
    if(!note && !firstNoteTriggered) {
      firstNoteTriggered = true;
    } else if (note && firstNoteTriggered) {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherIntake.setIntakePower(0);
    launcherWheels.setLaunchPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
