// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.notehandler.LauncherIntake;

public class IntakeNote extends Command {
  private LauncherIntake launcherIntake;
  private boolean done = false;

  /** Creates a new IntakeNote. */
  public IntakeNote(LauncherIntake tempSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tempSub);
    launcherIntake = tempSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherIntake.setIntakePower(.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = launcherIntake.notePresent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherIntake.setIntakePower(0);
    RobotContainer.pdh.setNoteLeds(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
