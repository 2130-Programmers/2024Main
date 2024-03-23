// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.notehandler.LauncherWheels;

public class AutoSpinLauncher extends Command {
  private final LauncherWheels launcherWheels;
  private boolean done;
  /** Creates a new AutoSpinLauncher. */
  public AutoSpinLauncher(LauncherWheels launcherWheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcherWheels);
    this.launcherWheels = launcherWheels;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherWheels.setLaunchPower(1);
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = launcherWheels.launcherAtSpeed(Constants.LauncherConstants.TARGET_LAUNCH_SPEED);
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
