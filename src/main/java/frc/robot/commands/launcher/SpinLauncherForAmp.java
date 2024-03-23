// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.notehandler.LauncherWheels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinLauncherForAmp extends InstantCommand {
  private final LauncherWheels launcherWheels;
  public SpinLauncherForAmp(LauncherWheels launcherWheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcherWheels);
    this.launcherWheels = launcherWheels;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherWheels.setLaunchPower(.25);
  }
}
