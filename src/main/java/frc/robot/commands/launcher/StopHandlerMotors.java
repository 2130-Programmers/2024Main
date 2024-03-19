// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.notehandler.LauncherAngle;
import frc.robot.subsystems.notehandler.LauncherIntake;
import frc.robot.subsystems.notehandler.LauncherWheels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopHandlerMotors extends InstantCommand {
  LauncherAngle noteHandler;
  LauncherWheels launcherWheels;
  LauncherIntake launcherIntake;
  public StopHandlerMotors(LauncherAngle noteHandler, LauncherWheels launcherWheels, LauncherIntake launcherIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
    this.noteHandler = noteHandler;
    this.launcherWheels = launcherWheels;
    this.launcherIntake = launcherIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteHandler.disable();
    noteHandler.stopAngleMotors();
    launcherIntake.setIntakePower(0);
    launcherWheels.setLaunchPower(0);
  }
}
