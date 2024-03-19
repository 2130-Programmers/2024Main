// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.notehandler.LauncherAngle;
import frc.robot.subsystems.notehandler.LauncherIntake;
import frc.robot.subsystems.notehandler.LauncherWheels;

public class ManualLauncher extends Command {
  private final LauncherAngle launcherAngle;
  private final LauncherIntake launcherIntake;
  private final LauncherWheels launcherWheels;
  /** Creates a new ManualLauncher. */
  public ManualLauncher(LauncherAngle launcherAngle, LauncherIntake launcherIntake, LauncherWheels launcherWheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcherAngle, launcherIntake, launcherWheels);
    this.launcherAngle = launcherAngle;
    this.launcherIntake = launcherIntake;
    this.launcherWheels = launcherWheels;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherAngle.moveArmFeedForward(launcherAngle.getMeasurement(), RobotContainer.operatorGamepad.getLeftY());
    launcherIntake.setIntakePower(RobotContainer.operatorGamepad.getLeftTriggerAxis());
    launcherWheels.setLaunchPower(RobotContainer.operatorGamepad.getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherAngle.stopAngleMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
